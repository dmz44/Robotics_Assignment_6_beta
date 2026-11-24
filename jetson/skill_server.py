#!/usr/bin/env python3
# Sample Code for Robotics_Assignment_6_Beta
# Copyright: 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

# Skill Server for Assignment 6_Beta. Runs ON THE TURTLEBOT JETSON (ROS 2 Foxy, Python 3.8).

# The LLM never publishes /cmd_vel itself. It asks for a SKILL ("turn 90 degrees", "move the arm to home"),
# and this node runs the skill closed-loop with hard safety limits that the LLM cannot change.
# Every message is JSON text in a std_msgs/String, so it passes through rosbridge unchanged:
#   /llm/cmd        IN   {"id": "a1", "skill": "turn", "args": {"angle_deg": 90}}
#   /llm/status     OUT  {"id": "a1", "state": "succeeded" | "failed" | "aborted" | "rejected", "message": "..."}
#   /llm/state      OUT  robot summary, twice per second (pose, arm, gripper, detections, ...)
#   /llm/heartbeat  IN   sent twice per second by the MCP server. Silence for 1.5 s = stop (watchdog)
#   /llm/estop      IN   std_msgs/Bool: true = emergency stop (latched), false = release
# This code will run as-is:  python3 skill_server.py

# Refer to the Lab PowerPoint materials and Appendix of Assignment 3 to learn more about coding on ROS2 and the hardware architecture of Turtlebot3.
# You would need a basic understanding of Python Data Structure and Object Oriented Programming to understand this code.

import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

##############################################################################
# SETTINGS. Do not raise the limits without asking the Doctoral Instructor Assistant.
MAX_SPEED = 0.10            # m/s
MAX_TURN_SPEED = 0.8        # rad/s
MAX_FORWARD = 0.5           # m per drive command
MAX_BACKWARD = 0.2          # m per drive command (the camera cannot see behind the robot)
HEARTBEAT_TIMEOUT = 1.5     # s without heartbeat -> stop
OBSTACLE_STOP = 0.25        # m, forward driving stops if the LiDAR sees something closer (only if /scan exists)
CAMERA_HFOV_DEG = 62.0      # horizontal field of view of the camera
IMAGE_WIDTH = 640           # used if the YOLO message gives pixel boxes without the image size

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4"]
ARM_POSES = {                       # [joint1, joint2, joint3, joint4] in radians
    "home":       [0.0, -1.0, 0.3, 0.7],
    "init":       [0.0, 0.0, 0.0, 0.0],
    "ready":      [0.0, -0.3, 0.2, 0.8],
    "look_left":  [1.2, -0.5, 0.3, 0.6],
    "look_right": [-1.2, -0.5, 0.3, 0.6],
}
GRIPPER_POSITION = {"open": 0.019, "close": -0.010}
##############################################################################


class Aborted(Exception):
    pass


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_detections(text):
    """Turn the YOLO JSON message into a list of {label, conf, cx}. cx: 0 = left edge, 1 = right edge.
    HINT: if your YOLO publisher uses other key names, add them here."""
    data = json.loads(text)
    width = float(data.get("width") or data.get("image_width") or IMAGE_WIDTH)
    result = []
    for d in data.get("detections", []):
        label = d.get("label") or d.get("class_name") or d.get("name")
        conf = d.get("conf", d.get("confidence", d.get("score")))
        cx = d.get("cx")
        box = d.get("bbox") or d.get("box") or d.get("bbox_px")
        if isinstance(box, dict):
            box = [box.get("x1", box.get("xmin")), box.get("y1", box.get("ymin")),
                   box.get("x2", box.get("xmax")), box.get("y2", box.get("ymax"))]
        if cx is None and box and None not in box[:4]:
            cx = (float(box[0]) + float(box[2])) / 2.0          # assumes [x1, y1, x2, y2]
        if label is None or cx is None:
            continue
        cx = float(cx) / width if float(cx) > 1.5 else float(cx)  # pixels -> fraction of the image
        result.append({"label": str(label), "conf": None if conf is None else round(float(conf), 2),
                       "cx": round(cx, 3)})
    return result


class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_server")
        group = ReentrantCallbackGroup()
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.status_pub = self.create_publisher(String, "/llm/status", 10)
        self.state_pub = self.create_publisher(String, "/llm/state", 10)
        self.gripper = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd", callback_group=group)

        self.create_subscription(Odometry, "/odom", self.on_odom, 20, callback_group=group)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 20, callback_group=group)
        self.create_subscription(String, "/yolo/detections_json", self.on_detections, 10, callback_group=group)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(String, "/llm/cmd", self.on_cmd, 10, callback_group=group)
        self.create_subscription(String, "/llm/heartbeat", self.on_heartbeat, 10, callback_group=group)
        self.create_subscription(Bool, "/llm/estop", self.on_estop, 10, callback_group=group)
        self.create_timer(0.5, self.publish_state, callback_group=group)

        self.lock = threading.Lock()
        self.pose = None                  # {"x", "y", "yaw"}
        self.odom_time = 0.0
        self.joints = {}
        self.detections, self.detections_time = [], 0.0
        self.front_clearance, self.scan_time = float("inf"), 0.0
        self.heartbeat_time = 0.0
        self.estop = False
        self.busy = None                  # name of the running skill, or None
        self.cancel = threading.Event()
        self.last_result = None
        self.get_logger().info("skill_server ready. Waiting for commands on /llm/cmd")

    # ------------------------------------------------------------------ callbacks (sensor input)
    def on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.pose = {"x": p.position.x, "y": p.position.y, "yaw": yaw_from_quaternion(p.orientation)}
            self.odom_time = time.monotonic()

    def on_joints(self, msg):
        with self.lock:
            self.joints.update(zip(msg.name, msg.position))

    def on_detections(self, msg):
        try:
            dets = parse_detections(msg.data)
        except (ValueError, TypeError, AttributeError) as e:
            self.get_logger().warn("cannot read YOLO message: %r" % e, throttle_duration_sec=10.0)
            return
        with self.lock:
            self.detections, self.detections_time = dets, time.monotonic()

    def on_scan(self, msg):
        # closest LiDAR point within +-25 degrees of the front (angle 0 = forward on the TurtleBot3 LDS)
        closest, angle = float("inf"), msg.angle_min
        for r in msg.ranges:
            if abs(wrap_angle(angle)) < math.radians(25) and 0.12 <= r <= msg.range_max:
                closest = min(closest, r)
            angle += msg.angle_increment
        with self.lock:
            self.front_clearance, self.scan_time = closest, time.monotonic()

    def on_heartbeat(self, msg):
        self.heartbeat_time = time.monotonic()

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        if self.estop:
            self.cancel.set()
            self.send_velocity(0.0, 0.0)
        self.get_logger().warn("E-STOP %s" % ("LATCHED" if self.estop else "released"))

    # ------------------------------------------------------------------ commands
    def on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
            cid, name, args = str(cmd.get("id", "")), cmd["skill"], cmd.get("args") or {}
        except (ValueError, KeyError, TypeError):
            return self.reply("", "rejected", "bad command JSON")
        if name == "stop":
            self.cancel.set()
            self.send_velocity(0.0, 0.0)
            return self.reply(cid, "succeeded", "stopped")
        skill = getattr(self, "skill_" + name, None)
        if skill is None:
            return self.reply(cid, "rejected", "unknown skill " + name)
        if self.estop:
            return self.reply(cid, "rejected", "E-STOP is latched; a human must release it")
        if time.monotonic() - self.heartbeat_time > HEARTBEAT_TIMEOUT:
            return self.reply(cid, "rejected", "no heartbeat from the MCP server")
        with self.lock:
            if self.busy:
                return self.reply(cid, "rejected", "busy with %s; send stop first" % self.busy)
            self.busy = name
        self.cancel.clear()
        threading.Thread(target=self.run_skill, args=(cid, name, skill, args), daemon=True).start()

    def run_skill(self, cid, name, skill, args):
        try:
            state, message = "succeeded", skill(**args)
        except Aborted as e:
            state, message = "aborted", str(e)
        except TypeError as e:
            state, message = "rejected", "bad arguments: %s" % e
        except Exception as e:  # noqa: BLE001
            state, message = "failed", str(e)
        self.send_velocity(0.0, 0.0)
        with self.lock:
            self.busy = None
        self.last_result = "%s: %s %s" % (name, state, message)
        self.reply(cid, state, message)

    def reply(self, cid, state, message):
        self.status_pub.publish(String(data=json.dumps({"id": cid, "state": state, "message": message})))
        self.get_logger().info("[%s] %s: %s" % (cid, state, message))

    def check(self):
        """Called every control cycle. Raises Aborted if the skill must stop now."""
        if self.estop:
            raise Aborted("E-STOP")
        if self.cancel.is_set():
            raise Aborted("stopped by request")
        if time.monotonic() - self.heartbeat_time > HEARTBEAT_TIMEOUT:
            raise Aborted("lost heartbeat from the MCP server")
        if time.monotonic() - self.odom_time > 1.0:
            raise Exception("no odometry on /odom (is the bringup running?)")

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = max(-MAX_SPEED, min(MAX_SPEED, linear))
        msg.angular.z = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, angular))
        self.cmd_vel_pub.publish(msg)

    def get_pose(self):
        with self.lock:
            if self.pose is None:
                raise Exception("no odometry on /odom (is the bringup running?)")
            return dict(self.pose)

    # ------------------------------------------------------------------ skills
    def skill_drive(self, distance_m):
        target = max(-MAX_BACKWARD, min(MAX_FORWARD, float(distance_m)))   # safety clamp
        start, direction, t0 = self.get_pose(), (1.0 if target >= 0 else -1.0), time.monotonic()
        while True:
            self.check()
            p = self.get_pose()
            travelled = math.hypot(p["x"] - start["x"], p["y"] - start["y"])
            remaining = abs(target) - travelled
            if remaining <= 0.005:
                return "drove %.2f m" % (direction * travelled)
            if direction > 0 and time.monotonic() - self.scan_time < 0.6 and self.front_clearance < OBSTACLE_STOP:
                raise Exception("obstacle %.2f m ahead (LiDAR), stopped after %.2f m" % (self.front_clearance, travelled))
            if time.monotonic() - t0 > abs(target) / 0.03 + 3.0:
                raise Exception("timed out after %.2f m" % travelled)
            self.send_velocity(direction * max(0.03, min(MAX_SPEED, 1.5 * remaining)), 0.0)
            time.sleep(0.05)

    def skill_turn(self, angle_deg):
        target = math.radians(max(-360.0, min(360.0, float(angle_deg))))
        previous, turned, t0 = self.get_pose()["yaw"], 0.0, time.monotonic()
        while True:
            self.check()
            yaw = self.get_pose()["yaw"]
            turned += wrap_angle(yaw - previous)
            previous = yaw
            remaining = target - turned
            if abs(remaining) < math.radians(1.5):
                return "turned %.0f degrees" % math.degrees(turned)
            if time.monotonic() - t0 > abs(target) / 0.12 + 3.0:
                raise Exception("timed out after %.0f degrees" % math.degrees(turned))
            self.send_velocity(0.0, math.copysign(max(0.12, min(MAX_TURN_SPEED, 2.0 * abs(remaining))), remaining))
            time.sleep(0.05)

    def skill_arm_pose(self, name):
        if name not in ARM_POSES:
            raise Exception("unknown pose %r, known poses: %s" % (name, sorted(ARM_POSES)))
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = ARM_POSES[name]
        point.time_from_start = Duration(sec=2, nanosec=500000000)        # 2.5 s motion
        traj.points = [point]
        self.arm_pub.publish(traj)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            if self.cancel.is_set() or self.estop:
                raise Aborted("stopped by request")
            time.sleep(0.05)
        return "arm at pose " + name

    def skill_gripper(self, action):
        if action not in GRIPPER_POSITION:
            raise Exception("action must be 'open' or 'close'")
        if not self.gripper.wait_for_server(timeout_sec=2.0):
            raise Exception("gripper action server not available")
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = GRIPPER_POSITION[action], 1.0
        future = self.gripper.send_goal_async(goal)
        self.wait(future, 3.0)
        result_future = future.result().get_result_async()
        self.wait(result_future, 5.0)
        stalled = result_future.result().result.stalled
        return "gripper %s%s" % (action, " (holding something)" if stalled and action == "close" else "")

    def wait(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done():
            if self.cancel.is_set() or self.estop:
                raise Aborted("stopped by request")
            if time.monotonic() > end:
                raise Exception("gripper timed out")
            time.sleep(0.02)

    def skill_center_on(self, label):
        """Rotate until the object is in the middle of the camera image. Searches one full turn."""
        previous, searched, centered, t0 = self.get_pose()["yaw"], 0.0, 0, time.monotonic()
        while time.monotonic() - t0 < 40.0:
            self.check()
            yaw = self.get_pose()["yaw"]
            with self.lock:
                fresh = time.monotonic() - self.detections_time < 1.0
                matches = [d for d in self.detections if fresh and d["label"].lower() == label.lower()]
            if not matches:
                searched += abs(wrap_angle(yaw - previous))
                if searched > 2 * math.pi:
                    raise Exception("turned a full circle, no %s found" % label)
                self.send_velocity(0.0, 0.4)                               # search: turn left slowly
            else:
                error = max(matches, key=lambda d: d["conf"] or 0)["cx"] - 0.5   # > 0: object is right of center
                centered = centered + 1 if abs(error) < 0.06 else 0
                if centered >= 3:
                    return "%s is centered in front of the robot" % label
                turn = -1.2 * error * math.radians(CAMERA_HFOV_DEG)
                self.send_velocity(0.0, 0.0 if centered else math.copysign(max(0.12, min(abs(turn), 0.5)), turn))
            previous = yaw
            time.sleep(0.05)
        raise Exception("timed out")

    # ------------------------------------------------------------------ state for the LLM
    def publish_state(self):
        now = time.monotonic()
        with self.lock:
            pose = None if self.pose is None else {
                "x": round(self.pose["x"], 2), "y": round(self.pose["y"], 2),
                "heading_deg": round(math.degrees(self.pose["yaw"]))}
            state = {
                "pose": pose,
                "odometry_ok": now - self.odom_time < 1.0,
                "visible_objects": self.detections if now - self.detections_time < 1.0 else [],
                "arm_joints": [None if j not in self.joints else round(self.joints[j], 2) for j in ARM_JOINTS],
                "arm_poses": sorted(ARM_POSES),
                "busy_with": self.busy,
                "last_result": self.last_result,
                "estop": self.estop,
                "heartbeat_ok": now - self.heartbeat_time < HEARTBEAT_TIMEOUT,
                "lidar_front_clearance_m": (round(self.front_clearance, 2) if now - self.scan_time < 0.6
                                            and self.front_clearance != float("inf") else None),
            }
        self.state_pub.publish(String(data=json.dumps(state)))


def main():
    rclpy.init()
    node = SkillServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel.set()
        node.send_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
