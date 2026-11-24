# Sample Code for Robotics_Assignment_6_Beta
# Copyright: 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

# MCP Server for the TurtleBot3 for Assignment 6_Beta. Runs on the Remote-PC docker image.

# This program is an MCP SERVER. It offers robot TOOLS (drive, turn, move_arm, ...) to any MCP client.
# It talks MCP (JSON-RPC 2.0) over its stdin/stdout, and it talks to the robot's skill_server through
# rosbridge (a WebSocket on port 9090 of the Jetson) using the roslibpy library.
# You normally do not start it yourself: gemini_agent.py and mcp_raw_demo.py start it as a child process.
#
# IMPORTANT: stdout carries the MCP messages. Never print() to stdout in this file. Use log() (stderr).
#
# How a Python function becomes an MCP tool (the @mcp.tool() decorator does this):
#   function name   -> tool name
#   docstring       -> tool description   (THIS IS A PROMPT: the LLM reads it to decide when to use the tool)
#   typed arguments -> input JSON Schema  (angle_deg: float -> {"angle_deg": {"type": "number"}})
#   returned string -> tool result         (the LLM reads it as the observation)

# You would need a basic understanding of Python Data Structure, Object Oriented Programming and
# asyncio (async/await) to understand this code.

import asyncio
import json
import os
import subprocess  # noqa: F401  (Part 3, TODO 1)
import sys
import threading
import time
import uuid
from typing import Literal

import roslibpy
from mcp.server.mcpserver import MCPServer

ROBOT_IP = os.environ.get("ROBOT_IP", "")
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", "9090"))


def log(*args):
    print("[robot_mcp_server]", *args, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------------------------
# Connection to the robot through rosbridge
# ---------------------------------------------------------------------------------------------
class Robot:
    def __init__(self, host, port):
        self.ros = roslibpy.Ros(host=host, port=port)
        log(f"connecting to rosbridge ws://{host}:{port} ...")
        self.ros.run(timeout=10)
        log("connected to rosbridge")
        self.cmd_topic = roslibpy.Topic(self.ros, "/llm/cmd", "std_msgs/msg/String")
        self.heartbeat_topic = roslibpy.Topic(self.ros, "/llm/heartbeat", "std_msgs/msg/String")
        roslibpy.Topic(self.ros, "/llm/status", "std_msgs/msg/String").subscribe(self.on_status)
        roslibpy.Topic(self.ros, "/llm/state", "std_msgs/msg/String").subscribe(self.on_state)
        self.waiting = {}             # command id -> (threading.Event, result dict)
        self.latest_state = None
        threading.Thread(target=self.heartbeat_loop, daemon=True).start()
        for _ in range(30):                      # wait up to 3 s for the first /llm/state message
            if self.latest_state:
                break
            time.sleep(0.1)
        else:
            log("WARNING: no /llm/state from the robot. Is skill_server.py running on the Jetson?")

    def heartbeat_loop(self):
        # Tells the skill_server "the Remote-PC is still in control". If this stops, the robot stops.
        while True:
            self.heartbeat_topic.publish(roslibpy.Message({"data": str(time.time())}))
            time.sleep(0.5)

    def on_status(self, msg):
        status = json.loads(msg["data"])
        waiter = self.waiting.get(status.get("id"))
        if waiter and status["state"] in ("succeeded", "failed", "aborted", "rejected"):
            waiter[1].update(status)
            waiter[0].set()

    def on_state(self, msg):
        self.latest_state = json.loads(msg["data"])

    def run_skill(self, skill, args, timeout=60.0):
        """Send one skill to the skill_server and wait for its final status (blocking)."""
        cid = uuid.uuid4().hex[:6]
        done, result = threading.Event(), {}
        self.waiting[cid] = (done, result)
        self.cmd_topic.publish(roslibpy.Message({"data": json.dumps({"id": cid, "skill": skill, "args": args})}))
        finished = done.wait(timeout)
        del self.waiting[cid]
        if not finished:
            self.stop()
            return f"FAILED: no answer from the robot within {timeout:.0f} s, sent stop"
        prefix = "OK" if result["state"] == "succeeded" else result["state"].upper()
        return f"{prefix}: {result['message']}"

    def stop(self):
        self.cmd_topic.publish(roslibpy.Message({"data": json.dumps({"id": "stop", "skill": "stop", "args": {}})}))


robot = None   # created in main()


async def skill(name, args, timeout=60.0):
    """Run a robot skill without blocking the MCP server. Returns 'OK: ...' on success."""
    return await asyncio.to_thread(robot.run_skill, name, args, timeout)


# ---------------------------------------------------------------------------------------------
# The MCP server and its tools
# ---------------------------------------------------------------------------------------------
INSTRUCTIONS = """You control a real TurtleBot3 mobile robot with a 4-joint arm and a gripper.
It has a forward camera with YOLO object detection and wheel odometry. It may not see obstacles,
so prefer short moves. Positive turn angles turn left, negative turn right. Fold the arm to 'home'
before driving. If a tool result is not OK, tell the user why instead of retrying."""

mcp = MCPServer("turtlebot3", instructions=INSTRUCTIONS)


@mcp.tool()
async def get_robot_state() -> str:
    """Robot position (x, y in meters, heading in degrees, from where it started), visible objects,
    arm joints, the named arm poses, and safety status."""
    return json.dumps(robot.latest_state) if robot.latest_state else "No state received from the robot yet."


@mcp.tool()
async def list_visible_objects() -> str:
    """Objects YOLO sees in the camera image right now. cx is the horizontal position in the image:
    0 = left edge, 0.5 = center, 1 = right edge."""
    objects = (robot.latest_state or {}).get("visible_objects") or []
    return json.dumps(objects) if objects else "Nothing detected right now."


@mcp.tool()
async def drive(distance_m: float) -> str:
    """Drive straight. Positive = forward, negative = backward. Meters. Limits: 0.5 m forward and
    0.2 m backward per call, 0.1 m/s."""
    return await skill("drive", {"distance_m": distance_m})


@mcp.tool()
async def turn(angle_deg: float) -> str:
    """Rotate in place. Positive = left (counter-clockwise), negative = right. Degrees."""
    return await skill("turn", {"angle_deg": angle_deg})


@mcp.tool()
async def stop() -> str:
    """Stop all motion immediately."""
    robot.stop()
    return "OK: stop sent"


@mcp.tool()
async def move_arm(pose: str) -> str:
    """Move the arm to a named pose: home (folded, for driving), init (straight up), ready (forward),
    look_left, look_right."""
    return await skill("arm_pose", {"name": pose}, 15.0)


@mcp.tool()
async def gripper(action: Literal["open", "close"]) -> str:
    """Open or close the gripper."""
    return await skill("gripper", {"action": action}, 15.0)


@mcp.tool()
async def center_on_object(label: str) -> str:
    """Rotate until an object with this YOLO label (e.g. 'bottle', 'cup', 'person') is straight ahead.
    Turns up to one full circle to search for it."""
    return await skill("center_on", {"label": label}, 50.0)


# =============================================================================================
# PART 3: YOUR TOOLS
# Rules for good tools:
#   1. Write the docstring for the LLM: what the tool does, units, limits, when to use it.
#   2. Use simple argument types (str, float, int, bool).
#   3. For an expected failure, return a short explanation instead of raising an exception.
#   4. Never block: run slow work with `await asyncio.to_thread(...)` or `await skill(...)`.
# =============================================================================================

# EXAMPLE (complete): a read-only tool built from the robot state.
@mcp.tool()
async def count_objects(label: str) -> str:
    """Count how many objects with this YOLO label the camera sees right now. Does not move the robot."""
    objects = (robot.latest_state or {}).get("visible_objects") or []
    n = sum(1 for o in objects if o["label"].lower() == label.strip().lower())
    return f"I see {n} {label}(s) right now."


# TODO 1: speak(text)
# Make the robot talk on the Remote-PC speaker with espeak-ng (Milestone 5).
#   a) Uncomment the block below and write a docstring that tells the LLM when to use the tool.
#   b) Run espeak-ng without blocking:  await asyncio.to_thread(subprocess.run, ["espeak-ng", text])
#   c) Return a short result, e.g. f"Said: {text}"
#
# @mcp.tool()
# async def speak(text: str) -> str:
#     """TODO: describe the tool for the LLM."""
#     ...


# TODO 2: wave_hello()
# One tool that COMBINES several skills: wave hello with the arm.
#   a) Uncomment the block below and write the docstring.
#   b) Call skills one after another, and stop if one fails:
#          result = await skill("arm_pose", {"name": "look_left"}, 15.0)
#          if not result.startswith("OK"):
#              return "Wave failed: " + result
#      Suggested sequence: look_left -> look_right -> look_left -> home
#   c) Return a summary, e.g. "Waved hello, arm back at home."
#
# @mcp.tool()
# async def wave_hello() -> str:
#     """TODO: describe the tool for the LLM."""
#     ...


def main():
    global robot
    if not ROBOT_IP:
        sys.exit("[robot_mcp_server] ROBOT_IP is not set. Run: export ROBOT_IP=<Jetson IP address>")
    robot = Robot(ROBOT_IP, ROSBRIDGE_PORT)
    mcp.run("stdio")      # serve MCP over stdin/stdout until the client disconnects


if __name__ == "__main__":
    main()
