# Sample Code for Robotics_Assignment_6_Beta
# Copyright: 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

# Keyboard Emergency Stop for Assignment 6_Beta. Runs on the Remote-PC docker image.
# Works without Gemini and without the MCP server: it publishes directly to the Jetson through rosbridge.
#   SPACE = emergency stop (the robot stops, every new command is rejected)
#   r     = release
#   q     = quit
# Keep this running in its own terminal whenever the robot is controlled by Gemini.

import json
import os
import sys
import termios
import tty

import roslibpy

ros = roslibpy.Ros(host=os.environ["ROBOT_IP"], port=int(os.environ.get("ROSBRIDGE_PORT", "9090")))
ros.run(timeout=10)
estop = roslibpy.Topic(ros, "/llm/estop", "std_msgs/msg/Bool")
cmd = roslibpy.Topic(ros, "/llm/cmd", "std_msgs/msg/String")
print("E-stop ready.  SPACE = stop   r = release   q = quit")

old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())          # read single key presses without Enter
try:
    while True:
        key = sys.stdin.read(1)
        if key == " ":
            estop.publish(roslibpy.Message({"data": True}))
            cmd.publish(roslibpy.Message({"data": json.dumps({"id": "estop", "skill": "stop", "args": {}})}))
            print("E-STOP LATCHED")
        elif key == "r":
            estop.publish(roslibpy.Message({"data": False}))
            print("e-stop released")
        elif key == "q":
            break
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    ros.terminate()
