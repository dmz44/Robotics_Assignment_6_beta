# Demo Code for Robotics_Assignment_6_Beta
# Copyright: 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

# Raw MCP Message Demo for Assignment 6_Beta (Part 2). Runs on the Remote-PC docker image.

# MCP messages are plain JSON-RPC 2.0 text. This program starts robot_mcp_server.py as a child process
# and talks to it through its stdin/stdout WITHOUT any MCP library, so you can see every message.
# The robot will turn 30 degrees left at the end. This code will run as-is.
#
# Messages (MCP version 2026-07-28, which is stateless: every request carries a "_meta" block with the
# protocol version and the client's name and capabilities):
#   server/discover  "Which versions and capabilities do you support?"  (the reply also has the LLM instructions)
#   tools/list       "Which tools do you have?"
#   tools/call       "Run tool T with arguments A."
# Older MCP versions start with an "initialize" handshake instead of server/discover.

# You need ROBOT_IP set in this shell, and the skill_server and rosbridge running on the Jetson (Part 1).
# You would need a basic understanding of Python Data Structure to understand this code.

import json
import os
import subprocess
import sys

SERVER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_mcp_server.py")
META = {
    "io.modelcontextprotocol/protocolVersion": "2026-07-28",
    "io.modelcontextprotocol/clientInfo": {"name": "raw-demo", "version": "1.0"},
    "io.modelcontextprotocol/clientCapabilities": {},
}

##############################################################################
# HINT: change or add calls, e.g. ("drive", {"distance_m": 3.0}) to see the safety limit.
CALLS = [
    ("get_robot_state", {}),
    ("turn", {"angle_deg": 30}),
]
##############################################################################


def request(server, request_id, method, params):
    message = {"jsonrpc": "2.0", "id": request_id, "method": method, "params": {**params, "_meta": META}}
    print(f"\n>>> CLIENT SENDS\n{json.dumps(message, indent=2)}")
    server.stdin.write(json.dumps(message) + "\n")      # one JSON message per line
    server.stdin.flush()
    reply = json.loads(server.stdout.readline())
    return reply


def main():
    if not os.environ.get("ROBOT_IP"):
        sys.exit("ROBOT_IP is not set. Run: source ~/my_code/gemini_key.sh")
    server = subprocess.Popen([sys.executable, SERVER_SCRIPT], stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, text=True)   # its log lines (stderr) still show

    reply = request(server, 1, "server/discover", {})
    print(f"<<< SERVER REPLIES\n{json.dumps(reply, indent=2)}")

    reply = request(server, 2, "tools/list", {})
    print("<<< SERVER REPLIES with these tools (name, description, input schema):")
    for tool in reply["result"]["tools"]:
        print(f"  {tool['name']}: {tool['description'].splitlines()[0]}")
        print(f"      schema: {json.dumps(tool['inputSchema'].get('properties', {}))}")

    for i, (name, args) in enumerate(CALLS, start=3):
        reply = request(server, i, "tools/call", {"name": name, "arguments": args})
        print(f"<<< SERVER REPLIES\n{json.dumps(reply, indent=2)[:1200]}")

    server.terminate()


if __name__ == "__main__":
    main()
