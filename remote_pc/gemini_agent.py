# Sample Code for Robotics_Assignment_6_Beta
# Copyright: 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

# Gemini Agent for Assignment 6_Beta. Runs on the Remote-PC docker image.

# This program is the MCP HOST: the application you talk to. It contains one MCP CLIENT, which starts
# robot_mcp_server.py and talks MCP to it. For every command:
#   1. Your text (or a voice recording) goes to Gemini together with the robot tools as function declarations.
#   2. Gemini answers with a function call, e.g. turn(angle_deg=90).
#   3. The MCP client sends tools/call to the MCP server, which runs the skill on the robot.
#   4. The result goes back to Gemini, which calls the next tool or gives the final answer.
#   5. The final answer is printed and read aloud with espeak-ng (as in Milestone 5).
#
# Usage:   python3 gemini_agent.py
#   type a command + Enter     text command
#   v + Enter                  voice command (records 5 seconds with arecord, as in Milestone 5)
#   stop + Enter               stop the robot directly (does not go through Gemini)
#   quit + Enter               exit (the robot also stops: the heartbeat ends)

# You need GEMINI_API_KEY, GEMINI_MODEL and ROBOT_IP set in this shell (Part 1 of the assignment).
# You would need a basic understanding of Python Data Structure, Object Oriented Programming and
# asyncio (async/await) to understand this code.

import asyncio
import json
import os
import re
import shutil
import subprocess
import sys

from google import genai
from google.genai import errors, types
from mcp.client.client import Client
from mcp.client.stdio import StdioServerParameters

##############################################################################
# HINT: change these to your needs.
MODEL = os.environ.get("GEMINI_MODEL", "gemini-3.5-flash-lite")
RECORD_SECONDS = 5
CONFIRM_TOOLS = {"drive"}       # ask the operator (y/N) before these tools run
SPEAK_REPLIES = True            # read Gemini's answers aloud with espeak-ng
ESPEAK_ARGS = ["-s", "160"]     # espeak-ng options, e.g. "-s" speed in words per minute, "-v" voice
MAX_STEPS = 8                   # maximum tool calls for one command
SYSTEM_INSTRUCTION = """You are the assistant of a TurtleBot3 robot and you act only through the tools.
The user may speak to you with an audio recording: treat the spoken words as the request. If the audio is
unclear, ask the user to repeat instead of guessing a motion. Do only what the user asked. Answer in one
or two short sentences, in the language the user used."""
##############################################################################

SERVER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_mcp_server.py")


def to_gemini_declaration(tool):
    """Convert one MCP tool (name, description, JSON Schema) into a Gemini function declaration."""
    schema = json.loads(json.dumps(tool.input_schema))
    for prop in schema.get("properties", {}).values():
        prop.pop("title", None)                     # MCP's auto-generated titles are not needed
    schema.pop("title", None)
    return types.FunctionDeclaration(name=tool.name, description=tool.description,
                                     parameters_json_schema=schema)


speech = None   # the espeak-ng process that is currently talking


def speak_reply(text):
    """Read the answer aloud in the background, so you can type the next command right away."""
    global speech
    if not SPEAK_REPLIES or not text or shutil.which("espeak-ng") is None:
        return
    text = re.sub(r"[*_#`]", "", text)          # drop markdown symbols, or espeak reads "asterisk"
    speech = subprocess.Popen(["espeak-ng", *ESPEAK_ARGS, text],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def record_voice(filename="command.wav"):
    if speech is not None and speech.poll() is None:
        speech.wait()                              # do not record the robot's own voice
    print(f"  Recording for {RECORD_SECONDS} seconds. Speak now!")
    subprocess.run(["arecord", "-q", "-d", str(RECORD_SECONDS), "-f", "cd", filename])
    with open(filename, "rb") as f:
        return f.read()


async def call_gemini(gemini, history, config):
    for attempt in range(3):
        try:
            return await gemini.aio.models.generate_content(model=MODEL, contents=history, config=config)
        except errors.APIError as e:
            if e.code == 429 and attempt < 2:      # free-tier rate limit: wait and retry
                print("  (rate limit reached, waiting 15 s)")
                await asyncio.sleep(15)
            else:
                raise


async def handle_command(user_parts, mcp_client, gemini, history, config):
    history.append(types.Content(role="user", parts=user_parts))
    for step in range(1, MAX_STEPS + 1):
        response = await call_gemini(gemini, history, config)
        model_turn = response.candidates[0].content
        history.append(model_turn)                 # keep Gemini's turn (it may contain hidden "thought" data)
        calls = [p.function_call for p in (model_turn.parts or []) if p.function_call]
        if not calls:
            answer = response.text or ""
            print(f"robot> {answer}")
            speak_reply(answer)
            return

        results = []
        for call in calls:
            args = dict(call.args or {})
            print(f"  [step {step}] Gemini asks to call {call.name}({json.dumps(args)})")
            if call.name in CONFIRM_TOOLS and (await asyncio.to_thread(input, "  allow? [y/N] ")).strip() != "y":
                text = "DENIED by the operator."
            else:
                result = await mcp_client.call_tool(call.name, args)          # MCP tools/call
                text = " ".join(c.text for c in result.content if hasattr(c, "text"))
            print(f"  [step {step}] MCP server returns: {text[:200]}")
            results.append(types.Part(function_response=types.FunctionResponse(
                id=call.id, name=call.name, response={"result": text})))
        history.append(types.Content(role="user", parts=results))
    print("robot> (stopped after too many steps)")


async def main():
    for var in ("GEMINI_API_KEY", "ROBOT_IP"):
        if not os.environ.get(var):
            sys.exit(f"{var} is not set in this shell. Run: source ~/my_code/gemini_key.sh")

    gemini = genai.Client(api_key=os.environ["GEMINI_API_KEY"], http_options=types.HttpOptions(timeout=60000))
    server = StdioServerParameters(command=sys.executable, args=[SERVER_SCRIPT], env=dict(os.environ))

    async with Client(server) as mcp_client:                     # start the MCP server and connect
        tools = (await mcp_client.list_tools()).tools             # MCP tools/list
        print("MCP tools:", ", ".join(t.name for t in tools))
        config = types.GenerateContentConfig(
            system_instruction=SYSTEM_INSTRUCTION + "\n" + (mcp_client.instructions or ""),
            tools=[types.Tool(function_declarations=[to_gemini_declaration(t) for t in tools])],
            temperature=0.2,
            automatic_function_calling=types.AutomaticFunctionCallingConfig(disable=True),
        )
        history = []
        print(f"Gemini model: {MODEL}. Type a command, 'v' for voice, 'stop', or 'quit'.")
        while True:
            line = (await asyncio.to_thread(input, "\nyou> ")).strip()
            if line in ("quit", "exit"):
                break
            if line == "stop":
                result = await mcp_client.call_tool("stop", {})
                print(" ", result.content[0].text)
                continue
            if line == "v":
                # The audio itself goes to Gemini. There is no Whisper step.
                parts = [types.Part.from_bytes(data=record_voice(), mime_type="audio/wav")]
            elif line:
                parts = [types.Part.from_text(text=line)]
            else:
                continue
            try:
                await handle_command(parts, mcp_client, gemini, history, config)
            except Exception as e:  # noqa: BLE001
                print(f"  error: {e}  (conversation history cleared)")
                history.clear()
            for content in history:   # replace old recordings with a short note to keep requests small
                content.parts = [types.Part.from_text(text="[voice command]") if p.inline_data else p
                                 for p in content.parts or []]


if __name__ == "__main__":
    asyncio.run(main())
