# 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

## Programming Assignment: Milestone 6_Beta (V0.9 Beta)

**Minhyuk Park and Tsz-Chiu Au**

### Introduction

In this sixth milestone, you will control the TurtleBot3 with the OpenMANIPULATOR-X arm through a conversation with **Google Gemini**, using a **free Google AI Studio account** and the **Model Context Protocol (MCP)**. You will give the robot commands by typing or by voice, and you will write your own robot tools for the LLM.

In Milestone 5, you built a natural language pipeline by hand: Whisper turned your voice into text, LLaMa produced a JSON array of actions, and you had to parse that text yourself. In this milestone, you will use the two ideas that modern LLM-controlled robots are built on:

* **Function calling:** the LLM does not write free text for you to parse. It returns a structured request such as "call `turn` with `angle_deg = 90`". Your program runs the function and sends the result back, and the LLM decides the next step.
* **Model Context Protocol (MCP):** an open standard for how an AI application finds and calls tools that live in another program. We wrap the robot in an **MCP server**, so any MCP-capable AI application can use it.

Gemini can also listen to your voice directly: the recording itself goes into the request, so no Whisper step is needed.

We are using **ROS 2 Humble in a Docker environment** for Remote-PC and **ROS 2 Foxy natively** on the TurtleBot3's Nvidia Jetson, as in the previous milestones.

For all questions regarding milestone assignments and the robot, **you should contact the Doctoral Instructor Assistant via direct message on Slack**. Please do not contact the Instructor with questions regarding the milestone assignments. This is the URL for Slack for this course.

<https://spring2026txstrobot.slack.com/>

Here is the official introduction to the Model Context Protocol.
[https://modelcontextprotocol.io/docs/getting-started/intro](https://modelcontextprotocol.io/docs/getting-started/intro)

Here is Google's guide to function calling with the Gemini API.
[https://ai.google.dev/gemini-api/docs/function-calling](https://ai.google.dev/gemini-api/docs/function-calling)

**This is a Beta assignment.** If a step does not work exactly as written, copy the terminal output and message the Doctoral Instructor Assistant on Slack.

### Assignment requirement

A hardware video demonstration submission is required for Milestone Assignment 6_Beta.

You need to demonstrate that you understand how MCP and function calling work, that you can control the robot through Gemini by text and by voice, and that you can add your own MCP tools. Refer to the demo requirement section at the end of the milestone assignment on what to include in the video.

**[SUBMISSION RULES]**

* **Individual Submission:** **Every team member must submit the video link(s) separately to Canvas.** If the video is duplicated within a team, that is acceptable; however, this ensures that only active participants who have access to the team's recordings can receive credit.

* **Standardized Hosting:** **To manage file sizes, do not upload raw video files (e.g., MP4) directly to Canvas.** Instead, **upload your videos to YouTube (set as "Unlisted")** and submit the links via a document.

* **Keep your API key secret:** **Never show your Gemini API key in the video**, in a screenshot, in Slack, or in a Git commit.

### Video Demo Requirements

Your group will **record** one or more video clips. The estimated total length of the video clips is approximately five minutes. **While you do not need to perform complex editing, please keep the total duration to a few minutes to ensure it remains concise.** One group member should narrate the video, explaining each step as it's performed. At the beginning of the first video clip, please show every group member's face and state the names of all group members.

Your recording setup should be organized to show all relevant windows at once: the Remote-PC terminals (agent and e-stop), the Jetson terminals, and the physical robot.

You do not need to edit the videos, and uploading raw **footage** will suffice. You can split the demonstration into multiple videos **if necessary to show different parts of the requirement.**

Rules for robot usage will apply for working with the physical Turtlebot3. Please refer to the inventory list given to you separately and the rules for the Robot room usage.

### Major Changes

* **v 0.9 Beta:** Initial Beta Release

---

### Overview

The assignment consists of five single-file Python programs. There is nothing to build: you copy the files and run them.

| File | Runs on | Role |
| :--- | :--- | :--- |
| `jetson/skill_server.py` | Jetson (ROS 2 Foxy) | Runs the robot **skills** (drive, turn, arm pose, gripper, center on object) with hard safety limits |
| `remote_pc/robot_mcp_server.py` | Remote-PC Docker | **MCP server**: offers the skills as MCP tools, and talks to the Jetson through rosbridge |
| `remote_pc/gemini_agent.py` | Remote-PC Docker | **MCP host and client**: talks to Gemini and calls the MCP tools |
| `remote_pc/estop.py` | Remote-PC Docker | Keyboard emergency stop, independent of Gemini |
| `remote_pc/mcp_raw_demo.py` | Remote-PC Docker | Shows the raw MCP messages (Part 2) |

```
 Remote-PC (Docker)                                                Turtlebot Nvidia Jetson
 ┌───────────────────────────────────────────────┐                ┌───────────────────────────────────────┐
 │ you ──► gemini_agent.py ◄────► Gemini (cloud)  │                │ bringup (wheels, arm, gripper)        │
 │              │  MCP messages (stdin/stdout)    │                │ YOLO publisher ─► /yolo/detections_json│
 │              ▼                                 │     Wi-Fi      │ skill_server.py ◄─► /cmd_vel, arm, ... │
 │        robot_mcp_server.py ────────────────────┼────────────────┼─► rosbridge (port 9090) ◄─► /llm/...   │
 │        estop.py ───────────────────────────────┼────────────────┼─►                                     │
 └───────────────────────────────────────────────┘                └───────────────────────────────────────┘
```

**Why rosbridge?** The MCP Python library needs Python 3.10 or newer, but ROS 2 Foxy on the Jetson uses Python 3.8. Also, ROS 2 Humble (Remote-PC) and ROS 2 Foxy (Jetson) are not guaranteed to talk to each other directly. **rosbridge** is a ROS 2 node that turns topics into JSON messages over a WebSocket, so the Remote-PC can publish and subscribe without ROS 2 Foxy.

**Why skills instead of `/cmd_vel`?** The LLM never publishes velocities itself. It asks for a skill such as "turn 90 degrees", and `skill_server.py` runs it closed-loop on odometry, with limits the LLM cannot change: at most 0.1 m/s, 0.5 m forward and 0.2 m backward per command. An LLM is too slow to run a control loop over the internet, but it is good at choosing the next skill.

---

### Part 1: Setting Up the Robot, the Remote-PC and Gemini

This assumes that you have a working setup from Milestone Assignment 1 Part 1. Please execute all instructions with **[Remote-PC]** on the Docker shell. Note that you have to enable the GUI and start the Docker container by following instruction from Milestone Assignment 1. Please execute all instructions with **[Native Bash Shell Remote-PC]** on the Remote-PC's own terminal outside Docker, and all instructions with **[Turtlebot Nvidia Jetson]** on Turtlebot Jetson's native bash shell without Docker.

The following instructions are from our own and not from online sources. Please follow carefully and ask the TA for assistance should you face any problems.

#### Getting a free Gemini API key

An **API key** is a secret string that identifies you to Google. Every request to Gemini includes it, and anyone who has it can use your quota, so treat it like a password.

**[Any web browser]** Create the key. If a button has a slightly different name, look for the closest match.

1. Open [https://aistudio.google.com](https://aistudio.google.com) and sign in with a personal Google account. Each student uses their own account.
2. Click **Get API key**, then **Create API key**. If you are asked for a Google Cloud project, let AI Studio create one.
3. Copy the key.

**Free tier rules:**

* **Rate limits:** there is a limit on requests per minute and per day. Your limits are shown in AI Studio. **One robot command can use several requests**, because every tool call is one round trip to Gemini. If you see **HTTP 429**, wait a minute.
* **Data use:** on the free tier, Google may use your text, voice recordings and images to improve its products, and human reviewers may read them. Do not say personal information to the robot. Details: [https://ai.google.dev/gemini-api/terms](https://ai.google.dev/gemini-api/terms)

#### Downloading the files and installing on the Remote-PC

**[Native Bash Shell Remote-PC]** While connected to the internet, git clone the assignment into the my_code folder of the Docker's shared folder.

```bash
cd ~/turtlebot_docker/my_code
git clone https://github.com/dmz44/Robotics_Assignment_6_Beta.git
```

**[Native Bash Shell Remote-PC]** Save your settings in a small file in the shared folder, so you do not have to type them in every terminal. This file is **outside** the assignment folder, so it can never be committed to Git by accident.

```bash
nano ~/turtlebot_docker/my_code/gemini_key.sh
```

Type the following three lines. Replace `PASTE_YOUR_KEY_HERE` with your key. Leave the robot address as it is for now: you will fill it in at the end of this part. Save with `Ctrl+O`, `Enter`, and exit with `Ctrl+X`.

```bash
export GEMINI_API_KEY="PASTE_YOUR_KEY_HERE"
export GEMINI_MODEL="gemini-3.5-flash-lite"
export ROBOT_IP="0.0.0.0"
```

`GEMINI_MODEL` selects the model. Model names change over time: if you get an error that says the model is **not found**, open AI Studio, copy the name of a current "Flash-Lite" or "Flash" model, and put it here.

**[Remote-PC]** In a Docker shell, install the Python libraries for this assignment: `mcp` (the MCP SDK), `google-genai` (the Gemini SDK) and `roslibpy` (the rosbridge client). Also install `espeak-ng`, the text-to-speech program from Milestone 5, which reads Gemini's answers aloud. As in Milestone 5, we install into the container's Python, not into a virtual environment. You are root inside Docker, so `sudo` is not needed.

```bash
cd ~/my_code/Robotics_Assignment_6_Beta/remote_pc
pip install -r requirements.txt
apt update && apt install -y espeak-ng
```

**[Remote-PC]** Check the installation. The line must print without an error.

```bash
python3 -c "import mcp, google.genai, roslibpy; print('All imports OK')"
```

Note: always type `python3`, not `python`. If you delete and re-create your Docker container, repeat this step.

#### Starting the robot side on the Jetson

**[Turtlebot Nvidia Jetson]** While connected to the internet, git clone the assignment into the home folder of the Jetson.

```bash
cd ~
git clone https://github.com/dmz44/Robotics_Assignment_6_Beta.git
```

**[Turtlebot Nvidia Jetson]** Check that rosbridge is installed. Both names must be printed. If not, run `sudo apt install -y ros-foxy-rosbridge-suite`, or contact the Doctoral Instructor Assistant.

```bash
ros2 pkg list | grep -E "^rosbridge_server$|^rosapi$"
```

**[Turtlebot Nvidia Jetson]** Start four programs, **each in its own terminal**, and in each terminal source your ROS 2 environment as in the previous milestones. Use the same `ROS_DOMAIN_ID` as in the previous milestones.

1. **Terminal 1:** bring up the robot hardware exactly as in the previous milestones.
2. **Terminal 2:** start the YOLO JSON publisher exactly as in the previous milestones.
3. **Terminal 3:** start rosbridge. It listens for WebSocket connections on port 9090.

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

   **Example output (look for this line):**
   ```
   [rosbridge_websocket-1] [INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
   ```

4. **Terminal 4:** start the skill server.

   ```bash
   python3 ~/Robotics_Assignment_6_Beta/jetson/skill_server.py
   ```

   **Example output:**
   ```
   [INFO] [skill_server]: skill_server ready. Waiting for commands on /llm/cmd
   ```

Do **not** run keyboard teleoperation or MoveIt Servo at the same time. They send commands to the same topics as the skill server, and the robot would receive conflicting commands.

**[Turtlebot Nvidia Jetson]** In a fifth terminal, look at the robot state that the skill server publishes twice per second. Press `Ctrl+C` after one message.

```bash
ros2 topic echo /llm/state
```

**Example output:**
```
data: '{"pose": {"x": 0.0, "y": 0.0, "heading_deg": 0}, "odometry_ok": true, "visible_objects": [{"label": "bottle", "conf": 0.86, "cx": 0.31}], ... "heartbeat_ok": false, ...}'
```

| Field | Expected | If not |
| :--- | :--- | :--- |
| `odometry_ok` | `true` | The bringup (Terminal 1) is not running |
| `visible_objects` | The objects YOLO sees. `cx` is the position in the image: 0 = left edge, 0.5 = center, 1 = right edge | Check Terminal 2. If YOLO sees objects but the list stays empty, look for a `cannot read YOLO message` warning in Terminal 4 |
| `heartbeat_ok` | `false` for now | Correct: it becomes `true` when the Remote-PC connects in Part 2 |

**What is the heartbeat?** While the MCP server on the Remote-PC is connected, it sends a small message to `/llm/heartbeat` twice per second. If the skill server receives no heartbeat for 1.5 seconds (Wi-Fi dropped, a program crashed), it stops the robot and rejects new commands. The robot never keeps driving after the Remote-PC has lost control. This is called a **watchdog**.

**[Turtlebot Nvidia Jetson]** Find the Jetson's IP address. The first address is usually the Wi-Fi address.

```bash
hostname -I
```

**[Native Bash Shell Remote-PC]** Open `~/turtlebot_docker/my_code/gemini_key.sh` again with `nano`, and replace `0.0.0.0` in the `ROBOT_IP` line with the Jetson's address.

**[Remote-PC]** In a Docker shell, load your settings and check the connection to rosbridge. Run `source` in **every new Docker shell** you use for this assignment.

```bash
source ~/my_code/gemini_key.sh
echo "key starts with ${GEMINI_API_KEY:0:4}..., robot at $ROBOT_IP"
python3 -c "import socket, os; socket.create_connection((os.environ['ROBOT_IP'], 9090), 3); print('rosbridge is reachable')"
```

The `echo` line prints only the first 4 characters of your key, so it is safe to show in your video. If the last line prints an error, check that rosbridge (Terminal 3) is running, that both machines are on the same Wi-Fi network, and that `ROBOT_IP` is correct.

---

### Part 2: Controlling the TurtleBot3 through MCP and Gemini

Part 2 will show you what MCP messages look like, and then let Gemini control the robot.

#### Function calling in one paragraph

With **function calling**, the program sends Gemini a list of **function declarations** together with your command. Each declaration has a **name** (`turn`), a **description** ("Rotate in place. Positive = left...") and a **JSON Schema** for the arguments (`angle_deg` is a number). The description is a prompt: everything you learned about prompt engineering in Milestone 5 applies to it. Gemini then answers either with text, or with a **function call** such as `{"name": "turn", "args": {"angle_deg": 90}}`. **Gemini never runs anything itself.** The program runs the function and sends the result back, and Gemini reads it and decides the next step. This is the **Thought → Action → Observation** loop (ReAct) from the Milestone 5 appendix, in a structured form: the plan is not fixed in advance, and Gemini sees the result of every step.

#### MCP in one table

MCP moves the tools out of the AI program into a separate program, the **MCP server**, so that the tools are written once and any MCP application can use them.

| MCP role | Meaning | In this assignment |
| :--- | :--- | :--- |
| **MCP Host** | The AI application the user talks to | `gemini_agent.py` |
| **MCP Client** | The part of the host that keeps the connection to one MCP server | the `Client` object in `gemini_agent.py` |
| **MCP Server** | The program that offers the tools | `robot_mcp_server.py` |

* **Messages:** MCP messages are **JSON-RPC 2.0**. A **request** has a `method`, `params` and an `id`, and the **response** carries the same `id`, so the client knows which request it answers.
* **Transport:** our host starts the server as a child process and writes one JSON message per line to the server's **stdin**, then reads the answers from its **stdout**. That is why the MCP server must never `print()` normal text to stdout. It writes its log lines to stderr instead.
* **Translation:** Gemini does not speak MCP. `gemini_agent.py` converts each MCP tool into a Gemini function declaration (`to_gemini_declaration()`), and converts each Gemini function call into an MCP `tools/call` request.

#### Looking at the raw MCP messages

**[Remote-PC]** In a Docker shell, run the raw demo. It starts `robot_mcp_server.py` and talks to it **without any MCP library**, so you see every message. At the end, **the robot turns 30 degrees to the left**, so make sure it has space.

```bash
source ~/my_code/gemini_key.sh
cd ~/my_code/Robotics_Assignment_6_Beta/remote_pc
python3 mcp_raw_demo.py
```

**Example output (shortened):**
```
>>> CLIENT SENDS
{ "jsonrpc": "2.0", "id": 1, "method": "server/discover",
  "params": { "_meta": { "io.modelcontextprotocol/protocolVersion": "2026-07-28", ... } } }
[robot_mcp_server] connecting to rosbridge ws://192.168.50.132:9090 ...
[robot_mcp_server] connected to rosbridge
<<< SERVER REPLIES
{ "jsonrpc": "2.0", "id": 1, "result": { "capabilities": { "tools": ... },
  "instructions": "You control a real TurtleBot3 mobile robot ...", "supportedVersions": [ "2026-07-28" ] } }

>>> CLIENT SENDS
{ "jsonrpc": "2.0", "id": 2, "method": "tools/list", ... }
<<< SERVER REPLIES with these tools (name, description, input schema):
  drive: Drive straight. Positive = forward, negative = backward. Meters. Limits: 0.5 m forward and
      schema: {"distance_m": {"title": "Distance M", "type": "number"}}
  ...
>>> CLIENT SENDS
{ "jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": { "name": "turn", "arguments": { "angle_deg": 30 }, ... } }
<<< SERVER REPLIES
{ "jsonrpc": "2.0", "id": 4, "result": { "content": [ { "type": "text", "text": "OK: turned 30 degrees" } ], "isError": false } }
```

In the output, find:

* **`server/discover`:** the client asks which protocol versions and capabilities the server supports. The reply contains the **instructions**: the robot's rules, which the agent gives to Gemini.
* **`tools/list`:** the server lists its tools, each with a name, a description, and an input schema. Each one becomes a Gemini function declaration.
* **`tools/call`:** the client asks the server to run one tool. The server sends the skill to the Jetson and replies with the result text.
* **`_meta`:** in the current MCP version (2026-07-28), every request carries the protocol version and the client's name and capabilities, so each request stands on its own. Older MCP versions instead start with an `initialize` handshake, and you will still see it in many applications.

Look at Terminal 4 on the Jetson: the skill server logs the `turn` command it received from rosbridge.

**[Remote-PC]** Open `mcp_raw_demo.py`, find the `HINT` comment, and add `("drive", {"distance_m": 3.0})` to the `CALLS` list. Run the demo again. How far does the robot drive, and which program decided that?

#### Opening the e-stop terminal

**[Remote-PC]** Before Gemini controls the robot, open the e-stop in a **new Docker shell** and keep it visible. One team member is responsible for the e-stop and does nothing else.

```bash
source ~/my_code/gemini_key.sh
cd ~/my_code/Robotics_Assignment_6_Beta/remote_pc
python3 estop.py
```

* **Space:** emergency stop. The robot stops at once, and every new command is rejected.
* **r:** release.
* **q:** quit.

The e-stop talks to the Jetson directly through rosbridge. It does not depend on Gemini or on the MCP server, and Gemini has no tool that can release it.

**[SAFETY RULES]** Run the robot on the floor in an open area, at least 2 meters away from stairs, desks and people's feet. The robot only "sees" objects YOLO knows. If the LiDAR is running, forward drives stop at 0.25 m from an obstacle, but nothing protects backward motion.

#### Controlling the robot with Gemini

**[Remote-PC]** In a Docker shell, start the agent.

```bash
source ~/my_code/gemini_key.sh
cd ~/my_code/Robotics_Assignment_6_Beta/remote_pc
python3 gemini_agent.py
```

**Example output:**
```
[robot_mcp_server] connecting to rosbridge ws://192.168.50.132:9090 ...
[robot_mcp_server] connected to rosbridge
MCP tools: get_robot_state, list_visible_objects, drive, turn, stop, move_arm, gripper, center_on_object, count_objects
Gemini model: gemini-3.5-flash-lite. Type a command, 'v' for voice, 'stop', or 'quit'.

you>
```

When the agent started, it launched `robot_mcp_server.py`, connected to it as an MCP client, and called `tools/list`: these are the same steps you saw in the raw demo. On the Jetson, `/llm/state` now shows `heartbeat_ok: true`.

Every final answer from Gemini (the `robot>` line) is also **read aloud with espeak-ng** on the Remote-PC speaker. Speech runs in the background, so you can type the next command while the robot is talking. To turn it off, or to change the speed or the voice, edit `SPEAK_REPLIES` and `ESPEAK_ARGS` at the top of `gemini_agent.py`.

**[Remote-PC]** Type the following commands in this order. Start with commands that do not move the robot, then move it a little, then more. Read the `[step]` lines: each one is a function call from Gemini and the MCP result that goes back.

1. `what do you see?`
2. `turn left 45 degrees`
3. `turn to face the bottle` (use an object that YOLO detects in your room)
4. `drive forward 0.3 meters`
5. `open the gripper, move the arm to the ready pose, then close the gripper and fold the arm`

**Example output:**
```
you> turn to face the bottle and drive 0.3 m toward it
  [step 1] Gemini asks to call center_on_object({"label": "bottle"})
  [step 1] MCP server returns: OK: bottle is centered in front of the robot
  [step 2] Gemini asks to call drive({"distance_m": 0.3})
  allow? [y/N] y
  [step 2] MCP server returns: OK: drove 0.30 m
robot> I turned to face the bottle and drove 0.3 m toward it.
```

Before every `drive`, the agent asks `allow? [y/N]`. Type `y` and Enter to allow it, or anything else to refuse it (Gemini is told the operator refused). You can change this with `CONFIRM_TOOLS` at the top of `gemini_agent.py`.

#### Voice commands (no Whisper)

In Milestone 5 the voice pipeline was **microphone → Whisper → text → LLaMa → espeak**. Here it is **microphone → audio file → Gemini (with tool calls) → espeak**: Gemini is a **multimodal** model that understands the audio itself and chooses the tools in the same step, so the Whisper step is gone.

**[Remote-PC]** At the `you>` prompt, type `v` and Enter. The agent records 5 seconds with `arecord`, as in Milestone 5. Say a command clearly, for example "turn right ninety degrees and tell me what you see". If the robot is still speaking its previous answer, the recording starts when it finishes, so that the microphone does not record the robot's own voice.

```
you> v
  Recording for 5 seconds. Speak now!
  [step 1] Gemini asks to call turn({"angle_deg": -90})
  [step 1] MCP server returns: OK: turned -90 degrees
  ...
```

#### Testing the safety features

**[Remote-PC]** Give a command that keeps the robot moving for a few seconds, for example `turn left 270 degrees`, and while it moves, run each test:

* **E-stop test:** press **Space** in the e-stop terminal. The robot must stop at once, and the result must say `ABORTED: E-STOP`. Then try another command: it must be rejected. Press **r** to release.
* **Watchdog test:** while the robot turns, press **Ctrl+C** in the agent terminal. The agent and its MCP server exit, the heartbeat stops, and the robot must stop within about 1.5 seconds without any stop command. Terminal 4 on the Jetson shows `aborted: lost heartbeat from the MCP server`.

Type `quit` to exit the agent.

---

### Part 3: Writing Your Own MCP Tools

For Part 3, you will add two tools to the MCP server. Open `remote_pc/robot_mcp_server.py` and scroll to the section **PART 3: YOUR TOOLS**. It contains one complete example tool and two TODOs.

The `@mcp.tool()` decorator turns a Python function into an MCP tool:

| Python | Becomes | Example |
| :--- | :--- | :--- |
| Function name | Tool name | `count_objects` |
| Docstring | Tool description (**a prompt for the LLM**) | `"Count how many objects with this YOLO label ..."` |
| Typed arguments | Input JSON Schema | `label: str` → `{"label": {"type": "string"}}` |
| Returned string | Tool result (the LLM's observation) | `"I see 2 bottle(s) right now."` |

Inside a tool, `await skill(name, args, timeout)` runs a skill on the robot and returns a string that starts with `OK:` on success. The available skills are `"drive"`, `"turn"`, `"arm_pose"`, `"gripper"` and `"center_on"`.

**[Remote-PC]** Test the example tool first. Start the agent and ask:

```
you> how many bottles can you see?
  [step 1] Gemini asks to call count_objects({"label": "bottle"})
  [step 1] MCP server returns: I see 2 bottle(s) right now.
robot> I can see two bottles.
```

**[Remote-PC]** **TODO 1: `speak(text)`.** Complete the tool so that the robot talks through the Remote-PC speaker with `espeak-ng`. The agent already reads Gemini's **final answer** aloud. With a `speak` tool, Gemini can also **decide by itself** to say something in the middle of a task, for example announcing "I am turning to the bottle now" before it moves. Follow steps a) to c) in the file, then test with:

```
you> say hello to everyone in the room
```

**[Remote-PC]** **TODO 2: `wave_hello()`.** Complete the tool, which combines several arm skills into one behaviour and stops if one of them fails. Follow steps a) to c) in the file, then test with:

```
you> wave hello and then say good morning
```

After every change, quit the agent (`quit`) and start it again, so that the MCP server restarts and lists the new tools. If the agent stops at start-up with an error, there is a Python mistake in `robot_mcp_server.py`: the error message shows the line.

---

### Video Demo Requirements (5 Minute Demonstration)

Please refer to the video submission requirements in the introduction.

Your submission must include links to the video files.

#### Part A: MCP Messages

The goal of this part is to demonstrate that you understand the MCP messages.

* **Raw Demo:** Run `mcp_raw_demo.py` on the real robot. Point to `server/discover`, `tools/list` and `tools/call` in the output, and explain what each message does and what the `id` field is for.
* **MCP Roles:** In your narration, name the MCP host, client and server in this assignment, and explain why `gemini_agent.py` has to translate between MCP tools and Gemini function declarations.
* **Safety Limit:** Show the `drive 3.0` call from Part 2, and explain where the limit is enforced and why the LLM cannot change it.

#### Part B: Controlling the Robot with Gemini

In this part, you will control the physical TurtleBot3 through Gemini.

* **Typed Commands:** Show a multi-step command (for example, "turn to face the bottle and drive 0.3 m toward it"). Using the `[step]` lines, explain who chooses each tool, who runs it, and how the result returns to Gemini. In your narration, compare this with the JSON action array of Milestone 5.
* **Arm and Gripper:** Show a command that uses the gripper and an arm pose.
* **Voice Command:** Show one voice command that moves the robot, with the spoken answer. In your narration, explain how this pipeline differs from the Whisper pipeline of Milestone 5.
* **Safety:** Show the e-stop test and the watchdog test from Part 2. The robot must stop in both cases.

#### Part C: Your Tools and Code Walkthrough

The goal of this part is to explain the code you wrote. With your source code visible:

* Show Gemini calling `count_objects`, `speak` and `wave_hello` on the real robot.
* Explain your `speak` and `wave_hello` code, and why you wrote the docstrings the way you did.
* Answer this question: why is it safer to give the LLM one `wave_hello` tool than to let it call `move_arm` four times by itself?

---

### Troubleshooting

| Problem | What to do |
| :--- | :--- |
| `GEMINI_API_KEY is not set` or `ROBOT_IP is not set` | Run `source ~/my_code/gemini_key.sh` in this shell |
| Error with `HTTP 400` or `HTTP 403` | The key is wrong or not active: copy it again from AI Studio |
| Error with `HTTP 404` or "model not found" | Put a current model name from AI Studio in `GEMINI_MODEL` |
| Error with `HTTP 429` | Free-tier limit: wait one minute. If it says `limit: 0`, choose another model |
| The agent hangs at `connecting to rosbridge` | Check Terminal 3 on the Jetson, the Wi-Fi, and `ROBOT_IP` |
| `WARNING: no /llm/state from the robot` | `skill_server.py` (Terminal 4) is not running |
| Every motion: `REJECTED: no heartbeat from the MCP server` | Wait 2 seconds after the agent starts, then try again |
| Every motion: `REJECTED: E-STOP is latched` | Press `r` in the e-stop terminal |
| `FAILED: no odometry on /odom` | The bringup (Terminal 1) is not running |
| `visible_objects` is always empty | Terminal 4 shows `cannot read YOLO message`: send that line to the Doctoral Instructor Assistant |
| No spoken answers | Test the speaker with `espeak-ng "hello"`. Check that `SPEAK_REPLIES = True` in `gemini_agent.py` |
| The voice recording is silent | Test with `arecord -d 3 -f cd test.wav && aplay test.wav`, as in Milestone 5 |
| The robot jerks or moves strangely | Stop keyboard teleop and MoveIt Servo |

### Recommended Reading

**[Recommended Reading] Model Context Protocol: Architecture overview**
[https://modelcontextprotocol.io/docs/learn/architecture](https://modelcontextprotocol.io/docs/learn/architecture)

**[Recommended Reading] Gemini API: Function calling**
[https://ai.google.dev/gemini-api/docs/function-calling](https://ai.google.dev/gemini-api/docs/function-calling)

**[Recommended Reading] ReAct: Synergizing Reasoning and Acting in Language Models**
[https://arxiv.org/abs/2210.03629](https://arxiv.org/abs/2210.03629)
