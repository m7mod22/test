# PROJECT CONTEXT: Dual Robotic Arm Control System

I'm building a control application for two robotic arms, and I want you to understand the full architecture before helping me with any part of it. Read this entire spec first. When I ask you to write code or make decisions later, follow the structure and rules below exactly — don't propose a different architecture unless I ask you to.

## 1. Goal

Two robotic arms, driven by stepper motors, controlled from a custom desktop app on Ubuntu. The priority is **repeatable precision**: the app should be able to teach points (by jogging or hand-guiding the arm), save them, and replay programs built from those points — the same way industrial arm controllers (ABB, Fanuc, UR) work. A secondary but critical requirement is that the two arms must never collide with each other, whether moving under a program or under manual/joystick control.

## 2. Hardware

- Two arms, each driven by stepper motors.
- Position feedback per joint via AS5600 magnetic encoders (or potentiometers as a fallback — AS5600 preferred for resolution and full rotation range).
- Either one Arduino Mega per arm, or one Mega driving both — firmware is per-arm either way.
- A Ubuntu laptop running the main application and ROS2, connected to the Mega(s) over USB serial.

## 3. Core design principles (don't deviate from these)

**Brain / muscle split.** The Arduino never makes decisions — it only executes parameterized commands sent from the laptop and reports sensor state back. All kinematics, trajectory planning, program logic, and collision checking live on the laptop. The Arduino's command set is small and generic (move-to-joint-angles, jog-at-speed, stop, home, gripper, ping), NOT one case per taught point — points and programs are data stored on the laptop, not code on the Arduino.

**Teach points as joint angles.** Like industrial controllers, saved points store the actual joint angles the arm reached (plus XYZ for display/reference). This makes replay accuracy depend only on sensor/motor repeatability, not on kinematic model accuracy.

**Position correction, not true PID.** Steppers are inherently open-loop position devices. The encoder's job is to catch lost steps and backlash, not to replace step counting. Use a simple proportional correction with a deadband, not a full PID loop, unless testing shows it's insufficient.

**Single command gate.** Every motion source — manual jog, sliders, saved-program playback, the collision monitor's stop signal — passes through one chokepoint (`gate.py`) before anything is sent to hardware. No UI code, no program runner, and no safety logic is allowed to talk to the serial port directly. This is the single most important architectural rule in the project.

**Dead-man heartbeat.** The laptop sends a PING to each Arduino continuously. If a Mega doesn't receive one within ~150-200ms (crashed app, unplugged cable, frozen UI thread), it decelerates its own motors to a stop independently. This must work even if the laptop app has completely frozen.

**Controlled stop, not "reset to home."** When danger is detected (collision risk, lost steps, fault), the system decelerates and holds position — it does NOT send the arm back to a home position, because the path home could itself cause a collision between the two arms. "Reset" is a separate, deliberate recovery action the operator chooses afterward (clear fault → optionally re-home → resume/jog/abort).

**MoveIt2 is used ONLY for collision checking — nothing else.** No MoveIt2 execution, no MoveIt2 IK, no controller manager, no trajectory execution through MoveIt2. It is configured once (URDF + SRDF via Setup Assistant) purely so a custom node can call its `/check_state_validity` service and tell the app whether the current or predicted joint state collides. All kinematics (FK/IK/Jacobian) for the app itself are implemented separately in plain Python.

**Kinematics runs on the laptop, not the Arduino.** The Arduino only understands joint angles. Any "move +10mm in X" command is converted to joint angles by the laptop's own FK/IK code before being sent over serial.

**Threading: nothing slow runs on the UI thread.** Serial I/O, the control loop, and ROS2 spinning each run on their own thread and communicate with the Qt UI only via signals. The heartbeat is sent from the control thread specifically so a frozen UI window can never stop the safety heartbeat.

**User data lives outside the project folder**, in standard Linux locations (`~/.config/robotarm/` for settings, `~/.local/share/robotarm/` for taught points, programs, and logs), managed only through one `storage.py` module — never hardcoded paths elsewhere.

## 4. Software is three separate things

1. **`arduino_firmware/`** — plain Arduino IDE sketches, not part of any app build. Pure command executor + sensor reporter.
2. **`ros2_ws/`** — a standard ROS2 workspace used ONLY for: (a) visualizing the live digital twin in RViz, and (b) one custom node that asks MoveIt2 "would this collide?" and publishes a simple safety status. No ROS2 node ever sends a trajectory to the real hardware.
3. **`app/`** — the PySide6 desktop application, split strictly into `core/` (backend logic, zero Qt imports, testable without a GUI) and `ui/` (display and user input only, zero business logic, zero direct hardware/serial/ROS access).

## 5. Data flow, end to end

```
Arduino A  <--serial-->  arm_link.py (A)  --\
Arduino B  <--serial-->  arm_link.py (B)  ---> protocol.py <-> gate.py <-> ui/modes/*
                                               |
                                               v
                                        ros_client.py --publishes--> /joint_states
                                               ^                          |
                                               |                          v
                                   /safety_state, /min_distance   robot_state_publisher -> /tf -> RViz
                                               ^                          ^
                                               |                          |
                                   collision_monitor_node <--calls-- move_group
                                     (the one custom ROS2 node)    (/check_state_validity)
                                               ^
                                   arm_description (URDF) + arm_moveit_config (SRDF)
```

`gate.py` reads the latest `/safety_state` (SAFE / SLOW / STOP) on every tick and uses it to approve, scale down, or block any outgoing motion command, regardless of whether that command came from a joystick, a slider, or a running program.

## 6. Application modes (all inside one shell with persistent E-stop, connection status, and a live 3D twin)

- **Setup** — serial ports, per-joint calibration/limits/gear ratios, Arm B's position relative to Arm A, safety margins. Done once, then locked.
- **Manual** — joint jog or Cartesian jog (sliders/gamepad), speed limit, live readout, "Save Point" button.
- **Teach** — table of saved points (name, joint angles, XYZ, arm), go-to-point, pairing handoff points between arms.
- **Program** — block-based editor (MoveJ/MoveL/Wait/Gripper/Loop), two lanes (one per arm) with sync/zone-lock blocks between them, validate (limits + reachability) and simulate (digital twin only) before running.
- **Run** — Start/Pause/Resume/Stop/Step, current-block highlight, fault handling with recovery dialog.
- **Monitor** — live target-vs-measured plots per joint, lost-step warnings, min inter-arm distance over time, event log.

## 7. Folder structure

```
robot_arm_project/
├── arduino_firmware/                    # separate from app, no build system
│   ├── arm_a/arm_a.ino                  # generic command set (MOVE_JOINTS, JOG, STOP, HOME, GRIPPER, PING...)
│   ├── arm_b/arm_b.ino                  # (or one merged .ino if one Mega drives both)
│   └── README.md                        # hand-kept doc of the command protocol
│
├── ros2_ws/
│   └── src/
│       ├── arm_description/
│       │   ├── urdf/robot.urdf          # single source of truth for geometry
│       │   └── meshes/                  # visual + collision shapes
│       │
│       ├── arm_moveit_config/           # generated via MoveIt Setup Assistant
│       │   ├── config/robot.srdf        # planning groups arm_a/arm_b, disabled self-collision pairs
│       │   ├── config/kinematics.yaml   # wizard defaults — not used for IK, just required boilerplate
│       │   ├── config/joint_limits.yaml
│       │   └── launch/move_group.launch.py   # move_group with NO controller manager — collision checking only
│       │
│       ├── arm_collision_monitor/       # the ONLY custom ROS2 node in the project
│       │   ├── src/collision_monitor_node.py   # subs /joint_states -> calls /check_state_validity -> pubs /safety_state, /min_distance
│       │   └── launch/collision_monitor.launch.py
│       │
│       └── arm_bringup/
│           └── launch/arm_system.launch.py     # starts robot_state_publisher + rviz2 + move_group + collision_monitor together
│
├── app/
│   ├── core/                            # BACKEND — no Qt imports
│   │   ├── protocol.py                  # dicts/dataclasses <-> serial command strings
│   │   ├── arm_link.py                  # SerialArmLink + FakeArmLink, one instance per Mega
│   │   ├── kinematics.py                # FK / IK / Jacobian, loaded from robot.urdf, pure math
│   │   ├── gate.py                      # single chokepoint for all motion; reads /safety_state, approves/blocks/scales every command
│   │   ├── program.py                   # blocks, limit/reachability validation, runner — steps flow through gate.py
│   │   ├── storage.py                   # ONLY file that knows file paths (via platformdirs); points/programs/settings in, objects out
│   │   └── ros_client.py                # only rclpy code in the app; pubs /joint_states, subs /safety_state + /min_distance
│   │
│   ├── ui/                              # FRONTEND — display + input only
│   │   ├── main_window.py               # shell: E-stop, arm selector, status bar, QStackedWidget of mode pages
│   │   ├── widgets/                     # dumb, reusable, display-only
│   │   │   ├── terminal_widget.py       # TX/RX serial log, capped buffer
│   │   │   ├── jog_panel.py
│   │   │   ├── point_table.py
│   │   │   └── twin_view.py             # RViz launcher, or embedded PyVistaQt later
│   │   ├── modes/                       # thin wiring: widget signal -> core call -> core signal -> widget update
│   │   │   ├── setup_mode.py
│   │   │   ├── manual_mode.py
│   │   │   ├── teach_mode.py
│   │   │   ├── program_mode.py
│   │   │   ├── run_mode.py
│   │   │   └── monitor_mode.py
│   │   ├── dialogs/                     # popups — collect input / show info, never call core directly
│   │   │   ├── confirm_dialog.py        # go-to-point, run program, overwrite, delete, edit-setup-while-connected
│   │   │   ├── fault_dialog.py          # shown on STOP/fault; Resume / Back off / Manual jog / Abort + its own E-stop button
│   │   │   ├── save_point_dialog.py     # name a new point/program
│   │   │   └── calibration_wizard.py    # Setup-mode sensor calibration, step-through
│   │   └── resources/
│   │       ├── icon.png
│   │       └── style.qss
│   │
│   ├── config/                          # ships WITH the app, same for everyone — not user data
│   │   ├── robot.urdf                   # symlink -> ros2_ws/src/arm_description/urdf/robot.urdf
│   │   └── arms.yaml.template           # copied to ~/.config/robotarm/arms.yaml on first run
│   │
│   ├── main.py                          # wires everything: loads config, starts threads, builds main_window, runs event loop; supports --fake
│   └── requirements.txt
│
├── packaging/
│   ├── robotarm.desktop                 # Exec= launch_app.sh, Icon= icon.png
│   ├── launch_app.sh                    # activates venv, sources ROS2 setup.bash, runs app/main.py
│   └── install.sh                       # installs .desktop + icon, runs update-desktop-database
│
└── README.md

User data (outside the project tree, managed only by storage.py):
~/.config/robotarm/arms.yaml
~/.local/share/robotarm/points/*.json
~/.local/share/robotarm/programs/*.json
~/.local/share/robotarm/logs/
```

## 8. How I want you to help

- Respect the layering strictly: `core/` never imports Qt; `ui/` never touches serial/ROS2/hardware directly; all motion goes through `gate.py`.
- If I ask for a feature that would break this structure (e.g., a widget calling the serial port directly), point that out before writing the code.
- If something about the hardware, wiring, or a specific joint count isn't specified above and matters for what I'm asking, ask me rather than assuming.
- Prefer small, testable modules over monoliths, matching the file breakdown above.
