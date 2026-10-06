# Robot Arm Project

## Project Structure

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
│       │   └── launch/move_group.launch.py   # move_group with NO controller manager — collision checking only, never drives hardware
│       │
│       ├── arm_collision_monitor/       # YOUR ONLY custom ROS2 node
│       │   ├── src/collision_monitor_node.py   # subs /joint_states -> calls move_group's /check_state_validity -> pubs /safety_state, /min_distance
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
```
