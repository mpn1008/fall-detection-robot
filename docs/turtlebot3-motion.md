# First TurtleBot3 motion test in Isaac Sim

The `turtlebot3_motion` ROS 2 package drives forward at **0.1 m/s for 3 simulation
seconds**, then sends zero velocity repeatedly and exits. Expected travel is
roughly **30 cm**, not a guaranteed distance: this is timed, open-loop control.
No Nav2, Gazebo, teleoperation, or physical robot drivers are needed.

```text
Windows: C:\isaac-sim\python.bat + turtlebot3_scene.py
   TurtleBot3 Burger + flat ground + differential wheel controller
         ↑ /turtlebot3_test/cmd_vel  (geometry_msgs/msg/Twist)
         ↓ /clock                  (rosgraph_msgs/msg/Clock)
         ↓ /turtlebot3_test/odom    (nav_msgs/msg/Odometry, ground truth)
Linux container: ros2 run turtlebot3_motion move_a_bit
```

The separate `/turtlebot3_test` command topic avoids the usual physical robot
`/cmd_vel`. **Keep physical robots disconnected from this test domain.** The node
cannot distinguish a simulator subscriber from an arbitrary ROS subscriber.
Do not run another command publisher or another `/clock` source during the test.

## 1. Build inside the dev container

```bash
cd /workspaces/fall-detection-robot/ros_ws
python /usr/bin/colcon build --symlink-install --packages-select turtlebot3_motion
source install/setup.bash
```

The package only needs `rclpy` and standard ROS messages already supplied by the
container. The Windows script runs in Isaac Sim's bundled Python, **not** this
workspace's Python environment; do not copy the Linux `install` directory to Windows.

## 2. Check the Windows installation and DDS profile

The supplied launcher defaults to **`C:\isaac-sim`**, and requires its
`python.bat`, bundled **Jazzy** ROS libraries, and access to Isaac's TurtleBot3
Burger assets. It checks both the newer `isaacsim.ros2.core\jazzy\lib` layout
and the older `isaacsim.ros2.bridge\jazzy\lib` layout.

**Version:** the Windows startup log identifies Isaac Sim **6.2.0** and confirms
that bundled Jazzy `rclpy` loads. The scene uses the classic
`isaacsim.core.api.World` and `isaacsim.robot.wheeled_robots` APIs retained as
deprecated extensions in that release. Its asset search includes the renamed
6.2 tutorial asset `Turtlebot/Turtlebot3/tb3_burger_processed.usda` and the nested
multiphysics Burger asset, followed by older `.usd` paths. The current 6.2 asset
URLs have been checked, but Windows scene creation and physics still need live
validation. Do not silently load Humble libraries into a Jazzy process.

Follow [Windows ↔ container communication](../README.md#windows--container-communication)
to generate `.devcontainer/generated/windows-fastdds.xml`. Reuse your existing
working profile; do not replace routed WSL IP addresses with localhost. For a
routed WSL setup, recheck the Windows/WSL addresses after a restart. The profile
and container must use the same domain (default `0`).

The launcher runs from a **Windows-accessible checkout of this repository**.
If the checkout is in WSL, open its folder through
`\\wsl.localhost\<your-distribution>\...`, or copy `scripts/isaac_sim` and the
generated DDS profile to Windows and pass `-DdsProfile` explicitly. The container
path `/workspaces/fall-detection-robot` is not a Windows path.

## 3. Start the scene in Windows PowerShell

Close other Isaac Sim instances first to avoid extra memory/GPU load and duplicate
clocks. Open a **fresh PowerShell without an external ROS environment sourced**,
change to the repository root, and run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\isaac_sim\start-turtlebot3.ps1 -IsaacPath 'C:\isaac-sim'
```

The execution policy override applies only to this process; it does not change
the machine-wide policy. Review the script before running it.

The script creates a new scene, loads TurtleBot3 Burger, starts physics, and
publishes clock and ground-truth odometry. **It does not command movement until
a ROS velocity command arrives.** The first launch may take time downloading
assets and compiling shaders. The robot should settle on the ground.

An alternate local/imported Burger USD can be specified with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\isaac_sim\start-turtlebot3.ps1 -IsaacPath 'C:\isaac-sim' -RobotUsd 'C:\robot-assets\turtlebot3_burger.usd' -DdsProfile 'C:\robot-config\windows-fastdds.xml'
```

That asset must have a free-moving articulation, wheel collision geometry,
velocity drives, and joints named `wheel_left_joint` and `wheel_right_joint`.
The defaults follow NVIDIA's TurtleBot drive tutorial: wheel radius `0.025 m`,
wheel separation `0.16 m`. For a different USD, adjust `--wheel-radius` and
`--wheel-base` when running `turtlebot3_scene.py` directly with Isaac's
`python.bat` in the configured PowerShell environment. ROBOTIS Burger URDFs
commonly use a `0.033 m` wheel radius; mismatched geometry changes travel distance.

## 4. Verify communication before movement

Inside the container, with the workspace sourced:

```bash
ros2 daemon stop
ros2 topic echo /clock --once
ros2 topic info /turtlebot3_test/cmd_vel --verbose
ros2 topic echo /turtlebot3_test/odom --once
```

Expect a nonzero simulation timestamp, a `geometry_msgs/msg/Twist` subscriber,
and a robot pose. Check advancing clock messages with `ros2 topic echo /clock`
if necessary. Topic discovery alone is not proof that data crosses the network.
Do not start the motion test until the robot is visible and on the ground.

## 5. Run the node inside the container

```bash
ros2 run turtlebot3_motion move_a_bit --ros-args -p use_sim_time:=true
```

Expected logs: `Waiting ...`, `Moving ...`, then `Motion complete; sending zero
velocity`. Confirm **in the simulator** that the robot moves forward a short
distance and stops. Rerun the command for another short move; it never repeats
automatically.

For an even smaller first move:

```bash
ros2 run turtlebot3_motion move_a_bit --ros-args \
  -p use_sim_time:=true -p linear_speed:=0.05 -p duration:=2.0
```

In another terminal, observe the actual pose before/after and near-zero velocity
after stopping:

```bash
ros2 topic echo /turtlebot3_test/odom
```

This is simulator ground truth, not wheel odometry or localization; no TF tree is
provided. Visual displacement and changing pose verify movement, not just command
publication. Wheel calibration, physics settling and friction affect distance.

### Stopping and failure behavior

- **Ctrl+C** or **SIGTERM** requests a burst of stop commands before exit.
- Missing subscriber or simulation clock: no movement; startup fails after 30 wall seconds.
- Paused/missing clock for 2 wall seconds, a backwards clock reset, or a lost
  subscriber: abort and send stops. Resume/reset the scene and rerun manually.
- The simulator watchdog zeros wheel targets after **0.5 wall or simulation
  seconds** without a fresh command. Pausing/stopping the scene clears its command.
- Stop/Play invalidates Isaac's articulation handles. The script resets/reinitializes
  the robot, verifies both wheel indices, and restores velocity control before
  commanding it again. It never replays a movement command from before a reset.
- Missing, scalar or non-finite physics velocities are not valid odometry. The
  script skips those samples and warns rather than reporting a fabricated stop or
  crashing clock/control publication. Status lines include `physics_ready` and
  `odom_skipped` so persistent physics-state failures remain visible.
- Stop messages are repeated for **0.5 wall seconds**. They are best effort at the
  application level, not a safety-rated emergency stop. `kill -9`, process crashes,
  network loss, and other publishers require the simulator watchdog. That watchdog
  only works while the simulator process is able to execute.
- Exit status is `0` for completion, `1` for a runtime/startup abort, `2` for invalid
  parameters, and `130` for interruption.

The normal ROS default for `use_sim_time` is `false`. **Always pass
`-p use_sim_time:=true` for Isaac Sim** so a slow simulator does not shorten the
move. With `false`, the test deliberately uses wall time and does not wait for
`/clock`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `cmd_vel_topic` | `/turtlebot3_test/cmd_vel` | Twist destination |
| `linear_speed` | `0.1` | Forward m/s; bounded to ±0.22 |
| `angular_speed` | `0.0` | Yaw rad/s; bounded to ±1.0 |
| `duration` | `3.0` | Motion seconds; 0.01–30 |
| `publish_rate` | `10.0` | Command/monitor frequency; 5–50 Hz |
| `startup_timeout` | `30.0` | Wall seconds to wait for readiness |
| `clock_timeout` | `2.0` | Wall seconds without clock progress before abort |
| `stop_duration` | `0.5` | Wall seconds of repeated zeros before exit |

For an existing scene, wire a Twist subscriber on the same command topic to its
differential controller and publish `/clock`. Remapping the node to `/cmd_vel`
does not automatically configure the scene or provide its command watchdog.

## Troubleshooting

- **App starts then immediately shuts down:** use the latest scene script. It
  prints `[turtlebot3]` progress messages and any Python traceback **before**
  closing Kit. Isaac Sim 6.2 fast shutdown can otherwise hide a pending exception
  and report exit status 0. The launcher now preserves the scene's failure status.
  To save both stdout and stderr, rerun your working launch command with
  `2>&1 | Tee-Object -FilePath .\data\isaac-turtlebot3.log` appended, from the
  repository root. Send the traceback and the last `[turtlebot3]` progress line.
- **`Could not import system rclpy` followed by `rclpy loaded`:** normal when
  using Isaac Sim's bundled ROS Python libraries; not itself a bridge failure.
- **`XMLPARSER Error` / empty `metatrafficMulticastLocatorList`:** an older DDS
  profile was rejected. Regenerate both profiles with the corrected generator;
  keep the existing Windows/WSL addresses. Restart ROS commands/the ROS daemon and
  fully relaunch Isaac Sim so they read the fixed XML. Rebuild the container image
  once to replace its older baked-in generator. See the main README's profile
  repair instructions; do not bypass the profile by unsetting its environment variables.
- **No `/clock` or subscriber:** check the simulator console for ROS-library or
  asset-loading errors; verify matching domain, current DDS IPs, UDP firewall
  rules and Play state. See the networking section in the main README.
- **Topics are discoverable but `/clock` has no samples:** the scene logs a
  `[turtlebot3] Status` heartbeat every five seconds and when Play state changes.
  Keep `ros2 topic echo /clock` running while reading two successive heartbeats:
  - `playing=False`: press Play; clock/odometry are deliberately not published
    while the scene is paused or stopped.
  - `playing=True` but `sim_time` or `clock_tx` does not increase: inspect the
    Windows simulator/physics logs before changing networking.
  - `clock_tx` increases but `clock_subscribers=0`: check endpoint matching, ROS
    domain, profiles and QoS. The echo subscriber should be counted after discovery.
  - `clock_tx` increases and `clock_subscribers` is nonzero, but Linux receives
    nothing: investigate data delivery/firewall/advertised locators. Discovery
    alone is not proof of delivery; a successful parameter-service call confirms
    some bidirectional DDS data traffic, not necessarily every topic's delivery.
  Paste two heartbeat lines and the container command output for diagnosis.
- **Commands visible, wheels do not move:** inspect joint names, articulation
  root, drive damping, a fixed base, wheel collisions, and ground contact. A Twist
  echo alone is not an end-to-end test.
- **`ValueError: matmul ... operand ... has 0` from odometry:** the old script
  tried to rotate an unavailable/scalar physics velocity (Isaac may return `None`
  for invalid articulation handles). Use the updated script, fully exit Isaac Sim
  and wait for the launcher to return, then relaunch. Editing the Python file does
  not update a running process; traceback line numbers can even refer to different
  on-disk lines after an edit. Look for `Articulation ready` with two valid wheel
  indices, and check `physics_ready=True` in the status heartbeat.
- **Missing assets:** configure Isaac's asset server/local asset pack or supply
  `-RobotUsd`. Asset paths differ between releases; errors list the attempted paths.
- **Startup extremely slow/out of memory:** start only this simple scene and close
  other applications. The main README's Isaac Sim RAM warning still applies.

## Automated tests (container only)

```bash
cd /workspaces/fall-detection-robot/ros_ws
python /usr/bin/colcon test --packages-select turtlebot3_motion --event-handlers console_direct+
python /usr/bin/colcon test-result --verbose
cd /workspaces/fall-detection-robot
python -B scripts/isaac_sim/test_drive_support.py
python -B scripts/isaac_sim/test_scene_diagnostics.py
python -B scripts/isaac_sim/test_robot_state.py
```

The ROS tests use actual Fast DDS **UDP loopback**, an isolated domain `42`,
unique command topics, and a fake `/clock`. They cover timing, readiness, stop
bursts, clock stalls/resets, subscriber loss and invalid parameters. The
standard-library tests cover the simulator watchdog, asset candidate selection,
error reporting before fast shutdown, invalid/missing odometry measurements,
and Stop/Pause/Play handle recovery with cleared commands. These tests do not exercise
Windows, GPU rendering, Isaac's Python APIs, asset loading, or physics: the live
Windows steps above are still required.

References:
- [NVIDIA: Driving TurtleBot using ROS 2 Messages](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html)
- [NVIDIA: ROS 2 standalone workflow](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_python.html)
