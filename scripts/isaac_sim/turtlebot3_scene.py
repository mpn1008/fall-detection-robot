"""Run with Isaac Sim's python.bat, not the container/system Python.

Uses Isaac Sim's bundled rclpy for Twist, clock and ground-truth odometry,
and the classic World/WheeledRobot API for physics control.
"""

import argparse
import inspect
import math
import sys
import time
import traceback

from drive_support import (
    CommandWatchdog,
    configure_drive,
    read_robot_sample,
    robot_asset_candidates,
    step_robot,
)


def parse_args():
    """Parse scene options, leaving additional Kit arguments to SimulationApp."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--robot-usd", help="Override the bundled TurtleBot3 Burger asset")
    parser.add_argument("--cmd-vel-topic", default="/turtlebot3_test/cmd_vel")
    parser.add_argument("--odom-topic", default="/turtlebot3_test/odom")
    parser.add_argument("--command-timeout", type=float, default=0.5)
    # NVIDIA's TurtleBot driving tutorial uses 0.025 m; imported ROBOTIS URDFs
    # commonly use 0.033 m. Override to match your USD, not the physical robot.
    parser.add_argument("--wheel-radius", type=float, default=0.025)
    parser.add_argument("--wheel-base", type=float, default=0.16)
    args, _ = parser.parse_known_args()
    for name in ("command_timeout", "wheel_radius", "wheel_base"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    return args


def run(simulation_app, args):
    """Create the robot and service ROS messages while stepping its physics."""
    # Omniverse modules must be imported after constructing SimulationApp.
    from omni.kit.app import get_app

    print("[turtlebot3] Loading ROS and wheel-controller extensions", flush=True)
    extension_manager = get_app().get_extension_manager()
    for extension in ("isaacsim.ros2.bridge", "isaacsim.robot.wheeled_robots"):
        extension_manager.set_extension_enabled_immediate(extension, True)
        if not extension_manager.is_extension_enabled(extension):
            raise RuntimeError(f"Could not enable required extension: {extension}")
    simulation_app.update()

    import numpy as np
    import omni.client
    import omni.usd
    from pxr import UsdLux
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from isaacsim.core.api import World
    from isaacsim.core.utils.rotations import quat_to_rot_matrix
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.robot.wheeled_robots.controllers import DifferentialController
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot
    from isaacsim.storage.native import get_assets_root_path

    print("[turtlebot3] Scene imports ready; locating Burger asset", flush=True)
    candidates = robot_asset_candidates(
        None if args.robot_usd else get_assets_root_path(), args.robot_usd
    )
    asset_path = None
    for candidate in candidates:
        print(f"[turtlebot3] Checking asset: {candidate}", flush=True)
        result, _ = omni.client.stat(candidate)
        if result == omni.client.Result.OK:
            asset_path = candidate
            break
        print(f"[turtlebot3] Asset unavailable: {result}", flush=True)
    if asset_path is None:
        raise RuntimeError(
            "TurtleBot3 asset not found. Check asset-server access or pass --robot-usd. "
            f"Tried: {candidates}"
        )

    print(f"[turtlebot3] Building scene with {asset_path}", flush=True)
    dt = 1.0 / 60.0
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1000.0)
    robot = world.scene.add(
        WheeledRobot(
            prim_path="/World/TurtleBot3",
            name="turtlebot3",
            wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
            create_robot=True,
            usd_path=asset_path,
            position=np.array([0.0, 0.0, 0.05]),
        )
    )
    controller = DifferentialController(
        name="turtlebot3_drive",
        wheel_radius=args.wheel_radius,
        wheel_base=args.wheel_base,
        max_linear_speed=0.22,
        max_angular_speed=1.0,
    )
    if not args.headless:
        set_camera_view(eye=[1.2, 1.2, 0.9], target=[0.15, 0.0, 0.1])
    print("[turtlebot3] Initializing robot physics", flush=True)
    world.reset()
    wheel_indices = configure_drive(robot, controller)
    print(
        f"[turtlebot3] Articulation ready: root={robot.prim_path}, "
        f"wheel_indices={wheel_indices}, velocity_control=True",
        flush=True,
    )
    watchdog = CommandWatchdog(args.command_timeout)

    rclpy.init(args=[])
    node = Node("isaac_turtlebot3_test")
    clock_pub = node.create_publisher(
        Clock, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    )
    odom_pub = node.create_publisher(Odometry, args.odom_topic, QoSProfile(depth=1))
    commands_received = 0

    def receive_command(message):
        nonlocal commands_received
        commands_received += 1
        if not world.is_playing():
            watchdog.clear()
            return
        if not watchdog.receive(
            message.linear.x, message.angular.z, time.monotonic(), world.current_time
        ):
            node.get_logger().warning("Rejected non-finite or out-of-range velocity")

    node.create_subscription(Twist, args.cmd_vel_topic, receive_command, QoSProfile(depth=1))
    node.get_logger().info(f"Ready: {args.cmd_vel_topic}; asset: {asset_path}")
    node.get_logger().info("ROS endpoints ready; /clock and odometry publish only while playing")
    clocks_published = 0
    odometry_published = 0
    odometry_skipped = 0
    odometry_unavailable = False
    last_status_time = time.monotonic() - 5.0
    last_playing = None
    try:
        while simulation_app.is_running() and rclpy.ok():
            frame_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            if step_robot(
                world, robot, controller, watchdog, time.monotonic(), render=not args.headless
            ):
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(
                    round(world.current_time * 1e9), 1_000_000_000
                )
                clock_pub.publish(clock)
                clocks_published += 1
                if world.current_time_step_index % 6 == 0:
                    sample = read_robot_sample(robot)
                    if sample is None:
                        odometry_skipped += 1
                        if not odometry_unavailable:
                            node.get_logger().warning(
                                "Skipping odometry: pose/velocity physics state is unavailable; "
                                "not publishing fabricated zero velocity"
                            )
                        odometry_unavailable = True
                    else:
                        if odometry_unavailable:
                            node.get_logger().info("Odometry physics measurements available again")
                        odometry_unavailable = False
                        odom = Odometry()
                        odom.header.stamp = clock.clock
                        odom.header.frame_id = "world"
                        odom.child_frame_id = "turtlebot3_test/base_link"
                        for index, axis in enumerate(("x", "y", "z")):
                            setattr(odom.pose.pose.position, axis, sample.position[index])
                        # Isaac quaternions are w,x,y,z; ROS stores x,y,z,w.
                        for index, axis in enumerate(("w", "x", "y", "z")):
                            setattr(odom.pose.pose.orientation, axis, sample.orientation[index])
                        rotation = quat_to_rot_matrix(np.asarray(sample.orientation))
                        linear = rotation.T @ np.asarray(sample.linear_velocity)
                        angular = rotation.T @ np.asarray(sample.angular_velocity)
                        for index, axis in enumerate(("x", "y", "z")):
                            setattr(odom.twist.twist.linear, axis, float(linear[index]))
                            setattr(odom.twist.twist.angular, axis, float(angular[index]))
                        odom_pub.publish(odom)
                        odometry_published += 1
            playing = world.is_playing()
            status_time = time.monotonic()
            if playing != last_playing or status_time - last_status_time >= 5.0:
                print(
                    f"[turtlebot3] Status: playing={playing}, "
                    f"sim_time={world.current_time:.3f}s, "
                    f"clock_tx={clocks_published}, odom_tx={odometry_published}, "
                    f"physics_ready={robot.handles_initialized}, odom_skipped={odometry_skipped}, "
                    f"clock_subscribers={clock_pub.get_subscription_count()}, "
                    f"commands_rx={commands_received}",
                    flush=True,
                )
                if not playing and last_playing is not False:
                    print("[turtlebot3] Simulation paused/stopped: press Play", flush=True)
                last_playing = playing
                last_status_time = status_time
            # Don't let an empty scene run many times faster than real time.
            time.sleep(max(0.0, dt - (time.monotonic() - frame_start)))
    finally:
        watchdog.clear()
        world.stop()
        node.destroy_node()
        rclpy.try_shutdown()


def run_with_diagnostics(simulation_app, args):
    """Report exceptions before Kit's fast shutdown can terminate the process."""
    exit_code = 0
    try:
        run(simulation_app, args)
    except KeyboardInterrupt:
        exit_code = 130
    except BaseException:
        # In 6.2 close() can exit the process immediately. A bare finally/close
        # therefore loses the pending exception and incorrectly reports status 0.
        exit_code = 1
        print("[turtlebot3] Scene failed. Python traceback:", file=sys.stderr, flush=True)
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()
    finally:
        # The exit_code argument was added in 6.2; keep older launchers usable.
        if "exit_code" in inspect.signature(simulation_app.close).parameters:
            simulation_app.close(exit_code=exit_code)
        else:
            simulation_app.close()
    return exit_code


def main():
    """Start and close Isaac Sim, including when scene construction fails."""
    args = parse_args()
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    return run_with_diagnostics(simulation_app, args)


if __name__ == "__main__":
    sys.exit(main())
