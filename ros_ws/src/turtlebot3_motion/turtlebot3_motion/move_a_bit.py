"""Publish a bounded velocity command, then a burst of stop commands."""

from dataclasses import dataclass, fields
import math
import signal
import sys
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions


@dataclass(frozen=True)
class MotionConfig:
    """Conservative bounds for a small, timed velocity test."""

    linear_speed: float = 0.1
    angular_speed: float = 0.0
    duration: float = 3.0
    publish_rate: float = 10.0
    startup_timeout: float = 30.0
    clock_timeout: float = 2.0
    stop_duration: float = 0.5

    def __post_init__(self):
        for field in fields(self):
            if not math.isfinite(getattr(self, field.name)):
                raise ValueError(f"{field.name} must be finite")
        limits = {
            "linear_speed": (-0.22, 0.22),
            "angular_speed": (-1.0, 1.0),
            "duration": (0.01, 30.0),
            "publish_rate": (5.0, 50.0),
            "startup_timeout": (0.1, 300.0),
            "clock_timeout": (0.1, 30.0),
            "stop_duration": (0.2, 5.0),
        }
        for name, (low, high) in limits.items():
            if not low <= getattr(self, name) <= high:
                raise ValueError(f"{name} must be between {low} and {high}")


class MoveABit(Node):
    """Wait for readiness, move once, then stop without automatically restarting."""

    def __init__(self, **kwargs):
        super().__init__("move_a_bit", **kwargs)
        values = {
            field.name: self.declare_parameter(field.name, field.default).value
            for field in fields(MotionConfig)
        }
        self.config = MotionConfig(**values)
        topic = self.declare_parameter(
            "cmd_vel_topic", "/turtlebot3_test/cmd_vel"
        ).value
        # Reliable, volatile, keep-last-1: never replay a latched movement command.
        self.publisher = self.create_publisher(Twist, topic, QoSProfile(depth=1))
        self.done = False
        self.exit_code = 0
        self.state = "waiting"
        self._created_at = time.monotonic()
        self._last_ros_ns = None
        self._last_clock_advance = None
        self._motion_start_ns = None
        self._stop_until = None
        # A ROS-time timer would never fire if /clock is absent or paused.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / self.config.publish_rate, self._tick, clock=self._steady_clock
        )
        self.get_logger().info(
            f"Waiting for a subscriber on {self.publisher.topic_name}"
            + (" and advancing /clock" if self._uses_sim_time() else " (wall time)")
        )

    def _uses_sim_time(self):
        return self.get_parameter("use_sim_time").value

    def publish_stop(self):
        """Publish a fresh, all-zero Twist."""
        self.publisher.publish(Twist())

    def request_stop(self, reason, exit_code=1):
        """Enter the terminal stop phase; a second request cannot restart motion."""
        if self.state in ("stopping", "done"):
            return
        self.exit_code = exit_code
        self.state = "stopping"
        self._stop_until = time.monotonic() + self.config.stop_duration
        self.publish_stop()
        log = self.get_logger().info if exit_code == 0 else self.get_logger().warning
        log(f"{reason}; sending zero velocity")

    def _tick(self):
        wall_now = time.monotonic()
        if self.state == "done":
            return
        if self.state == "stopping":
            self.publish_stop()
            if wall_now >= self._stop_until:
                self.state = "done"
                self.done = True
            return

        ros_ns = self.get_clock().now().nanoseconds
        if self._last_ros_ns is not None and ros_ns < self._last_ros_ns:
            self.request_stop("Clock moved backwards; rerun after resetting the scene")
            return
        if self._last_ros_ns is not None and ros_ns > self._last_ros_ns:
            self._last_clock_advance = wall_now
        self._last_ros_ns = ros_ns
        clock_ready = not self._uses_sim_time() or (
            ros_ns > 0
            and self._last_clock_advance is not None
            and wall_now - self._last_clock_advance < self.config.clock_timeout
        )
        subscribers = self.publisher.get_subscription_count()

        if self.state == "waiting":
            if wall_now - self._created_at >= self.config.startup_timeout:
                self.request_stop("Startup timed out: check the subscriber, /clock and DDS")
                return
            if not clock_ready or subscribers == 0:
                return
            self.state = "moving"
            self._motion_start_ns = ros_ns
            self.get_logger().info(
                f"Moving: linear={self.config.linear_speed} m/s, "
                f"angular={self.config.angular_speed} rad/s, "
                f"duration={self.config.duration} s"
            )

        if subscribers == 0:
            self.request_stop("Command subscriber disappeared")
        elif not clock_ready:
            self.request_stop("Simulation clock stopped advancing")
        elif (ros_ns - self._motion_start_ns) / 1e9 >= self.config.duration:
            self.request_stop("Motion complete", exit_code=0)
        else:
            command = Twist()
            command.linear.x = self.config.linear_speed
            command.angular.z = self.config.angular_speed
            self.publisher.publish(command)


def main(args=None):
    """Run the node with signal handling that allows graceful stop publication."""
    # Keep the context alive on Ctrl+C/SIGTERM long enough to send stop commands.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    interrupted = threading.Event()
    previous_handlers = {}
    node = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(
                signum, lambda *_: interrupted.set()
            )
        node = MoveABit()
        while rclpy.ok() and not node.done:
            if interrupted.is_set():
                node.request_stop("Interrupted", exit_code=130)
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code if node.done else 1
    except (
        ValueError,
        rclpy.exceptions.InvalidParameterException,
        rclpy.exceptions.InvalidTopicNameException,
    ) as exc:
        print(f"Invalid motion parameters: {exc}", file=sys.stderr)
        return 2
    except ExternalShutdownException:
        return 1
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_stop()
            node.destroy_node()
        rclpy.try_shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())
