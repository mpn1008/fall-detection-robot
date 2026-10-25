"""Real rclpy pub/sub tests with a fake simulator clock, not Isaac physics tests."""

import math
import signal
import subprocess
import sys
import time
import uuid

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock

from turtlebot3_motion.move_a_bit import MotionConfig, MoveABit


@pytest.mark.parametrize("name", list(MotionConfig.__dataclass_fields__))
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_rejects_non_finite_parameters(name, value):
    with pytest.raises(ValueError, match="finite"):
        MotionConfig(**{name: value})


@pytest.mark.parametrize(
    "name,value",
    [
        ("linear_speed", 0.23),
        ("linear_speed", -0.23),
        ("angular_speed", 1.01),
        ("duration", 0.0),
        ("duration", 31.0),
        ("publish_rate", 0.0),
        ("startup_timeout", 0.0),
        ("clock_timeout", 0.0),
        ("stop_duration", 0.0),
    ],
)
def test_rejects_out_of_range_parameters(name, value):
    with pytest.raises(ValueError, match=name):
        MotionConfig(**{name: value})


@pytest.fixture
def ros_context(monkeypatch, tmp_path):
    # Force real UDP, but only on loopback and domain 42. No test commands reach
    # Windows, a physical robot, or the project's normal ROS domain.
    profile = tmp_path / "test-fastdds.xml"
    profile.write_text(
        """<?xml version="1.0"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor>
    <transport_id>test_udp</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="test" is_default_profile="true"><rtps>
    <builtin>
      <initialPeersList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></initialPeersList>
      <metatrafficUnicastLocatorList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></metatrafficUnicastLocatorList>
    </builtin>
    <defaultUnicastLocatorList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></defaultUnicastLocatorList>
    <userTransports><transport_id>test_udp</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles>""",
        encoding="utf-8",
    )
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", str(profile))
    monkeypatch.setenv("FASTDDS_DEFAULT_PROFILES_FILE", str(profile))
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    context = Context()
    rclpy.init(context=context, domain_id=42, signal_handler_options=SignalHandlerOptions.NO)
    yield context
    context.try_shutdown()


class SimulatorProbe:
    def __init__(self, context, subscribe_commands=True, start_motion=True, **overrides):
        topic = f"/test_motion_{uuid.uuid4().hex}/cmd_vel"
        self.topic = topic
        parameters = {
            "cmd_vel_topic": topic,
            "use_sim_time": True,
            "duration": 0.4,
            "publish_rate": 20.0,
            "startup_timeout": 2.0,
            "clock_timeout": 0.25,
            "stop_duration": 0.2,
            **overrides,
        }
        self.motion = None
        if start_motion:
            self.motion = MoveABit(
                context=context,
                parameter_overrides=[
                    Parameter(name, value=value) for name, value in parameters.items()
                ],
            )
        self.observer = Node("simulator_probe", context=context)
        self.clock_pub = self.observer.create_publisher(
            Clock, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.messages = []
        self.cmd_sub = None
        if subscribe_commands:
            self.cmd_sub = self.observer.create_subscription(Twist, topic, self._receive, 10)
        self.executor = SingleThreadedExecutor(context=context)
        if self.motion is not None:
            self.executor.add_node(self.motion)
        self.executor.add_node(self.observer)
        self.sim_ns = 10_000_000_000

    def _receive(self, message):
        self.messages.append((self.sim_ns, message))

    def pump_until(self, predicate, timeout=3.0, publish_clock=True, advance=True):
        last = time.monotonic()
        deadline = last + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            if advance:
                self.sim_ns += round((now - last) * 1e9)
            last = now
            if publish_clock:
                message = Clock()
                message.clock.sec, message.clock.nanosec = divmod(self.sim_ns, 1_000_000_000)
                self.clock_pub.publish(message)
            self.executor.spin_once(timeout_sec=0.01)
            if predicate():
                return True
        return predicate()

    def close(self):
        self.executor.shutdown()
        if self.motion is not None:
            self.motion.destroy_node()
        self.observer.destroy_node()


@pytest.fixture
def probe(ros_context):
    probes = []

    def create(**overrides):
        result = SimulatorProbe(ros_context, **overrides)
        probes.append(result)
        return result

    yield create
    for item in probes:
        item.close()


def moving_messages(probe):
    return [(stamp, msg) for stamp, msg in probe.messages if msg.linear.x or msg.angular.z]


def test_moves_for_sim_duration_then_repeatedly_stops(probe):
    sim = probe()
    assert sim.pump_until(lambda: sim.motion.done)
    # Drain the observer after the node's final stop callback.
    sim.pump_until(lambda: False, timeout=0.1)
    moving = moving_messages(sim)
    assert len(moving) >= 2
    assert all(msg.linear.x == 0.1 and msg.angular.z == 0.0 for _, msg in moving)
    first_stop = next(stamp for stamp, msg in sim.messages if not msg.linear.x)
    assert 0.30 <= (first_stop - moving[0][0]) / 1e9 <= 0.65
    assert all(msg.linear.x == 0.0 and msg.angular.z == 0.0 for _, msg in sim.messages[-3:])
    assert sim.motion.exit_code == 0


def test_waits_for_clock_before_moving(probe):
    sim = probe()
    sim.pump_until(lambda: False, timeout=0.2, publish_clock=False)
    assert sim.messages == []
    assert sim.motion.state == "waiting"
    assert sim.pump_until(lambda: sim.motion.done)
    assert moving_messages(sim)


def test_missing_clock_times_out_without_motion(probe):
    sim = probe(startup_timeout=0.3)
    assert sim.pump_until(lambda: sim.motion.done, publish_clock=False)
    assert not moving_messages(sim)
    assert sim.motion.exit_code == 1


def test_missing_subscriber_times_out_without_motion(probe):
    sim = probe(subscribe_commands=False, startup_timeout=0.3)
    assert sim.pump_until(lambda: sim.motion.done)
    assert sim.motion._motion_start_ns is None
    assert sim.motion.exit_code == 1


def test_paused_clock_aborts_with_stop_commands(probe):
    sim = probe(duration=5.0)
    assert sim.pump_until(lambda: bool(moving_messages(sim)))
    assert sim.pump_until(lambda: sim.motion.done, advance=False)
    assert sim.motion.exit_code == 1
    assert sim.messages[-1][1].linear.x == 0.0


def test_clock_reset_aborts_instead_of_restarting_motion(probe):
    sim = probe(duration=5.0)
    assert sim.pump_until(lambda: bool(moving_messages(sim)))
    sim.sim_ns = 1_000_000_000
    assert sim.pump_until(lambda: sim.motion.done)
    assert sim.motion.exit_code == 1
    assert sim.messages[-1][1].linear.x == 0.0


def test_disappearing_subscriber_aborts(probe):
    sim = probe(duration=5.0)
    assert sim.pump_until(lambda: bool(moving_messages(sim)))
    sim.observer.destroy_subscription(sim.cmd_sub)
    assert sim.pump_until(lambda: sim.motion.done)
    assert sim.motion.exit_code == 1


def test_interruption_sends_repeated_stops(probe):
    sim = probe(duration=5.0)
    assert sim.pump_until(lambda: bool(moving_messages(sim)))
    sim.motion.request_stop("Interrupted", exit_code=130)
    assert sim.pump_until(lambda: sim.motion.done)
    assert sim.motion.exit_code == 130
    assert all(msg.linear.x == 0.0 for _, msg in sim.messages[-3:])


def test_wall_time_mode_does_not_need_clock(probe):
    sim = probe(use_sim_time=False)
    assert sim.pump_until(lambda: sim.motion.done, publish_clock=False)
    assert moving_messages(sim)
    assert sim.motion.exit_code == 0


def start_process(sim, duration):
    return subprocess.Popen(
        [
            sys.executable, "-m", "turtlebot3_motion.move_a_bit", "--ros-args",
            "-p", f"cmd_vel_topic:={sim.topic}",
            "-p", "use_sim_time:=true",
            "-p", f"duration:={duration}",
            "-p", "startup_timeout:=5.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def clean_up_process(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
    process.stdout.close()


def test_separate_process_receives_clock_and_sends_motion_over_udp(probe):
    sim = probe(start_motion=False)
    process = start_process(sim, duration=0.5)
    try:
        assert sim.pump_until(lambda: process.poll() is not None, timeout=8.0)
        output = process.communicate(timeout=1.0)[0]
        assert process.returncode == 0, output
        assert "XMLPARSER" not in output, output
        assert "Motion complete" in output
        assert len(moving_messages(sim)) >= 2
        assert all(msg.linear.x == 0.0 for _, msg in sim.messages[-3:])
    finally:
        clean_up_process(process)


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_signal_keeps_process_alive_to_send_stops_over_udp(probe, signum):
    sim = probe(start_motion=False)
    process = start_process(sim, duration=10.0)
    try:
        assert sim.pump_until(lambda: bool(moving_messages(sim)), timeout=6.0)
        process.send_signal(signum)
        assert sim.pump_until(lambda: process.poll() is not None, timeout=3.0)
        output = process.communicate(timeout=1.0)[0]
        assert process.returncode == 130, output
        assert "XMLPARSER" not in output, output
        assert "Interrupted" in output
        assert all(msg.linear.x == 0.0 for _, msg in sim.messages[-3:])
    finally:
        clean_up_process(process)


def test_cli_rejects_unsafe_speed(ros_context):
    result = subprocess.run(
        [
            sys.executable, "-m", "turtlebot3_motion.move_a_bit", "--ros-args",
            "-p", "linear_speed:=0.23",
        ],
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    assert result.returncode == 2, result.stderr
    assert "Invalid motion parameters" in result.stderr
    assert "Traceback" not in result.stderr
    assert "XMLPARSER" not in result.stdout + result.stderr
