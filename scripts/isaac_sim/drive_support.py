"""Simulator-side velocity watchdog, independent of Isaac Sim and ROS imports."""

from dataclasses import dataclass
import math
import operator


TURTLEBOT3_ASSET_PATHS = (
    # Isaac Sim 6.2: the tutorial asset was renamed and the multiphysics asset
    # lives in a directory named after the robot, not directly under Turtlebot3.
    "/Isaac/Robots/Turtlebot/Turtlebot3/tb3_burger_processed.usda",
    "/Isaac/Robots_Multiphysics/Turtlebot/Turtlebot3/"
    "turtlebot3_burger/turtlebot3_burger.usda",
    "/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd",
    "/Isaac/Robots/Turtlebot/turtlebot3_burger.usd",
)


def robot_asset_candidates(asset_root, robot_usd=None):
    """Prefer current Burger assets without overriding an explicit user choice."""
    if robot_usd:
        return [robot_usd]
    if not asset_root:
        raise ValueError("Isaac assets unavailable; configure assets or pass --robot-usd")
    return [asset_root.rstrip("/") + path for path in TURTLEBOT3_ASSET_PATHS]


def configure_drive(robot, controller):
    """Attach physics handles and restore wheel velocity control after a reset."""
    robot.initialize()
    if not robot.handles_initialized:
        raise RuntimeError("TurtleBot3 articulation physics handles did not initialize")
    try:
        indices = tuple(operator.index(index) for index in robot.wheel_dof_indices)
        num_dof = operator.index(robot.num_dof)
    except TypeError as error:
        raise RuntimeError("TurtleBot3 wheel joint indices are unavailable") from error
    if len(indices) != 2 or len(set(indices)) != 2 or any(
        index < 0 or index >= num_dof for index in indices
    ):
        raise RuntimeError(f"Invalid TurtleBot3 wheel joint indices: {indices}")
    controller.reset()
    robot.get_articulation_controller().switch_control_mode(mode="velocity")
    return indices


def step_robot(world, robot, controller, watchdog, wall_time, render=True):
    """Step physics without using stale handles or replaying pre-reset commands."""
    if not world.is_playing():
        watchdog.clear()
        world.step(render=render)
        return False
    if not robot.handles_initialized:
        watchdog.clear()
        world.reset()
        configure_drive(robot, controller)
    command = watchdog.command(wall_time, world.current_time)
    robot.apply_wheel_actions(controller.forward(list(command)))
    # This update processes GUI Stop/Pause events, which may invalidate handles
    # even though is_playing() was true at the beginning of this frame.
    world.step(render=render)
    ready = world.is_playing() and robot.handles_initialized
    if not ready:
        watchdog.clear()
    return ready


def finite_vector(value, size):
    """Return a finite vector, rejecting None/scalars before odometry arithmetic."""
    try:
        vector = tuple(float(component) for component in value)
    except (TypeError, ValueError, OverflowError):
        return None
    if len(vector) != size or not all(math.isfinite(component) for component in vector):
        return None
    return vector


@dataclass(frozen=True)
class RobotSample:
    """Validated root-body measurements, not estimates or fabricated zero values."""

    position: tuple
    orientation: tuple
    linear_velocity: tuple
    angular_velocity: tuple


def read_robot_sample(robot):
    """Skip unavailable physics measurements instead of crashing or faking odometry."""
    if not robot.handles_initialized:
        return None
    pose = robot.get_world_pose()
    try:
        position, orientation = pose
    except (TypeError, ValueError):
        return None
    position = finite_vector(position, 3)
    orientation = finite_vector(orientation, 4)
    linear = finite_vector(robot.get_linear_velocity(), 3)
    angular = finite_vector(robot.get_angular_velocity(), 3)
    if (
        not robot.handles_initialized
        or position is None
        or orientation is None
        or linear is None
        or angular is None
    ):
        return None
    norm = math.sqrt(sum(component * component for component in orientation))
    if not math.isfinite(norm) or norm < 1e-12:
        return None
    orientation = tuple(component / norm for component in orientation)
    return RobotSample(position, orientation, linear, angular)


class CommandWatchdog:
    """Expire commands independently in wall time and simulation time."""

    def __init__(self, timeout=0.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("command timeout must be finite and positive")
        self.timeout = timeout
        self.clear()

    def clear(self):
        """Drop the saved command, including any command from before a reset."""
        self._command = (0.0, 0.0)
        self._wall_stamp = None
        self._sim_stamp = None

    def receive(self, linear, angular, wall_time, sim_time):
        """Save a bounded command, or stop immediately on invalid input."""
        if (
            not all(math.isfinite(value) for value in (linear, angular))
            or abs(linear) > 0.22
            or abs(angular) > 1.0
        ):
            self.clear()
            return False
        self._command = (linear, angular)
        self._wall_stamp = wall_time
        self._sim_stamp = sim_time
        return True

    def command(self, wall_time, sim_time):
        """Return a fresh velocity pair or zero after timeout/reset."""
        if self._wall_stamp is not None and self._sim_stamp is not None:
            wall_age = wall_time - self._wall_stamp
            sim_age = sim_time - self._sim_stamp
            if 0 <= wall_age < self.timeout and 0 <= sim_age < self.timeout:
                return self._command
        # Clear expired commands permanently, including after a timeline reset.
        self.clear()
        return (0.0, 0.0)
