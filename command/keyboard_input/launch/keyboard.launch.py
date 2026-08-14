import re
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _auto_discover_enabled(raw_value, robots_value):
    normalized = str(raw_value).strip().lower()
    if normalized == "auto":
        return str(robots_value).strip().lower() == "auto"
    return _as_bool(normalized)


def _split_robot_list(raw_value):
    robots = []
    for value in str(raw_value).replace(";", ",").split(","):
        robot = value.strip().strip("/")
        if robot:
            robots.append(robot)
    return robots


def _natural_key(value):
    return [
        int(part) if part.isdigit() else part
        for part in re.split(r"(\d+)", value)
    ]


def _ros_topic_list():
    try:
        completed = subprocess.run(
            ["ros2", "topic", "list"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    return completed.stdout.splitlines()


def _robot_namespace_from_topic(topic_name, leaf_name):
    suffix = "/" + leaf_name
    if topic_name == suffix:
        return ""
    if not topic_name.endswith(suffix):
        return None
    namespace = topic_name[: -len(suffix)].strip("/")
    return namespace if namespace else None


def _discover_robot_namespaces():
    topic_names = _ros_topic_list()
    by_namespace = {}
    for topic_name in topic_names:
        for leaf_name in ("control_input", "cmd_vel"):
            namespace = _robot_namespace_from_topic(topic_name, leaf_name)
            if namespace is None:
                continue
            by_namespace.setdefault(namespace, set()).add(leaf_name)

    robots = [
        namespace
        for namespace, leaves in by_namespace.items()
        if {"control_input", "cmd_vel"}.issubset(leaves)
    ]
    return sorted(robots, key=_natural_key)


def _discover_robot_namespaces_until(timeout_seconds):
    deadline = time.monotonic() + max(0.0, timeout_seconds)
    while True:
        robots = _discover_robot_namespaces()
        if robots or time.monotonic() >= deadline:
            return robots
        time.sleep(0.25)


def _terminal_prefix(enabled):
    if not enabled:
        return None
    return (
        "bash -lc 'if [ -r /dev/tty ]; then "
        "exec \"$0\" \"$@\" < /dev/tty; "
        "else exec \"$0\" \"$@\"; fi'"
    )


def launch_setup(context, *args, **kwargs):
    raw_robots = context.launch_configurations["robots"]
    auto_discover = _auto_discover_enabled(context.launch_configurations["auto_discover"], raw_robots)
    robots = [] if raw_robots.strip().lower() == "auto" else _split_robot_list(raw_robots)

    if not robots:
        timeout = float(context.launch_configurations["wait_timeout"])
        robots = _discover_robot_namespaces_until(timeout)

    if robots:
        print(f"[keyboard.launch.py] keyboard targets: {', '.join(robots)}")
        robots_parameter = ",".join(robots)
    elif auto_discover:
        print("[keyboard.launch.py] no robot command topics found yet; keyboard node will keep discovering")
        robots_parameter = ""
    else:
        print("[keyboard.launch.py] no robot command topics found; falling back to relative topics")
        robots_parameter = ""

    return [
        Node(
            package="keyboard_input",
            executable="keyboard_input",
            name="keyboard_input_node",
            output="screen",
            prefix=_terminal_prefix(_as_bool(context.launch_configurations["use_tty"])),
            parameters=[
                {
                    "robots": robots_parameter,
                    "auto_discover": auto_discover,
                    "active_robot": context.launch_configurations["active_robot"],
                    "linear_speed": float(context.launch_configurations["linear_speed"]),
                    "vertical_speed": float(context.launch_configurations["vertical_speed"]),
                    "yaw_rate": float(context.launch_configurations["yaw_rate"]),
                    "repeat_rate": float(context.launch_configurations["repeat_rate"]),
                    "command_timeout": float(context.launch_configurations["command_timeout"]),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robots",
                default_value="auto",
                description="Comma-separated robot namespaces, or auto to discover current command topics",
            ),
            DeclareLaunchArgument(
                "wait_timeout",
                default_value="5.0",
                description="Seconds to wait for robot command topics when robots is auto",
            ),
            DeclareLaunchArgument(
                "use_tty",
                default_value="true",
                description="Read keyboard input from /dev/tty so the node works from ros2 launch",
            ),
            DeclareLaunchArgument(
                "active_robot",
                default_value="",
                description="Initial active robot namespace, empty selects the first discovered robot",
            ),
            DeclareLaunchArgument(
                "auto_discover",
                default_value="auto",
                description="true refreshes robot command topics at runtime; auto enables it only when robots is auto",
            ),
            DeclareLaunchArgument("linear_speed", default_value="0.6"),
            DeclareLaunchArgument("vertical_speed", default_value="0.6"),
            DeclareLaunchArgument("yaw_rate", default_value="0.5"),
            DeclareLaunchArgument("repeat_rate", default_value="50.0"),
            DeclareLaunchArgument("command_timeout", default_value="0.12"),
            OpaqueFunction(function=launch_setup),
        ]
    )
