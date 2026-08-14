import copy
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _is_false(value):
    return str(value).strip().lower() in ("0", "false", "no", "off")


def _is_true(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _sanitize_namespace(raw_namespace, fallback_name):
    ns = str(raw_namespace).strip() if raw_namespace is not None else ""
    if not ns:
        ns = fallback_name
    return ns.strip("/")


def _set_child_text(parent, tag, text):
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = str(text)
    return child


def _write_temp_file(prefix, suffix, content):
    fd, file_path = tempfile.mkstemp(prefix=prefix, suffix=suffix)
    with os.fdopen(fd, "w") as output_file:
        output_file.write(content)
    return file_path


def _build_robot_description(
    robot_description_xml,
    robot_name,
    robot_namespace,
    controller_config_file,
    odom_frame,
    base_frame,
):
    root = ET.fromstring(robot_description_xml)
    if root.tag == "robot":
        root.set("name", robot_name)

    for plugin in root.iter("plugin"):
        plugin_name = plugin.get("name", "")
        if plugin_name == "gz::sim::systems::OdometryPublisher":
            _set_child_text(plugin, "odom_frame", odom_frame)
            _set_child_text(plugin, "robot_base_frame", base_frame)
        elif plugin_name == "gz_underwater_hardware::GazeboSimUnderwaterPlugin":
            _set_child_text(plugin, "parameters", controller_config_file)
            _set_child_text(plugin, "controller_manager_name", "controller_manager")
            ros = plugin.find("ros")
            if ros is None:
                ros = ET.SubElement(plugin, "ros")
            _set_child_text(ros, "namespace", f"/{robot_namespace}")

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def _resolve_rviz_template(pkg_path, raw_value, merge_tf):
    value = str(raw_value).strip()

    if not value or value == "auto":
        profile = "multi" if merge_tf else "single"
        rviz_path = os.path.join(pkg_path, "config", "subcat.rviz")
    elif value in ("single", "multi"):
        profile = value
        rviz_path = os.path.join(pkg_path, "config", "subcat.rviz")
    elif os.path.isabs(value):
        profile = "custom"
        rviz_path = value
    else:
        profile = "custom"
        rviz_path = os.path.join(pkg_path, "config", value)

    if not os.path.exists(rviz_path):
        raise RuntimeError(f"RViz config file does not exist: {rviz_path}")

    return rviz_path, profile


def _parse_robot_list(raw_value, fallback_robot):
    robots = [
        _sanitize_namespace(value, "")
        for value in str(raw_value).replace(";", ",").split(",")
        if value.strip()
    ]
    if not robots:
        robots = [fallback_robot]
    return robots


def _display_class(display):
    return display.get("Class", "") if isinstance(display, dict) else ""


def _display_name(display):
    return display.get("Name", "") if isinstance(display, dict) else ""


def _is_robot_display(display):
    return _display_class(display) == "rviz_default_plugins/RobotModel"


def _is_template_path_display(display):
    if _display_class(display) != "rviz_default_plugins/Path":
        return False
    return _display_name(display) in ("Full Trajectory", "Current Track Path")


def _find_display(displays, predicate):
    for display in displays:
        if predicate(display):
            return display
    raise RuntimeError("RViz template is missing a required display")


def _set_topic(display, field_name, topic_name):
    field = display.setdefault(field_name, {})
    field["Value"] = topic_name


def _trajectory_color(index):
    colors = [
        "25; 255; 0",
        "38; 162; 105",
        "94; 151; 246",
        "255; 190; 80",
        "191; 97; 255",
        "255; 119; 119",
    ]
    return colors[index % len(colors)]


def _current_path_color(index):
    colors = [
        "255; 0; 0",
        "255; 85; 85",
        "66; 180; 255",
        "255; 140; 40",
        "210; 120; 255",
        "160; 220; 80",
    ]
    return colors[index % len(colors)]


def _make_robot_displays(robot_template, full_path_template, current_path_template, robot_name, index):
    robot_display = copy.deepcopy(robot_template)
    robot_display["Name"] = f"RobotModel {robot_name}"
    robot_display["TF Prefix"] = robot_name
    _set_topic(robot_display, "Description Topic", f"/{robot_name}/robot_description")

    full_path_display = copy.deepcopy(full_path_template)
    full_path_display["Name"] = f"Full Trajectory {robot_name}"
    full_path_display["Color"] = _trajectory_color(index)
    _set_topic(full_path_display, "Topic", f"/{robot_name}/trajectory")

    current_path_display = copy.deepcopy(current_path_template)
    current_path_display["Name"] = f"Current Track Path {robot_name}"
    current_path_display["Color"] = _current_path_color(index)
    _set_topic(current_path_display, "Topic", f"/{robot_name}/curr_traj")

    return [robot_display, full_path_display, current_path_display]


def _set_rviz_fixed_frame(config, fixed_frame):
    visualization_manager = config.setdefault("Visualization Manager", {})
    global_options = visualization_manager.setdefault("Global Options", {})
    global_options["Fixed Frame"] = fixed_frame


def _build_dynamic_rviz_config(config, fixed_frame, robots):
    _set_rviz_fixed_frame(config, fixed_frame)

    visualization_manager = config.setdefault("Visualization Manager", {})
    displays = visualization_manager.setdefault("Displays", [])
    robot_template = _find_display(displays, _is_robot_display)
    full_path_template = _find_display(
        displays,
        lambda display: _display_name(display) == "Full Trajectory",
    )
    current_path_template = _find_display(
        displays,
        lambda display: _display_name(display) == "Current Track Path",
    )

    base_displays = [
        display
        for display in displays
        if not _is_robot_display(display) and not _is_template_path_display(display)
    ]
    new_displays = []
    inserted_robot_displays = False
    for display in base_displays:
        new_displays.append(display)
        if _display_class(display) == "rviz_default_plugins/Grid" and not inserted_robot_displays:
            for index, robot in enumerate(robots):
                new_displays.extend(
                    _make_robot_displays(
                        robot_template,
                        full_path_template,
                        current_path_template,
                        robot,
                        index,
                    )
                )
            inserted_robot_displays = True

    if not inserted_robot_displays:
        for index, robot in enumerate(robots):
            new_displays.extend(
                _make_robot_displays(
                    robot_template,
                    full_path_template,
                    current_path_template,
                    robot,
                    index,
                )
            )

    for display in new_displays:
        if _display_class(display) == "rviz_default_plugins/TF":
            display["Frames"] = {"All Enabled": True}
            display["Tree"] = {fixed_frame: {}}

    visualization_manager["Displays"] = new_displays
    return yaml.safe_dump(config, sort_keys=False)


def _build_rviz_config(template_rviz_path, fixed_frame, tf_prefix, robot_namespace, rviz_profile, rviz_robots):
    with open(template_rviz_path, "r") as rviz_input:
        rviz_config = yaml.safe_load(rviz_input)

    if rviz_profile == "multi":
        rviz_content = _build_dynamic_rviz_config(rviz_config, fixed_frame, rviz_robots)
        return _write_temp_file("rviz_", ".rviz", rviz_content)

    robot = robot_namespace.strip("/")
    if rviz_profile == "single":
        rviz_content = _build_dynamic_rviz_config(rviz_config, fixed_frame, [robot])
        rviz_config = yaml.safe_load(rviz_content)
        displays = rviz_config.get("Visualization Manager", {}).get("Displays", [])
        for display in displays:
            if _is_robot_display(display):
                display["TF Prefix"] = tf_prefix if tf_prefix else ""
        rviz_content = yaml.safe_dump(rviz_config, sort_keys=False)
        return _write_temp_file("rviz_", ".rviz", rviz_content)

    _set_rviz_fixed_frame(rviz_config, fixed_frame)
    return _write_temp_file("rviz_", ".rviz", yaml.safe_dump(rviz_config, sort_keys=False))


def _build_namespaced_controller_yaml(base_yaml, robot_namespace, path_frame):
    ns = robot_namespace.strip("/")
    controller_config = yaml.safe_load(base_yaml)
    if not isinstance(controller_config, dict):
        raise RuntimeError("Controller YAML must contain a top-level mapping")

    controller_params = (
        controller_config
        .setdefault("acados_nmpc_controller", {})
        .setdefault("ros__parameters", {})
    )
    trajectory_params = controller_params.setdefault("trajectory", {})
    trajectory_params["path_frame"] = path_frame

    namespaced_config = {
        f"/{ns}/{key}": value for key, value in controller_config.items()
    }
    return yaml.safe_dump(namespaced_config, sort_keys=False)


def _ros_topic_exists(topic_name):
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
        return False

    return topic_name in completed.stdout.splitlines()


def _ros_node_names():
    try:
        completed = subprocess.run(
            ["ros2", "node", "list"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []

    return completed.stdout.splitlines()


def _rviz_node_exists():
    return any(node.rstrip("/").split("/")[-1] == "rviz" for node in _ros_node_names())


def _should_start_gz(raw_value):
    if _is_false(raw_value):
        return False
    if _is_true(raw_value):
        return True

    if _ros_topic_exists("/clock"):
        print("[gz.launch.py] /clock already exists; skipping Gazebo launch")
        return False

    print("[gz.launch.py] /clock not found; launching Gazebo")
    return True


def _should_start_rviz(raw_value, merge_tf):
    if _is_false(raw_value):
        return False
    if _is_true(raw_value):
        return True

    if not merge_tf:
        print("[gz.launch.py] private TF mode; launching a namespaced RViz")
        return True

    if _rviz_node_exists():
        print("[gz.launch.py] RViz node already exists; skipping RViz launch")
        return False

    print("[gz.launch.py] RViz node not found; launching RViz")
    return True


def _should_bridge_clock(raw_value):
    value = str(raw_value).strip().lower()
    if _is_false(value):
        return False
    if _is_true(value):
        return True

    if _ros_topic_exists("/clock"):
        print("[gz.launch.py] /clock already exists; skipping clock bridge")
        return False

    print("[gz.launch.py] /clock not found; enabling clock bridge")
    return True


def _build_bridge_yaml(robot_name, include_clock, tf_ros_topic_name):
    lines = []

    if include_clock:
        lines.extend(
            [
                '- ros_topic_name: "/clock"',
                '  gz_topic_name: "/clock"',
                '  ros_type_name: "rosgraph_msgs/msg/Clock"',
                '  gz_type_name: "gz.msgs.Clock"',
                "  direction: GZ_TO_ROS",
                "",
            ]
        )

    lines.extend(
        [
            '- ros_topic_name: "odometry"',
            f'  gz_topic_name: "/model/{robot_name}/odometry"',
            '  ros_type_name: "nav_msgs/msg/Odometry"',
            '  gz_type_name: "gz.msgs.Odometry"',
            "  direction: GZ_TO_ROS",
            "",
            f'- ros_topic_name: "{tf_ros_topic_name}"',
            f'  gz_topic_name: "/model/{robot_name}/pose"',
            '  ros_type_name: "tf2_msgs/msg/TFMessage"',
            '  gz_type_name: "gz.msgs.Pose_V"',
            "  direction: GZ_TO_ROS",
            "",
        ]
    )

    return "\n".join(lines)


def launch_setup(context, *args, **kwargs):
    use_sim_time = _as_bool(context.launch_configurations["use_sim_time"])
    pkg_description = context.launch_configurations["pkg_description"]
    pkg_path = os.path.join(get_package_share_directory(pkg_description))
    world = context.launch_configurations["world"]
    default_sdf_path = os.path.join(pkg_path, "worlds", world + ".world")

    robot_name = context.launch_configurations["robot_name"]
    robot_namespace = _sanitize_namespace(
        context.launch_configurations["robot_namespace"], robot_name
    )
    rviz_robots = _parse_robot_list(
        context.launch_configurations.get("rviz_robots", ""),
        robot_namespace,
    )
    controller_manager_path = f"/{robot_namespace}/controller_manager"

    merge_tf = _as_bool(context.launch_configurations["merge_tf"])
    start_gz = _should_start_gz(context.launch_configurations["start_gz"])
    start_rviz = _should_start_rviz(context.launch_configurations["start_rviz"], merge_tf)
    bridge_clock = _should_bridge_clock(context.launch_configurations["bridge_clock"])
    global_fixed_frame = context.launch_configurations.get("global_fixed_frame", "map")
    legacy_shared_frame = context.launch_configurations.get("shared_odom_frame", "")
    if not global_fixed_frame and legacy_shared_frame:
        global_fixed_frame = legacy_shared_frame

    raw_tf_prefix = context.launch_configurations.get("tf_prefix", "")
    tf_prefix = _sanitize_namespace(raw_tf_prefix, "")
    if merge_tf and not tf_prefix:
        tf_prefix = robot_namespace

    if merge_tf:
        odom_frame = f"{tf_prefix}/odom" if tf_prefix else "odom"
        base_frame = f"{tf_prefix}/base_link" if tf_prefix else "base_link"
        rviz_fixed_frame = global_fixed_frame
    else:
        odom_frame = f"{tf_prefix}/odom" if tf_prefix else "odom"
        base_frame = f"{tf_prefix}/base_link" if tf_prefix else "base_link"
        rviz_fixed_frame = odom_frame

    tf_ros_topic_name = "/tf" if merge_tf else "tf"
    tf_remappings = [] if merge_tf else [("/tf", "tf"), ("/tf_static", "tf_static")]

    init_x = context.launch_configurations["x"]
    init_y = context.launch_configurations["y"]
    init_height = context.launch_configurations["height"]

    controller_yaml_file = os.path.join(pkg_path, "config", "gazebo.yaml")
    with open(controller_yaml_file, "r") as controller_input:
        controller_yaml = controller_input.read()

    namespaced_controller_yaml = _build_namespaced_controller_yaml(
        controller_yaml,
        robot_namespace,
        path_frame=odom_frame if merge_tf else "odom",
    )
    controller_config_file = _write_temp_file(
        f"{robot_name}_controllers_", ".yaml", namespaced_controller_yaml
    )

    xacro_file = os.path.join(pkg_path, "urdf", "subcat_gz.xacro")
    robot_description = _build_robot_description(
        xacro.process_file(xacro_file).toxml(),
        robot_name,
        robot_namespace,
        controller_config_file,
        odom_frame,
        base_frame,
    )

    bridge_yaml_content = _build_bridge_yaml(
        robot_name,
        bridge_clock,
        tf_ros_topic_name=tf_ros_topic_name,
    )
    bridge_config_file = _write_temp_file(
        f"{robot_name}_bridge_", ".yaml", bridge_yaml_content
    )

    robot_state_publisher_params = {
        "publish_frequency": 20.0,
        "use_tf_static": True,
        "robot_description": robot_description,
        "ignore_timestamp": True,
    }
    if tf_prefix:
        robot_state_publisher_params["frame_prefix"] = f"{tf_prefix}/"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_namespace,
        remappings=tf_remappings,
        parameters=[
            robot_state_publisher_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    global_to_odom_tf = None
    if merge_tf:
        global_to_odom_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{robot_namespace}_global_to_odom",
            namespace=robot_namespace,
            remappings=tf_remappings,
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                rviz_fixed_frame,
                odom_frame,
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            f"/{robot_namespace}/robot_description",
            "-name",
            robot_name,
            "-allow_renaming",
            "false",
            "-x",
            init_x,
            "-y",
            init_y,
            "-z",
            init_height,
        ],
    )

    rviz_template_file, rviz_profile = _resolve_rviz_template(
        pkg_path,
        context.launch_configurations["rviz_config"],
        merge_tf,
    )
    rviz_config_file = _build_rviz_config(
        rviz_template_file,
        fixed_frame=rviz_fixed_frame,
        tf_prefix=tf_prefix if merge_tf else "",
        robot_namespace=robot_namespace,
        rviz_profile=rviz_profile,
        rviz_robots=rviz_robots,
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        namespace=robot_namespace,
        remappings=tf_remappings,
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_path,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            controller_manager_path,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
    acados_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        arguments=[
            "acados_nmpc_controller",
            "--controller-manager",
            controller_manager_path,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {
                "config_file": bridge_config_file,
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    launch_nodes = []

    if start_gz:
        launch_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[("gz_args", [" -r -v 4 ", default_sdf_path])],
            )
        )

    launch_nodes.extend(
        [
            gz_spawn_entity,
            gz_bridge_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_publisher, imu_sensor_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_publisher,
                    on_exit=[acados_controller],
                )
            ),
        ]
    )

    if start_rviz:
        launch_nodes.insert(0, rviz)

    launch_nodes.insert(0, robot_state_publisher)

    if global_to_odom_tf:
        launch_nodes.insert(0, global_to_odom_tf)

    return launch_nodes


def generate_launch_description():
    world = DeclareLaunchArgument(
        "world",
        default_value="subcat_underwater",
        description="The world to load",
    )
    pkg_description = DeclareLaunchArgument(
        "pkg_description",
        default_value="subcat",
        description="package for robot description",
    )
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    height = DeclareLaunchArgument(
        "height", default_value="-3", description="Init height in simulation"
    )
    x = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial x position in simulation"
    )
    y = DeclareLaunchArgument(
        "y", default_value="0.0", description="Initial y position in simulation"
    )
    robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="subcat",
        description="Gazebo model name for this robot instance",
    )
    robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="subcat",
        description="ROS namespace used to isolate this robot stack",
    )
    start_gz = DeclareLaunchArgument(
        "start_gz",
        default_value="auto",
        description="Launch Gazebo when true; auto skips it if /clock is already available",
    )
    start_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="auto",
        description="Launch RViz when true; auto skips it if an RViz node already exists",
    )
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value="single",
        description="RViz config: auto, single, multi, a config filename, or an absolute path",
    )
    rviz_robots = DeclareLaunchArgument(
        "rviz_robots",
        default_value="",
        description="Comma-separated robot namespaces to show when rviz_config is multi",
    )
    bridge_clock = DeclareLaunchArgument(
        "bridge_clock",
        default_value="auto",
        description="Bridge /clock when true; auto bridges only if /clock is not already available",
    )
    merge_tf = DeclareLaunchArgument(
        "merge_tf",
        default_value="true",
        description="Merge all robot TF into global /tf with unique prefixed frame IDs",
    )
    tf_prefix = DeclareLaunchArgument(
        "tf_prefix",
        default_value="",
        description="Optional TF frame prefix for this robot; defaults to robot_namespace when merge_tf=true",
    )
    shared_odom_frame = DeclareLaunchArgument(
        "shared_odom_frame",
        default_value="",
        description="Deprecated alias of global_fixed_frame",
    )
    global_fixed_frame = DeclareLaunchArgument(
        "global_fixed_frame",
        default_value="map",
        description="Global fixed frame used to connect all robot odom frames in merge_tf mode",
    )
    return LaunchDescription(
        [
            world,
            pkg_description,
            height,
            x,
            y,
            use_sim_time,
            robot_name,
            robot_namespace,
            start_gz,
            start_rviz,
            rviz_config,
            rviz_robots,
            bridge_clock,
            merge_tf,
            tf_prefix,
            shared_odom_frame,
            global_fixed_frame,
            OpaqueFunction(function=launch_setup),
        ]
    )
