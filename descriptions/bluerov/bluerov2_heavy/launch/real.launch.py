import copy
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


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


def _write_temp_file(prefix, suffix, content):
    fd, file_path = tempfile.mkstemp(prefix=prefix, suffix=suffix)
    with os.fdopen(fd, "w") as output_file:
        output_file.write(content)
    return file_path


def _set_named_param(parent, name, value):
    for param in parent.findall("param"):
        if param.get("name") == name:
            param.text = str(value)
            return
    param = ET.SubElement(parent, "param")
    param.set("name", name)
    param.text = str(value)


def _build_robot_description(robot_description_xml, robot_name, hardware_params):
    root = ET.fromstring(robot_description_xml)
    if root.tag == "robot":
        root.set("name", robot_name)

    for hardware in root.iter("hardware"):
        plugin = hardware.find("plugin")
        if plugin is None or (plugin.text or "").strip() != "real_underwater_hardware/RealSystem":
            continue
        for name, value in hardware_params.items():
            _set_named_param(hardware, name, value)

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def _parse_robot_list(raw_value, fallback_robot):
    robots = [
        _sanitize_namespace(value, "")
        for value in str(raw_value).replace(";", ",").split(",")
        if value.strip()
    ]
    return robots if robots else [fallback_robot]


def _display_class(display):
    return display.get("Class", "") if isinstance(display, dict) else ""


def _display_name(display):
    return display.get("Name", "") if isinstance(display, dict) else ""


def _is_robot_display(display):
    return _display_class(display) == "rviz_default_plugins/RobotModel"


def _is_template_path_display(display):
    return (
        _display_class(display) == "rviz_default_plugins/Path"
        and _display_name(display) in ("Full Trajectory", "Current Track Path")
    )


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
                _make_robot_displays(robot_template, full_path_template, current_path_template, robot, index)
            )

    for display in new_displays:
        if _display_class(display) == "rviz_default_plugins/TF":
            display["Frames"] = {"All Enabled": True}
            display["Tree"] = {fixed_frame: {}}

    visualization_manager["Displays"] = new_displays
    return yaml.safe_dump(config, sort_keys=False)


def _resolve_rviz_template(pkg_path, raw_value, merge_tf):
    value = str(raw_value).strip()
    if not value or value == "auto":
        profile = "multi" if merge_tf else "single"
        rviz_path = os.path.join(pkg_path, "config", "single.rviz")
    elif value in ("single", "multi"):
        profile = value
        rviz_path = os.path.join(pkg_path, "config", "single.rviz")
    elif os.path.isabs(value):
        profile = "custom"
        rviz_path = value
    else:
        profile = "custom"
        rviz_path = os.path.join(pkg_path, "config", value)

    if not os.path.exists(rviz_path):
        raise RuntimeError(f"RViz config file does not exist: {rviz_path}")
    return rviz_path, profile


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
        return _write_temp_file("rviz_", ".rviz", yaml.safe_dump(rviz_config, sort_keys=False))

    _set_rviz_fixed_frame(rviz_config, fixed_frame)
    return _write_temp_file("rviz_", ".rviz", yaml.safe_dump(rviz_config, sort_keys=False))


def _build_namespaced_controller_yaml(base_yaml, robot_namespace, path_frame, imu_frame):
    ns = robot_namespace.strip("/")
    controller_config = yaml.safe_load(base_yaml)
    if not isinstance(controller_config, dict):
        raise RuntimeError("Controller YAML must contain a top-level mapping")

    controller_params = (
        controller_config
        .setdefault("acados_nmpc_controller", {})
        .setdefault("ros__parameters", {})
    )
    controller_params.setdefault("trajectory", {})["path_frame"] = path_frame

    imu_params = (
        controller_config
        .setdefault("imu_sensor_broadcaster", {})
        .setdefault("ros__parameters", {})
    )
    imu_params["frame_id"] = imu_frame

    namespaced_config = {f"/{ns}/{key}": value for key, value in controller_config.items()}
    return yaml.safe_dump(namespaced_config, sort_keys=False)


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


def _should_start_rviz(raw_value, merge_tf):
    if _is_false(raw_value):
        return False
    if _is_true(raw_value):
        return True
    if not merge_tf:
        print("[real.launch.py] private TF mode; launching a namespaced RViz")
        return True
    if _rviz_node_exists():
        print("[real.launch.py] RViz node already exists; skipping RViz launch")
        return False
    print("[real.launch.py] RViz node not found; launching RViz")
    return True


def launch_setup(context, *args, **kwargs):
    use_sim_time = _as_bool(context.launch_configurations["use_sim_time"])
    pkg_description = context.launch_configurations["pkg_description"]
    pkg_path = get_package_share_directory(pkg_description)

    robot_name = context.launch_configurations["robot_name"]
    robot_namespace = _sanitize_namespace(context.launch_configurations["robot_namespace"], robot_name)
    rviz_robots = _parse_robot_list(context.launch_configurations.get("rviz_robots", ""), robot_namespace)

    merge_tf = _as_bool(context.launch_configurations["merge_tf"])
    start_rviz = _should_start_rviz(context.launch_configurations["start_rviz"], merge_tf)
    global_fixed_frame = context.launch_configurations.get("global_fixed_frame", "map")
    legacy_shared_frame = context.launch_configurations.get("shared_odom_frame", "")
    if not global_fixed_frame and legacy_shared_frame:
        global_fixed_frame = legacy_shared_frame

    tf_prefix = _sanitize_namespace(context.launch_configurations.get("tf_prefix", ""), "")
    if merge_tf and not tf_prefix:
        tf_prefix = robot_namespace

    odom_frame = f"{tf_prefix}/odom" if tf_prefix else "odom"
    imu_frame = f"{tf_prefix}/imu_link" if tf_prefix else "imu_link"
    rviz_fixed_frame = global_fixed_frame if merge_tf else odom_frame
    tf_remappings = [] if merge_tf else [("/tf", "tf"), ("/tf_static", "tf_static")]

    controller_yaml_file = os.path.join(pkg_path, "config", "real.yaml")
    with open(controller_yaml_file, "r") as controller_input:
        controller_yaml = controller_input.read()
    controller_config_file = _write_temp_file(
        f"{robot_name}_real_controllers_",
        ".yaml",
        _build_namespaced_controller_yaml(controller_yaml, robot_namespace, odom_frame if merge_tf else "odom", imu_frame),
    )

    xacro_file = os.path.join(pkg_path, "urdf", "bluerov2_heavy.xacro")
    hardware_params = {
        "robot_name": robot_name,
        "mavros_namespace": context.launch_configurations["mavros_namespace"],
    }
    robot_description = _build_robot_description(
        xacro.process_file(xacro_file).toxml(),
        robot_name,
        hardware_params,
    )

    controller_manager_path = f"/{robot_namespace}/controller_manager"

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
        parameters=[robot_state_publisher_params, {"use_sim_time": use_sim_time}],
    )

    global_to_odom_tf = None
    if merge_tf:
        global_to_odom_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{robot_namespace}_global_to_odom",
            namespace=robot_namespace,
            remappings=tf_remappings,
            arguments=["0", "0", "0", "0", "0", "0", rviz_fixed_frame, odom_frame],
            parameters=[{"use_sim_time": use_sim_time}],
        )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controller_config_file,
            {"use_sim_time": use_sim_time},
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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        arguments=["imu_sensor_broadcaster", "--controller-manager", controller_manager_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    acados_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        arguments=["acados_nmpc_controller", "--controller-manager", controller_manager_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    launch_nodes = [
        controller_manager_node,
        robot_state_publisher,
        joint_state_publisher,
        imu_sensor_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[acados_controller],
            )
        ),
    ]

    if global_to_odom_tf:
        launch_nodes.insert(0, global_to_odom_tf)
    if start_rviz:
        launch_nodes.insert(0, rviz)

    return launch_nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pkg_description", default_value="bluerov2_heavy"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("robot_name", default_value="bluerov2_heavy"),
            DeclareLaunchArgument("robot_namespace", default_value="bluerov2_heavy"),
            DeclareLaunchArgument("start_rviz", default_value="auto"),
            DeclareLaunchArgument("rviz_config", default_value="single"),
            DeclareLaunchArgument("rviz_robots", default_value=""),
            DeclareLaunchArgument("merge_tf", default_value="true"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("shared_odom_frame", default_value=""),
            DeclareLaunchArgument("global_fixed_frame", default_value="map"),
            DeclareLaunchArgument("mavros_namespace", default_value="/mavros"),
            OpaqueFunction(function=launch_setup),
        ]
    )
