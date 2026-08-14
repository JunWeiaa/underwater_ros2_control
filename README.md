# underwater_ros2_control

ROS 2 sim-to-real control framework for underwater robots, combining Gazebo simulation, `ros2_control` hardware interfaces, a MAVROS-based real-hardware backend, an acados NMPC controller, BlueROV2 Heavy/Subcat robot descriptions, and keyboard teleoperation.

> Project status: research and engineering prototype. The simulation stack is intended for algorithm development and validation. The real-hardware stack can send commands directly to MAVROS `rc/override` and serial servos, so use it only with an emergency stop, actuator limits, removed propellers, or a controlled test tank.

## Contents

- [Features](#features)
- [Supported Robots](#supported-robots)
- [Repository Layout](#repository-layout)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Simulation](#simulation)
- [Keyboard Control](#keyboard-control)
- [Real Hardware](#real-hardware)
- [Interfaces and Topics](#interfaces-and-topics)
- [Configuration](#configuration)
- [Debugging](#debugging)
- [Development, Testing, and Contributing](#development-testing-and-contributing)
- [License and Acknowledgements](#license-and-acknowledgements)

## Features

- `ros2_control` controller: `acados_nmpc_controller/Acados_NMPC_Controller`, with NO_OUTPUT, AUTO, and MANUAL modes.
- Dynamic acados solver loading through `solver_id`, with bundled solver metadata for BlueROV2 Heavy and Subcat.
- Gazebo underwater simulation with hydrodynamics, buoyancy, IMU, odometry bridging, thruster `thrust` interfaces, and servo `position` interfaces.
- Real-hardware backend based on MAVROS RC override for thrusters, with optional serial servo control for Subcat.
- Multi-robot namespacing through unique `robot_namespace`, `robot_name`, TF prefixes, and dynamic RViz displays.
- Trajectory sources from `target.info` files, B-spline topic input, and manual `cmd_vel` commands.
- Keyboard teleoperation with automatic discovery of `/<robot>/control_input` and `/<robot>/cmd_vel` topics.

## Supported Robots

| Robot | Package | Simulation | Real hardware | Control outputs |
| --- | --- | --- | --- | --- |
| BlueROV2 Heavy | `bluerov2_heavy` | Gazebo world, SDF/URDF, RViz | MAVROS | 8 thrusters via `thrust` |
| Subcat | `subcat` | Gazebo world, URDF, RViz | MAVROS + serial servos | 4 thrusters via `thrust`, 4 servos via `position` |

## Repository Layout

```text
underwater_ros2_control/
|-- command/keyboard_input/              # Keyboard command node and launch file
|-- controllers/acados_nmpc_controller/  # NMPC ros2_control controller and solver loader
|-- descriptions/bluerov/bluerov2_heavy/ # BlueROV2 Heavy model, worlds, config, launch files
|-- descriptions/subcat/                 # Subcat model, worlds, config, launch files
|-- hardwares/gz_underwater_hardware/    # Gazebo ros2_control underwater hardware interface
|-- hardwares/real_underwater_hardware/  # MAVROS and serial real-hardware interface
|-- LAUNCH_COMMANDS.md                   # Additional launch examples
`-- LICENSE
```

## Requirements

Recommended environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / `ros_gz`
- `colcon`, `rosdep`, and `ament_cmake`
- acados, with `ACADOS_INSTALL_DIR` set

Some tools have been used with Ubuntu 22.04 and ROS 2 Humble, but the simulation packages depend on `gz_sim_vendor`, `gz_plugin_vendor`, and `ros_gz_*`. Humble environments may need Gazebo-specific dependency adjustments.

Main ROS dependencies include:

- `controller_manager`
- `hardware_interface`
- `controller_interface`
- `joint_state_broadcaster`
- `imu_sensor_broadcaster`
- `robot_state_publisher`
- `joint_state_publisher`
- `xacro`
- `rviz2`
- `ros_gz_sim`
- `ros_gz_bridge`
- `mavros_msgs`

## Quick Start

### 1. Create a Workspace

```bash
mkdir -p ~/underwater_ws/src
cd ~/underwater_ws/src
git clone https://github.com/JunWeiaa/underwater_ros2_control.git underwater_ros2_control
cd ~/underwater_ws
```

If you already have a ROS 2 workspace, place this repository at `src/underwater_ros2_control`.

### 2. Install ROS Dependencies

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If you only need simulation and do not have MAVROS installed, you can skip `real_underwater_hardware` during builds. Building every package requires the MAVROS message dependencies.

### 3. Configure acados

`acados_nmpc_controller` reads `ACADOS_INSTALL_DIR` during CMake configuration and links against:

- `libacados.so`
- `libhpipm.so`
- `libblasfeo.so`
- `libqpOASES_e.so`

Example:

```bash
export ACADOS_INSTALL_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_INSTALL_DIR/lib:$LD_LIBRARY_PATH
```

Add these exports to your shell profile or workspace setup script if you build and run this project often.

### 4. Check Generated Solvers

The controller reads solver metadata and generated acados C code from:

```text
controllers/acados_nmpc_controller/solvers/bluerov2_heavy/
controllers/acados_nmpc_controller/solvers/subcat/
```

In this repository, each `c_generated_code` directory is a symlink to the generated code stored under the matching description package. Check the links before building:

```bash
ls -l src/underwater_ros2_control/controllers/acados_nmpc_controller/solvers/*/c_generated_code
```

If the links are missing, recreate them:

```bash
cd ~/underwater_ws/src/underwater_ros2_control
ln -sfn ../../../../descriptions/bluerov/bluerov2_heavy/matlab/c_generated_code \
  controllers/acados_nmpc_controller/solvers/bluerov2_heavy/c_generated_code
ln -sfn ../../../../descriptions/subcat/matlab/c_generated_code \
  controllers/acados_nmpc_controller/solvers/subcat/c_generated_code
```

### 5. Build

Build the main packages:

```bash
cd ~/underwater_ws
colcon build --symlink-install --packages-select \
  acados_nmpc_controller \
  gz_underwater_hardware \
  real_underwater_hardware \
  bluerov2_heavy \
  subcat \
  keyboard_input
source install/setup.bash
```

Simulation-only build:

```bash
cd ~/underwater_ws
colcon build --symlink-install --packages-select \
  acados_nmpc_controller \
  gz_underwater_hardware \
  bluerov2_heavy \
  subcat \
  keyboard_input
source install/setup.bash
```

## Simulation

All commands assume:

```bash
cd ~/underwater_ws
source install/setup.bash
```

### Single BlueROV2 Heavy

```bash
ros2 launch bluerov2_heavy gz.launch.py \
  robot_name:=bluerov2_heavy \
  robot_namespace:=rov1 \
  x:=0.0 \
  y:=0.0 \
  height:=-1.0 \
  rviz_config:=single \
  merge_tf:=true
```

### Single Subcat

```bash
ros2 launch subcat gz.launch.py \
  robot_name:=subcat \
  robot_namespace:=sub1 \
  x:=0.0 \
  y:=0.0 \
  height:=-3.0 \
  rviz_config:=single \
  merge_tf:=true
```

### Multiple Robots

The first robot starts Gazebo, RViz, and `/clock`. Later launches reuse existing processes when `start_gz:=auto`, `start_rviz:=auto`, and `bridge_clock:=auto` are left at their defaults.

```bash
ros2 launch bluerov2_heavy gz.launch.py \
  robot_name:=bluerov2_heavy_1 \
  robot_namespace:=rov1 \
  x:=0.0 y:=0.0 height:=-1.0 \
  rviz_config:=multi \
  rviz_robots:=rov1,sub1 \
  merge_tf:=true
```

```bash
ros2 launch subcat gz.launch.py \
  robot_name:=subcat_1 \
  robot_namespace:=sub1 \
  x:=3.0 y:=0.0 height:=-3.0 \
  rviz_config:=multi \
  rviz_robots:=rov1,sub1 \
  merge_tf:=true
```

See [LAUNCH_COMMANDS.md](LAUNCH_COMMANDS.md) for more launch combinations.

### Common Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `robot_name` | `bluerov2_heavy` or `subcat` | Gazebo model name. Must be unique in one world. |
| `robot_namespace` | package default | ROS namespace, for example `rov1` or `sub1`. |
| `world` | package default world | Loads `worlds/<name>.world`. |
| `x` / `y` / `height` | launch-file default | Initial simulation pose. |
| `start_gz` | `auto` | Start Gazebo. `auto` checks for `/clock`. |
| `start_rviz` | `auto` | Start RViz. `auto` checks for an existing RViz node. |
| `bridge_clock` | `auto` | Bridge `/clock`. |
| `rviz_config` | `single` | `auto`, `single`, `multi`, a config filename, or an absolute path. |
| `rviz_robots` | empty | Comma-separated robot namespaces shown in multi-robot RViz. |
| `merge_tf` | `true` | Merge robot TF trees into the global `/tf`. |
| `tf_prefix` | empty | TF prefix. Defaults to `robot_namespace` when `merge_tf=true`. |
| `global_fixed_frame` | `map` | Global fixed frame for multi-robot RViz. |

## Keyboard Control

Start the keyboard node:

```bash
ros2 launch keyboard_input keyboard.launch.py
```

Force a robot list:

```bash
ros2 launch keyboard_input keyboard.launch.py robots:=rov1,sub1 active_robot:=rov1
```

Keys:

| Key | Action |
| --- | --- |
| `1` | NO_OUTPUT / stop output |
| `2` | AUTO / automatic trajectory tracking |
| `3` | MANUAL / manual velocity control |
| `W` / `S` | Forward / backward |
| `A` / `D` | Left / right sway |
| `Space` | Upward command |
| `C` or `X` | Downward command |
| `J` / `L` | Yaw left / right |
| `Q` / `E` | Yaw left / right |
| `,` / `.` | Previous / next robot |
| `[` / `]` | Previous / next robot |

The keyboard node publishes:

- `/<robot_namespace>/control_input`, type `std_msgs/msg/Int8`
- `/<robot_namespace>/cmd_vel`, type `geometry_msgs/msg/Twist`

Default parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `robots` | `auto` | Comma-separated robot namespaces, or automatic discovery. |
| `wait_timeout` | `5.0` | Seconds to wait for automatic discovery. |
| `active_robot` | empty | Initially selected robot. |
| `linear_speed` | `0.6` | Horizontal linear velocity command. |
| `vertical_speed` | `0.6` | Vertical velocity command. |
| `yaw_rate` | `0.5` | Yaw-rate command. |
| `repeat_rate` | `50.0` | Command refresh rate. |
| `command_timeout` | `0.12` | Delay before zeroing commands after key release. |

## Real Hardware

Real-hardware mode starts `controller_manager/ros2_control_node`, loads `real_underwater_hardware/RealSystem`, and sends actual actuator commands through MAVROS and, for Subcat, serial servos.

### Safety Checklist

- Remove propellers during bench tests, or physically isolate thrusters.
- Provide an independent emergency stop, manual RC takeover, and power disconnect.
- Keep the controller in `NO_OUTPUT` for first checks, then verify `/control_input`, MAVROS namespace, RC channel mapping, and servo directions.
- For Subcat, verify each `servo*_port`, `servo_command_map`, `servo_offsets`, and `servo_scales` entry.
- Start with short, low-power tests before increasing command magnitude or control frequency.

### BlueROV2 Heavy Real Hardware

```bash
ros2 launch bluerov2_heavy real.launch.py \
  robot_name:=bluerov2_heavy \
  robot_namespace:=rov1 \
  mavros_namespace:=/mavros \
  rviz_config:=single \
  merge_tf:=true
```

### Subcat Real Hardware

```bash
ros2 launch subcat real.launch.py \
  robot_name:=subcat \
  robot_namespace:=sub1 \
  mavros_namespace:=/mavros \
  servo0_port:=/dev/ttyACM1 \
  servo1_port:=/dev/ttyACM2 \
  servo2_port:=/dev/ttyACM3 \
  servo3_port:=/dev/ttyACM4 \
  rviz_config:=single \
  merge_tf:=true
```

Real-hardware parameters are mainly defined in:

- `descriptions/bluerov/bluerov2_heavy/urdf/real_plugins.xacro`
- `descriptions/subcat/urdf/real_plugins.xacro`
- `descriptions/bluerov/bluerov2_heavy/config/real.yaml`
- `descriptions/subcat/config/real.yaml`

## Interfaces and Topics

The common topics below are relative to the robot namespace, for example `/rov1/...` or `/sub1/...`.

| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `control_input` | `std_msgs/msg/Int8` | Input | Mode command: `1` stop, `2` auto, `3` manual. |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Input | Velocity command in MANUAL mode. |
| `odometry` | `nav_msgs/msg/Odometry` | Input | Gazebo bridge or external localization input used by the default estimator. |
| `trajectory` | `nav_msgs/msg/Path` | Output | Full target trajectory visualization. |
| `curr_traj` | `nav_msgs/msg/Path` | Output | Current NMPC horizon visualization. |
| `target_bspline` | `std_msgs/msg/Float64MultiArray` | Input | B-spline control points. |
| `observation` | `std_msgs/msg/Float64MultiArray` | Output | Optional debug state, enabled with `debug.publish_observation`. |

Hardware interfaces:

- BlueROV2 Heavy: `thruster1_joint` through `thruster8_joint`, command interface `thrust`.
- Subcat: `thruster1_joint` through `thruster4_joint`, command interface `thrust`; `servo1_joint` through `servo4_joint`, command interface `position`.
- IMU: `imu_sensor`, exposing orientation, angular velocity, and linear acceleration state interfaces.

## Configuration

### Controller Configuration

Main configuration files:

- `descriptions/bluerov/bluerov2_heavy/config/gazebo.yaml`
- `descriptions/bluerov/bluerov2_heavy/config/real.yaml`
- `descriptions/subcat/config/gazebo.yaml`
- `descriptions/subcat/config/real.yaml`

Key fields:

| Field | Description |
| --- | --- |
| `controller_manager.ros__parameters.update_rate` | `ros2_control` read/update/write rate. |
| `acados_nmpc_controller.ros__parameters.update_rate` | Controller output update rate. |
| `mpc.frequency` | acados solve frequency. |
| `robot_pkg` | Used to resolve the default `config/target.info`. |
| `solver_id` | Dynamic solver ID, such as `bluerov2_heavy` or `subcat`. |
| `state_dim` / `input_dim` | Solver state and input dimensions. |
| `solver.input_lower` / `solver.input_upper` | Input constraints. |
| `control_output_map` | Mapping from NMPC inputs to joint command interfaces. |
| `state_extra_map` | Subcat servo-state extension into the NMPC state vector. |
| `trajectory.*` | Trajectory source, sampling period, prediction horizon, and visualization parameters. |

### Trajectory Targets

Default target files:

- `descriptions/bluerov/bluerov2_heavy/config/target.info`
- `descriptions/subcat/config/target.info`

Supported target types:

- `circle`
- `figure8` or `figure_8`
- `points` or `trajectory`

`target.info` files use ENU coordinates by default: `x` east, `y` north, `z` up. `TargetManager` converts targets to the internal NED/FRD convention before passing them to the controller. Published RViz paths on `trajectory` and `curr_traj` are converted back for visualization.

### B-spline Topic Input

When `trajectory.enable_topic_targets=true`, the controller listens on `target_bspline`. Messages can use either format:

- Set `layout.dim[0].size = point_count`, `layout.dim[1].size = point_dim`, and flatten control points row-wise in `data`.
- Or put `degree`, `duration`, and `point_dim` at the start of `data`, followed by the flattened control points.

Each control point needs at least 3 position dimensions. Extra dimensions are treated as a prefix of the controller state vector.

### acados Solvers

Solver metadata lives at:

```text
controllers/acados_nmpc_controller/solvers/<solver_id>/solver.yaml
```

Fields:

| Field | Description |
| --- | --- |
| `solver_id` | Solver ID used by ROS parameters. |
| `model_name` | Model name in generated acados code. |
| `macro_prefix` | Macro prefix used by the shim. |
| `has_p_global` | Whether the solver has global-parameter precompute support. |

At runtime, the controller loads:

```text
install/lib/acados_nmpc_controller/solvers/lib<solver_id>_solver.so
```

You can override this path with the controller parameter `solver_library`.

## Debugging

List command and odometry topics:

```bash
ros2 topic list | grep -E '/(rov[0-9]+|sub[0-9]+|bluerov2_heavy|subcat)/((control_input)|(cmd_vel)|(odometry))'
```

Check active controllers:

```bash
ros2 control list_controllers -c /rov1/controller_manager
```

Watch commands and odometry:

```bash
ros2 topic echo /rov1/control_input
ros2 topic echo /rov1/cmd_vel
ros2 topic echo /rov1/odometry
```

Watch trajectory visualization topics:

```bash
ros2 topic echo /rov1/trajectory
ros2 topic echo /rov1/curr_traj
```

## FAQ

### CMake reports `ACADOS_INSTALL_DIR is not set`

Install acados and export the environment variables before building:

```bash
export ACADOS_INSTALL_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_INSTALL_DIR/lib:$LD_LIBRARY_PATH
```

### CMake cannot find `acados_solver_<model>.h`

Check that `controllers/acados_nmpc_controller/solvers/<solver_id>/c_generated_code` exists and contains `acados_solver_BlueROV_Heavy.h` or `acados_solver_subcat.h`.

### The controller keeps waiting for odometry

Check the Gazebo bridge and namespace:

```bash
ros2 topic list | grep odometry
ros2 topic echo /rov1/odometry
```

### Multi-robot TF appears mixed in RViz

Use `merge_tf:=true` and give every robot a unique `robot_namespace`. When `merge_tf=true`, the default `tf_prefix` is the robot namespace.

### Keyboard input does not respond

`keyboard_input` needs an interactive terminal. Try:

```bash
ros2 launch keyboard_input keyboard.launch.py use_tty:=true
```

## Development, Testing, and Contributing

### Development Guidelines

- Add new robots as description/config/launch packages and reuse `acados_nmpc_controller`, `gz_underwater_hardware`, and `real_underwater_hardware` when possible.
- Add new solvers under `controllers/acados_nmpc_controller/solvers/<solver_id>/` with a `solver.yaml` file and `c_generated_code`.
- When changing real-hardware output mapping, update the URDF/xacro, `real.yaml`, and the safety notes.
- Keep `colcon build`, `colcon test`, and launch smoke tests reproducible before submitting changes.

### Testing

Run package tests:

```bash
cd ~/underwater_ws
colcon test --packages-select \
  acados_nmpc_controller \
  gz_underwater_hardware \
  real_underwater_hardware \
  keyboard_input
colcon test-result --verbose
```

The explicit unit test currently lives in `real_underwater_hardware/test/test_load_real_system.cpp`. Other packages mostly rely on linting, compilation, and launch/smoke testing.

### Contributing

Issues and pull requests are welcome:

1. Describe the problem, reproduction steps, expected behavior, and runtime environment.
2. Keep changes focused. Avoid mixing formatting, refactoring, and functional changes in one PR.
3. Add usage notes, configuration examples, and tests for new features.
4. For real-hardware control, PWM mapping, coordinate frames, or solver constraints, describe the safety impact explicitly.

### Security and Safety Reports

If you find an issue that could cause unexpected thruster output, servo overtravel, emergency-stop failure, or real-vehicle loss of control, prefer a private maintainer contact or the repository's private security-reporting channel. If only a public issue is available, avoid posting complete dangerous reproduction steps.

## Roadmap

- Add CI and launch smoke tests.
- Add broader controller unit tests.
- Document the acados solver regeneration workflow.
- Normalize package descriptions, maintainers, and license fields across `package.xml` files.
- Add real-hardware calibration, RC channel mapping, and servo direction check tools.

## License and Acknowledgements

The root repository is licensed under the [Apache License 2.0](LICENSE). Some description packages currently declare BSD in their `package.xml` files. Check third-party models, meshes, MATLAB/acados generated code, and upstream dependencies for their own license terms.

This project was inspired in part by [legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control), which is licensed under the Apache License 2.0.

If you use this repository in a paper, course project, or engineering report, cite the repository name, commit hash, robot model, ROS 2/Gazebo/acados versions, and the simulation or real-hardware configuration you used.
