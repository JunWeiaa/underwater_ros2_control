# underwater_ros2_control

ROS 2 control framework for underwater robots. The repository combines Gazebo
simulation, `ros2_control` hardware interfaces, acados NMPC control, keyboard
teleoperation, and robot descriptions for BlueROV2 Heavy and SubCat. The framework
can also be extended to other underwater robots by adding new robot
descriptions, controller parameters, and generated acados solvers.

For readers interested in reproducing the SubCat hardware, please refer to the
[subcat-hw](https://github.com/JunWeiaa/subcat-hw) repository.

## Real-World Experiments

<p align="center">
  <img src="docs/images/underwater_ros2_control_demo_1.gif" alt="Underwater robot close view" width="32%">
  <img src="docs/images/underwater_ros2_control_demo_2.gif" alt="6DOF trajectory tracking" width="32%">
  <img src="docs/images/underwater_ros2_control_demo_3.gif" alt="Pool test overview" width="32%">
</p>

## Packages

```text
underwater_ros2_control/
|-- command/keyboard_input/              # Keyboard command node
|-- controllers/acados_nmpc_controller/  # NMPC ros2_control controller
|-- descriptions/bluerov/bluerov2_heavy/ # BlueROV2 Heavy model and launch files
|-- descriptions/subcat/                 # SubCat model and launch files
|-- hardwares/gz_underwater_hardware/    # Gazebo underwater hardware interface
|-- hardwares/real_underwater_hardware/  # MAVROS and serial hardware interface
|-- docs/launch_guide.md                 # Launch examples and quick checks
|-- docs/matlab_models.md                # MATLAB/acados model workflow
`-- LICENSE
```

## Requirements

Recommended environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / `ros_gz`
- `colcon`, `rosdep`, and `ament_cmake`
- acados v0.4.5, with `ACADOS_INSTALL_DIR` set

Main runtime packages include `controller_manager`, `hardware_interface`,
`joint_state_broadcaster`, `imu_sensor_broadcaster`, `robot_state_publisher`,
`xacro`, `rviz2`, `ros_gz_sim`, `ros_gz_bridge`, and `mavros_msgs`.

## Build

```bash
cd ~/underwater_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Install acados v0.4.5 before building the controller:

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3 python3-pip

git clone https://github.com/acados/acados.git $HOME/acados
cd $HOME/acados
git checkout v0.4.5
git submodule update --init --recursive

mkdir -p build
cd build
cmake -DACADOS_INSTALL_DIR=$HOME/acados ..
cmake --build . --target install -j$(nproc)
```

Export the acados environment before building or running the controller:

```bash
export ACADOS_INSTALL_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_INSTALL_DIR/lib:$LD_LIBRARY_PATH
```

For MATLAB/Octave setup, follow the official acados guide:
[MATLAB + Simulink and Octave Interface](https://docs.acados.org/matlab_octave_interface/index.html).
Keep this repository on the `v0.4.5` acados checkout when generating its solvers.
CasADi 3.7 has also been tested successfully with these MATLAB models.

Build the main workspace packages:

```bash
colcon build --symlink-install --packages-select \
  acados_nmpc_controller \
  gz_underwater_hardware \
  real_underwater_hardware \
  bluerov2_heavy \
  subcat \
  keyboard_input
source install/setup.bash
```

Simulation-only builds can skip `real_underwater_hardware` if MAVROS message
dependencies are not installed.

## Run

Single SubCat simulation:

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

Single BlueROV2 Heavy simulation:

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

More simulation, multi-robot, real-hardware, keyboard, and debug commands are in
[docs/launch_guide.md](docs/launch_guide.md).

## Keyboard Control

```bash
ros2 launch keyboard_input keyboard.launch.py
```

The keyboard node publishes:

- `/<robot_namespace>/control_input`, type `std_msgs/msg/Int8`
- `/<robot_namespace>/cmd_vel`, type `geometry_msgs/msg/Twist`

Mode keys are `1` for stop, `2` for auto, and `3` for manual. Movement keys are
listed in the launch guide.

## Configuration

Robot launch and controller parameters are mainly stored under:

- `descriptions/bluerov/bluerov2_heavy/config/`
- `descriptions/subcat/config/`
- `descriptions/bluerov/bluerov2_heavy/urdf/`
- `descriptions/subcat/urdf/`

acados solver metadata lives under:

```text
controllers/acados_nmpc_controller/solvers/<solver_id>/solver.yaml
```

Generated solver code is expected through each solver's `c_generated_code`
directory. Check those links if CMake cannot find generated acados headers.
See [docs/matlab_models.md](docs/matlab_models.md) before changing or
regenerating robot MATLAB/acados models.

## Development

- Add new robots as description/config/launch packages.
- Reuse `acados_nmpc_controller`, `gz_underwater_hardware`, and
  `real_underwater_hardware` where possible.
- Keep robot namespaces and Gazebo model names unique in multi-robot sessions.
- Rebuild and source the workspace after changing URDF, xacro, controller
  plugins, or generated solver code.

## License and Acknowledgements

The root repository is licensed under the [Apache License 2.0](LICENSE). Some
description packages currently declare BSD in their `package.xml` files. Check
third-party models, meshes, MATLAB/acados generated code, and upstream
dependencies for their own license terms.

This project was inspired by
[legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control)
and adapted for underwater robot simulation, control, and hardware integration.

## Citation

If this project or the SubCat robot model helps your research, a citation would
be greatly appreciated:

```bibtex
@ARTICLE{11661758,
  author={Wei, Jun and Miao, Zhiqiang and Chen, Yizong and Liu, Xinjiang and Wang, Yaonan},
  journal={IEEE Robotics and Automation Letters},
  title={SubCat: Design and Control of a Vectored-Thrust Underwater Robot with Model Predictive Control},
  year={2026},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2026.3726326}
}
```
