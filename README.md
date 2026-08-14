# underwater_ros2_control

Sim-to-real ROS 2 control framework for underwater robots powered by Gazebo, MAVROS, ros2_control, and acados NMPC.

## Packages

- `controllers/acados_nmpc_controller`: configurable acados NMPC controller with generated solver support.
- `hardwares/gz_underwater_hardware`: Gazebo ros2_control hardware interface for underwater simulation.
- `hardwares/real_underwater_hardware`: MAVROS-based real-hardware interface.
- `descriptions/bluerov/bluerov2_heavy`: BlueROV2 Heavy description, launch, config, and generated solver source.
- `descriptions/subcat`: Subcat description, launch, config, and generated solver source.
- `command/keyboard_input`: keyboard command node with single- and multi-robot target selection.

## Acknowledgements

This project was inspired in part by [legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control), which is licensed under the Apache License 2.0.
