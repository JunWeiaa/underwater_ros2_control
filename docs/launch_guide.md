# Launch Guide

All commands below assume the workspace root is:

```bash
cd ~/underwater_ws
source install/setup.bash
```

After rebuilding packages, source again:

```bash
colcon build --symlink-install --packages-select acados_nmpc_controller gz_underwater_hardware real_underwater_hardware bluerov2_heavy subcat keyboard_input
source install/setup.bash
```

## Single BlueROV2 Heavy

Recommended namespace is `rov1`, so the command topics are `/rov1/control_input` and `/rov1/cmd_vel`.

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

## Single Subcat

Recommended namespace is `sub1`, so the command topics are `/sub1/control_input` and `/sub1/cmd_vel`.

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

## Two BlueROV2 Heavy Robots

Terminal 1:

```bash
ros2 launch bluerov2_heavy gz.launch.py \
  robot_name:=bluerov2_heavy_1 \
  robot_namespace:=rov1 \
  x:=0.0 \
  y:=0.0 \
  height:=-1.0 \
  rviz_config:=multi \
  rviz_robots:=rov1,rov2 \
  merge_tf:=true
```

Terminal 2:

```bash
ros2 launch bluerov2_heavy gz.launch.py \
  robot_name:=bluerov2_heavy_2 \
  robot_namespace:=rov2 \
  x:=3.0 \
  y:=0.0 \
  height:=-1.0 \
  rviz_config:=multi \
  rviz_robots:=rov1,rov2 \
  merge_tf:=true
```

`start_gz:=auto`, `start_rviz:=auto`, and `bridge_clock:=auto` are the defaults. The second launch should reuse the existing Gazebo, RViz, and `/clock`.

## Two Subcat Robots

Terminal 1:

```bash
ros2 launch subcat gz.launch.py \
  robot_name:=subcat_1 \
  robot_namespace:=sub1 \
  x:=0.0 \
  y:=0.0 \
  height:=-3.0 \
  rviz_config:=multi \
  rviz_robots:=sub1,sub2 \
  merge_tf:=true
```

Terminal 2:

```bash
ros2 launch subcat gz.launch.py \
  robot_name:=subcat_2 \
  robot_namespace:=sub2 \
  x:=3.0 \
  y:=0.0 \
  height:=-3.0 \
  rviz_config:=multi \
  rviz_robots:=sub1,sub2 \
  merge_tf:=true
```

The second Subcat launch also reuses existing Gazebo, RViz, and `/clock` by default.

## Mixed BlueROV2 Heavy And Subcat

Terminal 1:

```bash
ros2 launch bluerov2_heavy gz.launch.py \
  robot_name:=bluerov2_heavy_1 \
  robot_namespace:=rov1 \
  x:=0.0 \
  y:=0.0 \
  height:=-1.0 \
  rviz_config:=multi \
  rviz_robots:=rov1,sub1 \
  merge_tf:=true
```

Terminal 2:

```bash
ros2 launch subcat gz.launch.py \
  robot_name:=subcat_1 \
  robot_namespace:=sub1 \
  x:=3.0 \
  y:=0.0 \
  height:=-3.0 \
  rviz_config:=multi \
  rviz_robots:=rov1,sub1 \
  merge_tf:=true
```

You can reverse the order. The first launch starts Gazebo/RViz/`/clock`; the second launch reuses them by default.

## Real BlueROV2 Heavy

MAVROS mode:

```bash
ros2 launch bluerov2_heavy real.launch.py \
  robot_name:=bluerov2_heavy \
  robot_namespace:=rov1 \
  mavros_namespace:=/mavros \
  rviz_config:=single \
  merge_tf:=true
```

## Real Subcat

MAVROS plus serial servos:

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

For multiple real robots, give each robot a unique `robot_namespace`, `robot_name`, TF prefix, and hardware/MAVROS connection.

## Keyboard Control

Auto-detect current robots from `/<robot_namespace>/control_input` and `/<robot_namespace>/cmd_vel`:

```bash
ros2 launch keyboard_input keyboard.launch.py
```

Force the robot list:

```bash
ros2 launch keyboard_input keyboard.launch.py \
  robots:=rov1,rov2
```

For Subcat:

```bash
ros2 launch keyboard_input keyboard.launch.py \
  robots:=sub1,sub2
```

For one BlueROV2 Heavy and one Subcat:

```bash
ros2 launch keyboard_input keyboard.launch.py \
  robots:=rov1,sub1
```

Start with `rov2` selected:

```bash
ros2 launch keyboard_input keyboard.launch.py \
  robots:=rov1,rov2 \
  active_robot:=rov2
```

Wait longer if robot topics are still appearing:

```bash
ros2 launch keyboard_input keyboard.launch.py \
  wait_timeout:=15
```

Useful keys:

```text
1        NO_OUTPUT / stop
2        AUTO
3        MANUAL
W/S      surge forward/back
A/D      sway left/right
Space    up
C or X   down
J/L      yaw left/right
Q/E      yaw left/right
,/.      switch previous/next robot
[/]      switch previous/next robot
```

## Quick Checks

If a fresh single-robot launch unexpectedly prints `/clock already exists; skipping Gazebo launch`,
check for an old Gazebo process before launching again:

```bash
pgrep -af "gz sim"
```

Stop stale processes before starting a new single-robot session.

List command topics:

```bash
ros2 topic list | grep -E '/(rov[0-9]+|sub[0-9]+|bluerov2_heavy|subcat)/((control_input)|(cmd_vel))'
```

Check active controllers for one robot:

```bash
ros2 control list_controllers -c /rov1/controller_manager
```

Watch keyboard mode command:

```bash
ros2 topic echo /rov1/control_input
```

Watch manual velocity command:

```bash
ros2 topic echo /rov1/cmd_vel
```
