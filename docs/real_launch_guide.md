# Real Hardware Guide

All commands below assume the workspace root is:

```bash
cd ~/underwater_ws
source install/setup.bash
```

After rebuilding packages, source again:

```bash
colcon build --symlink-install --packages-select \
  acados_nmpc_controller \
  real_underwater_hardware \
  bluerov2_heavy \
  subcat \
  keyboard_input
source install/setup.bash
```

GCC builds use `libgomp` from the GCC toolchain. If building with Clang, install
OpenMP first:

```bash
sudo apt install -y libomp-dev
```

## Localization Requirement

The real launch files start `ros2_control`, the real hardware interface, robot
state publishing, controllers, and RViz. They do not start MAVROS, cameras, DVL,
or SLAM/localization nodes.

Before enabling physical commands, provide these real-time inputs:

- `/mavros/imu/data`, type `sensor_msgs/msg/Imu`, from MAVROS.
- `/<robot_namespace>/odometry`, type `nav_msgs/msg/Odometry`, for the NMPC
  state estimator.
- TF frames consistent with `global_fixed_frame`, `tf_prefix`, and the odometry
  frame used by the launch command.

In our pool experiments, localization was provided by an AprilTag EKF pipeline
using a downward-facing camera and a known tag map. See
[tag_ekf_localization](https://github.com/JunWeiaa/tag_ekf_localization) for the
reference implementation. That pipeline can feed MAVROS vision pose, and the
resulting fused odometry should be relayed or remapped to
`/<robot_namespace>/odometry`.

You can also use a DVL-aided estimator, acoustic localization, visual SLAM, or
another underwater SLAM pipeline. The only interface requirement for this
framework is that the final estimate is available as namespaced
`nav_msgs/msg/Odometry`.

## MAVROS RC Override

The real hardware interface sends thruster commands through MAVROS
`mavros_msgs/msg/OverrideRCIn` on:

```text
/mavros/rc/override
```

ArduPilot checks the MAVLink source system ID before accepting RC override. If
the ROS side publishes `/mavros/rc/override` but the vehicle does not respond,
check this permission path before debugging the controller.

The relevant upstream ArduPilot paths are `ArduSub/GCS_MAVLink_Sub.cpp`,
`libraries/GCS_MAVLink/GCS_Common.cpp`, and
`libraries/GCS_MAVLink/GCS.cpp`.

### Output Function Mapping

This framework already computes the per-thruster commands and sends them as RC
override channels. In the GCS, configure the physical thruster outputs as
`RCIN`/`RCPassThru` functions instead of ArduPilot motor mixer functions such as
`Motor1`, `Motor2`, and so on.

In Mission Planner or QGroundControl, open the full parameter list and set the
corresponding `SERVOn_FUNCTION` values. ArduPilot parameter metadata labels
these functions as `RCIN1` to `RCIN16`; some GCS views or older docs describe the
same idea as RC pass-through or `RCPassThru`. The numeric value is:

```text
SERVOn_FUNCTION = 50 + RC input channel
```

For the default BlueROV2 Heavy mapping in `real_plugins.xacro`:

```text
SERVO1_FUNCTION = 51  # RCIN1 / RCPassThru1
SERVO2_FUNCTION = 52  # RCIN2 / RCPassThru2
SERVO3_FUNCTION = 53  # RCIN3 / RCPassThru3
SERVO4_FUNCTION = 54  # RCIN4 / RCPassThru4
SERVO5_FUNCTION = 55  # RCIN5 / RCPassThru5
SERVO6_FUNCTION = 56  # RCIN6 / RCPassThru6
SERVO7_FUNCTION = 57  # RCIN7 / RCPassThru7
SERVO8_FUNCTION = 58  # RCIN8 / RCPassThru8
```

For the default SubCat mapping:

```text
SERVO1_FUNCTION = 51  # RCIN1 / RCPassThru1
SERVO2_FUNCTION = 52  # RCIN2 / RCPassThru2
SERVO3_FUNCTION = 53  # RCIN3 / RCPassThru3
SERVO4_FUNCTION = 54  # RCIN4 / RCPassThru4
```

If `thrust_channels` is changed in the robot xacro, update the matching
`SERVOn_FUNCTION` parameters in ArduPilot as well. Keep propellers removed while
checking `/mavros/rc/override` and the GCS actuator output monitor.

### Option A: Keep ArduPilot Unchanged

Configure MAVROS to use a MAVLink source system ID accepted by ArduPilot as the
GCS. ArduPilot uses `MAV_GCS_SYSID` and `MAV_GCS_SYSID_HI` for this check. The
default accepted GCS system ID is usually `255`.

Example MAVROS ROS 2 parameters:

```yaml
mavros_node:
  ros__parameters:
    fcu_url: /dev/ttyACM0:921600
    system_id: 255
    component_id: 190
    target_system_id: 1
    target_component_id: 1
```

Set the ArduPilot parameters to match:

```text
MAV_GCS_SYSID    = 255
MAV_GCS_SYSID_HI = 0
```

If `MAV_GCS_SYSID_HI` is greater than or equal to `MAV_GCS_SYSID`, ArduPilot
treats the inclusive range as valid GCS system IDs. Also check the MAVROS
connection URL: `?ids=<sysid>,<compid>` in `fcu_url` overrides the
`system_id` and `component_id` parameters.

### Option B: Lab Firmware Patch

For controlled lab firmware, the GCS-only guard can be removed or commented out
in ArduPilot. This is what we used for convenience in our pool experiments.

Find the relevant checks in the ArduPilot checkout:

```bash
cd ~/ardupilot
rg -n -C 6 "MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE|handle_rc_channels_override|sysid_is_gcs" \
  ArduSub libraries/GCS_MAVLink
```

In current ArduSub, the first check is in `ArduSub/GCS_MAVLink_Sub.cpp`.

Original:

```cpp
case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {     // MAV ID: 70
    if (!gcs().sysid_is_gcs(msg.sysid)) {
        break;    // Only accept control from our gcs
    }

    sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
    handle_rc_channels_override(msg);
    break;
}
```

Lab patch:

```cpp
case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {     // MAV ID: 70
    // Lab firmware: allow RC override from the companion/MAVROS link.
    // if (!gcs().sysid_is_gcs(msg.sysid)) {
    //     break;    // Only accept control from our gcs
    // }

    sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
    handle_rc_channels_override(msg);
    break;
}
```

The common MAVLink handler has another guard in
`libraries/GCS_MAVLink/GCS_Common.cpp`. If this check remains, the message still
returns before setting the override.

Original:

```cpp
void GCS_MAVLINK::handle_rc_channels_override(const mavlink_message_t &msg)
{
    if (!gcs().sysid_is_gcs(msg.sysid)) {
        return; // Only accept control from our gcs
    }

    const uint32_t tnow = AP_HAL::millis();
    ...
}
```

Lab patch:

```cpp
void GCS_MAVLINK::handle_rc_channels_override(const mavlink_message_t &msg)
{
    // Lab firmware: allow RC override from the companion/MAVROS link.
    // if (!gcs().sysid_is_gcs(msg.sysid)) {
    //     return; // Only accept control from our gcs
    // }

    const uint32_t tnow = AP_HAL::millis();
    ...
}
```

Then rebuild and flash the custom ArduSub firmware:

```bash
./waf configure --board <board>
./waf sub
```

Record which option is used for each experiment, because it changes the authority
boundary for physical actuator commands. Do not mix this lab firmware with
uncontrolled field tests unless the vehicle-level safety procedures have been
reviewed again.

## Bring-Up Order

1. Start the vehicle autopilot connection and MAVROS.
2. Start the localization pipeline for the current test environment.
3. Verify IMU and odometry topics before launching the hardware interface.
4. Launch the real robot description and controllers.
5. Start keyboard control or another command source, then switch out of stop mode
   only after the state estimate is stable.

Useful checks:

```bash
ros2 topic hz /mavros/imu/data
ros2 topic echo /<robot_namespace>/odometry --once
ros2 control list_controllers -c /<robot_namespace>/controller_manager
```

Replace `<robot_namespace>` with `rov1`, `sub1`, or your chosen namespace.

## Real BlueROV2 Heavy

MAVROS RC override mode:

```bash
ros2 launch bluerov2_heavy real.launch.py \
  robot_name:=bluerov2_heavy \
  robot_namespace:=rov1 \
  mavros_namespace:=/mavros \
  rviz_config:=single \
  merge_tf:=true
```

The controller expects odometry on:

```text
/rov1/odometry
```

## Real SubCat

MAVROS RC override plus serial servos:

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

The controller expects odometry on:

```text
/sub1/odometry
```

## Keyboard Control

Start the keyboard command node after the robot controllers are active:

```bash
ros2 launch keyboard_input keyboard.launch.py \
  robots:=rov1
```

For SubCat:

```bash
ros2 launch keyboard_input keyboard.launch.py \
  robots:=sub1
```

Mode keys are `1` for stop, `2` for auto, and `3` for manual. Keep the selected
robot in stop mode while checking localization, MAVROS, and actuator direction.

## Multiple Real Robots

For multiple real robots, give each robot a unique `robot_namespace`,
`robot_name`, TF prefix, localization output topic, and MAVROS/hardware
connection. For example, the odometry topics should be separate:

```text
/rov1/odometry
/rov2/odometry
/sub1/odometry
```
