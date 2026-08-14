#ifndef _REAL_SYSTEM_HPP_
#define _REAL_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "real_underwater_hardware/servo_controller.hpp"

#include <array>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

namespace real_underwater_hardware {

class ImuData {
public:
    double orientation_w = 1.0;
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
    rclcpp::Time timestamp;

    void clear() {
        orientation_w = 1.0;
        orientation_x = orientation_y = orientation_z = 0.0;
        angular_velocity_x = angular_velocity_y = angular_velocity_z = 0.0;
        linear_acceleration_x = linear_acceleration_y = linear_acceleration_z = 0.0;
        timestamp = rclcpp::Time(0);
    }
};

class RealSystem : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RealSystem)

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo &system_info) override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time,
                                         const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) override;

private:
    struct JointStateData {
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
    };

    struct CommandInfo {
        size_t joint_index = 0;
        std::string joint_name;
        std::string interface_name;
        size_t command_index = 0;
    };

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void configure_command_metadata();
    void configure_hardware_parameters();
    void configure_servo_parameters();
    void configure_thrust_channels();
    bool configure_mavros_backend();
    void configure_mavros_streams();

    void send_actuator_control_target(const rclcpp::Time &time);
    void send_rc_override(const std::array<uint16_t, 18> &channels);
    void send_servo_positions();

    std::array<uint16_t, 18> build_runtime_rc_override_channels() const;
    std::array<uint16_t, 18> build_neutral_rc_override_channels() const;
    double thrust_to_pwm(double x) const;
    double thrust_to_vel(double x) const;

    std::vector<double> hw_commands_;
    std::vector<JointStateData> joint_states_;
    std::vector<CommandInfo> command_infos_;
    std::vector<size_t> thrust_command_indices_;
    std::vector<size_t> position_command_indices_;

    ImuData imu_;

    std::string robot_name_;
    std::string mavros_namespace_;
    std::string thrust_pwm_model_;
    double thrust_pwm_per_newton_ = 20.0;

    rclcpp::Node::SharedPtr mavros_client_node_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_long_client_;
    std::unique_ptr<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>
        rt_rc_override_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

    std::vector<int> thrust_channels_;
    std::vector<double> thrust_channel_scales_;

    bool servo_enabled_ = false;
    int servo_count_ = 0;
    int servo_timeout_ms_ = 100;
    int servo_duration_ms_ = 10;
    std::unique_ptr<ServoController> servo_controller_;
    std::vector<std::string> servo_ports_;
    std::vector<int> servo_ids_;
    std::vector<int> servo_baudrates_;
    std::vector<int> servo_command_map_;
    std::vector<double> servo_offsets_;
    std::vector<double> servo_scales_;
};

} // namespace real_underwater_hardware

#endif
