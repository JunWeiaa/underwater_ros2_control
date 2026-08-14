#ifndef _CONTROLLERINTERFACES_HPP
#define _CONTROLLERINTERFACES_HPP
#include "std_msgs/msg/int8.hpp"
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <string>
#include <unordered_map>
#include <vector>

class ControllerInterfaces {
public:
    ControllerInterfaces() = default;
    void clear() {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_thrust_command_interface_.clear();
        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
        joint_thrust_state_interface_.clear();
        imu_state_interface_.clear();
        odom_state_interface_.clear();
        command_interfaces_by_name_.clear();
        state_interfaces_by_name_.clear();
    }
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_thrust_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_thrust_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        imu_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        odom_state_interface_;
    std::unordered_map<std::string, hardware_interface::LoanedCommandInterface *>
        command_interfaces_by_name_;
    std::unordered_map<std::string, hardware_interface::LoanedStateInterface *>
        state_interfaces_by_name_;
    std_msgs::msg::Int8 ctrl_state_command_;
};

#endif
