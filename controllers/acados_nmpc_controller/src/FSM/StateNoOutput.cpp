#include "acados_nmpc_controller/FSM/StateNoOutput.hpp"
#include <rclcpp/logging.hpp>
#include <tuple>

namespace {
void clearCommands(ControllerInterfaces &controller_interfaces) {
    for (auto i : controller_interfaces.joint_thrust_command_interface_) {
        std::ignore = i.get().set_value(0.0);
    }
    for (auto i : controller_interfaces.joint_position_command_interface_) {
        std::ignore = i.get().set_value(0.0);
    }
    for (auto i : controller_interfaces.joint_velocity_command_interface_) {
        std::ignore = i.get().set_value(0.0);
    }
    for (auto i : controller_interfaces.joint_torque_command_interface_) {
        std::ignore = i.get().set_value(0.0);
    }
}
} // namespace

StateNoOutput::StateNoOutput(ControllerInterfaces &ctrl_interfaces) :
    FSMState(FSMStateName::NO_OUTPUT, "NoOutput", ctrl_interfaces) {
}

void StateNoOutput::enter() {
    clearCommands(controller_interfaces_);
}

void StateNoOutput::run(const rclcpp::Time & /*time*/,
                        const rclcpp::Duration & /*period*/) {
    clearCommands(controller_interfaces_);
    // double value = toggle ? 15.0 : 4.0;
    // std::ignore = controller_interfaces_.joint_thrust_command_interface_[4].get().set_value(value);
    // toggle = !toggle;

    // // std::ignore = controller_interfaces_.joint_thrust_command_interface_[4].get().set_value(4.0);
    // std::ignore = controller_interfaces_.joint_thrust_command_interface_[5].get().set_value(4.0);
    // std::ignore = controller_interfaces_.joint_thrust_command_interface_[6].get().set_value(4.0);
    // std::ignore = controller_interfaces_.joint_thrust_command_interface_[7].get().set_value(-4.0);
}

void StateNoOutput::exit() {
}

FSMStateName StateNoOutput::checkChange() {
    if (controller_interfaces_.ctrl_state_command_.data == 2) {
        return FSMStateName::AUTO;
    } else if (controller_interfaces_.ctrl_state_command_.data == 3) {
        return FSMStateName::MANUAL;
    }
    return FSMStateName::NO_OUTPUT;
}
