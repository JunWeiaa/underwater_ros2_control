#ifndef _ACADOS_NMPC_CONTROLLER_HPP
#define _ACADOS_NMPC_CONTROLLER_HPP

#include "acados_nmpc_controller/FSM/StateAuto.hpp"
#include "acados_nmpc_controller/FSM/StateFSM.hpp"
#include "acados_nmpc_controller/FSM/StateNoOutput.hpp"
#include "acados_nmpc_controller/FSM/StateManual.hpp"
#include "controller_interface/controller_interface.hpp"
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <unordered_map>
#include "acados_nmpc_controller/control/CtrlComponent.h"
#include "std_msgs/msg/int8.hpp"
namespace acados_nmpc_controller {

struct FSMStateList {
    std::shared_ptr<FSMState> invalid;
    std::shared_ptr<StateNoOutput> no_output;
    std::shared_ptr<StateAuto> auto_;
    std::shared_ptr<StateManual> manual;
};
struct JointConfig {
    std::string name;
    std::vector<std::string> command_interfaces;
    std::vector<std::string> state_interfaces;
};
class Acados_NMPC_Controller
    : public controller_interface::ControllerInterface {
public:
    Acados_NMPC_Controller() = default;
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    controller_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_error(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::return_type
    update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
    std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;
    FSMMode mode_ = FSMMode::NORMAL;
    FSMStateName next_state_name_ = FSMStateName::INVALID;
    FSMStateList state_list_;
    std::shared_ptr<FSMState> current_state_;
    std::shared_ptr<FSMState> next_state_;

    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;

    std::string imu_name_;
    std::string base_name_;
    std::string command_prefix_;
    std::string odom_name_;
    std::vector<std::string> imu_interface_types_;
    std::vector<std::string> odom_interface_types_;
    std::string estimator_type_{""};
    std::vector<JointConfig> joint_configs_;

    // inteface
    ControllerInterfaces controller_interfaces_;
    std::shared_ptr<CtrlComponent> ctrl_comp_;
    std::unordered_map<std::string,
                       std::vector<std::reference_wrapper<
                           hardware_interface::LoanedCommandInterface>> *>
        command_interface_map_ = {
            {"effort",
             &controller_interfaces_.joint_torque_command_interface_},
            {"position",
             &controller_interfaces_.joint_position_command_interface_},
            {"velocity",
             &controller_interfaces_.joint_velocity_command_interface_},
            {"thrust", &controller_interfaces_.joint_thrust_command_interface_}
        };
    std::unordered_map<std::string,
                       std::vector<std::reference_wrapper<
                           hardware_interface::LoanedStateInterface>> *>
        state_interface_map_ = {
            {"position", &controller_interfaces_.joint_position_state_interface_},
            {"effort", &controller_interfaces_.joint_effort_state_interface_},
            {"velocity",
             &controller_interfaces_.joint_velocity_state_interface_},
            {"thrust",
             &controller_interfaces_.joint_thrust_state_interface_},
        };
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr control_input_subscription_;
};

} // namespace acados_nmpc_controller

#endif
