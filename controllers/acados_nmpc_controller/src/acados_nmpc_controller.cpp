#include "acados_nmpc_controller/acados_nmpc_controller.hpp"
#include "acados_nmpc_controller/control/CtrlComponent.h"
#include "std_msgs/msg/int8.hpp"
#include <rclcpp/rclcpp.hpp>
namespace acados_nmpc_controller {
controller_interface::CallbackReturn Acados_NMPC_Controller::on_init() {
    try {
        std::vector<std::string> joint_names =
            get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
        RCLCPP_INFO(get_node()->get_logger(), "Found %zu joints", joint_names.size());

        command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        joint_configs_.clear();
        for (const auto &joint_name : joint_names) {
            JointConfig jc;
            jc.name = joint_name;
            std::string cmd_key = joint_name + ".command_interfaces";
            std::string state_key = joint_name + ".state_interfaces";
            jc.command_interfaces =
                get_node()->declare_parameter<std::vector<std::string>>(cmd_key, command_interface_types_);
            jc.state_interfaces =
                get_node()->declare_parameter<std::vector<std::string>>(state_key, state_interface_types_);
            joint_configs_.push_back(jc);
            RCLCPP_INFO(get_node()->get_logger(),
                        "Joint: %s, cmd_ifaces: %zu, state_ifaces: %zu",
                        jc.name.c_str(),
                        jc.command_interfaces.size(),
                        jc.state_interfaces.size());
        }
        // imu sensor
        imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
        base_name_ = auto_declare<std::string>("base_name", base_name_);
        imu_interface_types_ = auto_declare<std::vector<std::string>>(
            "imu_interfaces", state_interface_types_);
        command_prefix_ =
            auto_declare<std::string>("command_prefix", command_prefix_);

        // Odometer Sensor (Ground Truth)
        estimator_type_ = auto_declare<std::string>("estimator_type", estimator_type_);
        if (estimator_type_ == "ground_truth") {
            odom_name_ = auto_declare<std::string>("odom_name", odom_name_);
            odom_interface_types_ = auto_declare<std::vector<std::string>>(
                "odom_interfaces", state_interface_types_);
        }
        int update_rate = 0;
        get_node()->get_parameter("update_rate", update_rate);
        RCLCPP_INFO(get_node()->get_logger(),
                    "Controller update_rate parameter: %d Hz",
                    update_rate);
        ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), controller_interfaces_);
        ctrl_comp_->setupStateEstimate(estimator_type_);

        state_list_.no_output =
            std::make_shared<StateNoOutput>(controller_interfaces_);
        state_list_.auto_ =
            std::make_shared<StateAuto>(controller_interfaces_, ctrl_comp_);
        state_list_.manual =
            std::make_shared<StateManual>(controller_interfaces_, ctrl_comp_);
    } catch (const std::exception &e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration
Acados_NMPC_Controller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf = {
        controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (const auto &jc : joint_configs_) {
        for (const auto &iface : jc.command_interfaces) {
            conf.names.push_back(jc.name + "/" + iface);
        }
    }
    return conf;
}
controller_interface::InterfaceConfiguration
Acados_NMPC_Controller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf = {
        controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    size_t state_interface_count = 0;
    for (const auto &jc : joint_configs_) {
        state_interface_count += jc.state_interfaces.size();
    }
    conf.names.reserve(state_interface_count + imu_interface_types_.size() + odom_interface_types_.size());
    for (const auto &jc : joint_configs_) {
        for (const auto &iface : jc.state_interfaces) {
            conf.names.push_back(jc.name + "/" + iface);
        }
    }

    for (const auto &interface_type : imu_interface_types_) {
        conf.names.push_back(imu_name_ + "/" + interface_type);
    }
    for (const auto &interface_type : odom_interface_types_) {
        conf.names.push_back(odom_name_ + "/" + interface_type);
    }

    return conf;
}

controller_interface::CallbackReturn
Acados_NMPC_Controller::on_configure(const rclcpp_lifecycle::State &) {
    control_input_subscription_ = get_node()->create_subscription<std_msgs::msg::Int8>(
        "control_input", 10, [this](const std_msgs::msg::Int8::SharedPtr msg) {
            // Handle message
            controller_interfaces_.ctrl_state_command_.set__data(msg->data);
        });

    return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn
Acados_NMPC_Controller::on_activate(const rclcpp_lifecycle::State &) {
    controller_interfaces_.clear();
    // assign command interfaces
    for (auto &interface : command_interfaces_) {
        const std::string interface_name = interface.get_interface_name();
        controller_interfaces_.command_interfaces_by_name_[interface.get_name()] = &interface;
        const auto group_it = command_interface_map_.find(interface_name);
        if (group_it != command_interface_map_.end()) {
            group_it->second->push_back(interface);
        } else {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Ignoring unsupported command interface '%s'",
                        interface.get_name().c_str());
        }
    }

    // assign state interfaces
    for (auto &interface : state_interfaces_) {
        controller_interfaces_.state_interfaces_by_name_[interface.get_name()] = &interface;
        if (interface.get_prefix_name() == imu_name_) {
            controller_interfaces_.imu_state_interface_.emplace_back(interface);
        } else if (interface.get_prefix_name() == odom_name_) {
            controller_interfaces_.odom_state_interface_.emplace_back(interface);
        } else {
            const auto group_it = state_interface_map_.find(interface.get_interface_name());
            if (group_it != state_interface_map_.end()) {
                group_it->second->push_back(interface);
            } else {
                RCLCPP_WARN(get_node()->get_logger(),
                            "Ignoring unsupported state interface '%s'",
                            interface.get_name().c_str());
            }
        }
    }
    current_state_ = state_list_.no_output;
    current_state_->enter();
    next_state_ = current_state_;
    next_state_name_ = current_state_->state_name;
    mode_ = FSMMode::NORMAL;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type
Acados_NMPC_Controller::update(const rclcpp::Time &time,
                               const rclcpp::Duration &period) {
    ctrl_comp_->updateState(time, period);
    if (mode_ == FSMMode::NORMAL) {
        current_state_->run(time, period);
        next_state_name_ = current_state_->checkChange();
        if (next_state_name_ != current_state_->state_name) {
            mode_ = FSMMode::CHANGE;
            next_state_ = getNextState(next_state_name_);
            RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s", current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
        }
    } else if (mode_ == FSMMode::CHANGE) {
        current_state_->exit();
        current_state_ = next_state_;

        current_state_->enter();
        mode_ = FSMMode::NORMAL;
    }
    return controller_interface::return_type::OK;
}
controller_interface::CallbackReturn Acados_NMPC_Controller::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    release_interfaces();
    return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn Acados_NMPC_Controller::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Acados_NMPC_Controller::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Acados_NMPC_Controller::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}
std::shared_ptr<FSMState>
Acados_NMPC_Controller::getNextState(FSMStateName stateName) const {
    switch (stateName) {
    case FSMStateName::NO_OUTPUT:
        return state_list_.no_output;
    case FSMStateName::AUTO:
        return state_list_.auto_;
    case FSMStateName::MANUAL:
        return state_list_.manual;
    default:
        return state_list_.invalid;
    }
}
} // namespace acados_nmpc_controller
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(acados_nmpc_controller::Acados_NMPC_Controller, controller_interface::ControllerInterface);
