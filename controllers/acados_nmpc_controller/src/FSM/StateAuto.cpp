#include "acados_nmpc_controller/FSM/StateAuto.hpp"
#include "acados_nmpc_controller/control/TargetManager.hpp"
#include "acados_nmpc_controller/utils/TargetTrajectories.hpp"
#include "acados_nmpc_controller/utils/Types.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>

namespace {
double declareMpcFrequency(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
    constexpr double default_frequency = 200.0;
    if (!node->has_parameter("mpc.frequency")) {
        node->declare_parameter<double>("mpc.frequency", default_frequency);
    }
    double frequency = default_frequency;
    node->get_parameter("mpc.frequency", frequency);
    if (frequency <= 0.0) {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid mpc.frequency %.3f; using %.3f Hz",
                    frequency,
                    default_frequency);
        return default_frequency;
    }
    return frequency;
}

vector_t resizeWithDefaults(const vector_t &source, int size) {
    vector_t resized = vector_t::Zero(size);
    const int count = std::min<int>(size, static_cast<int>(source.size()));
    if (count > 0) {
        resized.head(count) = source.head(count);
    }
    if (size >= 7 && std::abs(resized(3)) + std::abs(resized(4)) +
                         std::abs(resized(5)) + std::abs(resized(6)) <
                     std::numeric_limits<double>::epsilon()) {
        resized(3) = 1.0;
    }
    return resized;
}
} // namespace

StateAuto::StateAuto(
    ControllerInterfaces &controller_interfaces,
    const std::shared_ptr<CtrlComponent> &ctrl_component) :
    FSMState(FSMStateName::AUTO, "Auto State", controller_interfaces),
    node_(ctrl_component->node_), ctrl_component_(ctrl_component) {
    mpc_frequency_ = declareMpcFrequency(node_);
}
StateAuto::~StateAuto() {
    controller_running_ = false;
    mpc_running_ = false;
    if (mpc_thread_.joinable()) {
        mpc_thread_.join();
    }
}

void StateAuto::initSolverIfNeeded() {
    if (acados_interface_) {
        return;
    }

    acados_interface_ = std::make_shared<AcadosInterface>(node_);
    ctrl_component_->observation_.state =
        resizeWithDefaults(ctrl_component_->observation_.state, acados_interface_->stateDim());
    if (static_cast<int>(ctrl_component_->observation_.input.size()) != acados_interface_->inputDim()) {
        ctrl_component_->observation_.input = vector_t::Zero(acados_interface_->inputDim());
    }

    u0 = vector_t::Zero(acados_interface_->inputDim());
    default_u0 = vector_t::Zero(acados_interface_->inputDim());
    current_u0 = vector_t::Zero(acados_interface_->inputDim());
    output_mapper_ = std::make_unique<ControlOutputMapper>(node_, acados_interface_->inputDim());

    controller_running_ = true;

    RCLCPP_INFO(node_->get_logger(), "Acados initialized.");
}

void StateAuto::enter() {
    initSolverIfNeeded();
    controller_running_ = true;
    mpc_thread_ = std::thread([&] {
        while (controller_running_) {
            try {
                executeAndSleep(
                    [&] {
                        if (mpc_running_) {
                            // mpc_timer_.startTimer();
                            TargetTrajectories current_trajectory = ctrl_component_->getTargetManager().getCurrentTrajectorySegment();
                            acados_interface_->create(ctrl_component_->observation_,
                                                      current_trajectory);
                            const bool solve_ok =
                                acados_interface_->solve() && acados_interface_->getControl().allFinite();
                            std::lock_guard<std::mutex> lock(u0_mutex_);
                            opt_failed_ = !solve_ok;
                            u0 = solve_ok ? acados_interface_->getControl() : default_u0;
                            u0_updated_ = true;
                            ctrl_component_->observation_.input = u0;
                        }

                        // mpc_timer_.endTimer();
                    },
                    mpc_frequency_

                );
            } catch (const std::exception &e) {
                controller_running_ = false;
                RCLCPP_WARN(node_->get_logger(), "[Acados MPC thread] Error : %s.Using default control input.", e.what());
                {
                    std::lock_guard<std::mutex> lock(u0_mutex_);
                    opt_failed_ = true;
                    u0 = default_u0; // 使用默认值
                    ctrl_component_->observation_.input = u0;
                    u0_updated_ = true;
                }
            }
        }
    });
    // TODO: add priority
    setThreadPriority(60,
                      mpc_thread_);

    if (mpc_running_ == false) {
        TargetTrajectories init_trajectories(
            vector_t::Zero(1),
            vector_array_t{ctrl_component_->observation_.state},
            vector_array_t{ctrl_component_->observation_.input}); // just hold position
        // Set the first observation and command and wait for optimization to finish
        acados_interface_->create(ctrl_component_->observation_, init_trajectories);
        const bool initial_solve_ok = acados_interface_->solve() && acados_interface_->getControl().allFinite();
        // RCLCPP_WARN(node_->get_logger(), "we made here");
        u0 = initial_solve_ok ? acados_interface_->getControl() : default_u0;

        RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

        mpc_running_ = true;
    }
}
void StateAuto::run(const rclcpp::Time & /**time**/,
                    const rclcpp::Duration &period) {
    if (mpc_running_ == false) {
        return;
    }
    // RCLCPP_INFO(node_->get_logger(), "We made here.");
    ctrl_component_->getTargetManager().updateTrajectoryBuffer();

    {
        std::lock_guard<std::mutex> lock(u0_mutex_);
        if (opt_failed_) {
            RCLCPP_WARN(node_->get_logger(), "Optimization failed, using default control input.");
            current_u0 = default_u0; // 使用默认值
        } else if (u0_updated_) {
            current_u0 = u0;
            u0_updated_ = false; // 重置标志位
        } else {
            // RCLCPP_WARN(node_->get_logger(), "u0 not updated, using previous value.");
            current_u0 = u0; // 使用上一次的值
        }
    }

    output_mapper_->write(current_u0, ctrl_component_->observation_.state, period, controller_interfaces_);
}

void StateAuto::exit() {
    mpc_running_ = false;
    controller_running_ = false;
    if (mpc_thread_.joinable()) {
        mpc_thread_.join();
    }
    const int input_dim = acados_interface_ ? acados_interface_->inputDim() : static_cast<int>(current_u0.size());
    u0 = vector_t::Zero(input_dim);
    ctrl_component_->observation_.input = u0;

    ctrl_component_->getTargetManager().exit();
    RCLCPP_INFO(node_->get_logger(), "Acados thread stopped.");
}
FSMStateName StateAuto::checkChange() {
    // Safety check, if failed, stop the controller

    if (ctrl_component_->observation_.state.size() >= 7) {
        vector3_t zyx = quatToZyx(quaternion_t(ctrl_component_->observation_.state(3),
                                               ctrl_component_->observation_.state(4),
                                               ctrl_component_->observation_.state(5),
                                               ctrl_component_->observation_.state(6)));
        (void)zyx;
    }
    // if ((std::norm(zyx(1)) > boost::math::double_constants::pi / 3)
    //     || (std::norm(zyx(2)) > boost::math::double_constants::pi / 3)) {
    //     RCLCPP_ERROR(node_->get_logger(), "Angle is too sharp, stopping the controller.");
    //     return FSMStateName::NO_OUTPUT;
    // }
    // if (opt_failed_) {
    //     RCLCPP_ERROR(node_->get_logger(), "Optimization failed, stopping the controller.");
    //     return FSMStateName::NO_OUTPUT;
    // }
    if (!current_u0.allFinite()) {
        RCLCPP_ERROR(node_->get_logger(), "Control input is not finite, stopping the controller.");
        return FSMStateName::NO_OUTPUT;
    }

    switch (controller_interfaces_.ctrl_state_command_.data) {
    case 1:
        return FSMStateName::NO_OUTPUT;
    case 3:
        return FSMStateName::MANUAL;
    default:
        return FSMStateName::AUTO;
    }
}
