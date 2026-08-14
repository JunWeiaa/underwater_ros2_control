#include "acados_nmpc_controller/FSM/StateManual.hpp"
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

bool declareManualPathConstraintMode(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
    constexpr bool default_value = false;
    if (!node->has_parameter("manual.enforce_path_constraints")) {
        node->declare_parameter<bool>("manual.enforce_path_constraints", default_value);
    }
    bool value = default_value;
    node->get_parameter("manual.enforce_path_constraints", value);
    return value;
}

vector_t resizeStateWithDefaults(const vector_t &source, int size) {
    vector_t resized = vector_t::Zero(size);
    const int count = std::min<int>(size, static_cast<int>(source.size()));
    if (count > 0) {
        resized.head(count) = source.head(count);
    }
    if (size >= 7 && std::abs(resized(3)) + std::abs(resized(4)) + std::abs(resized(5)) + std::abs(resized(6)) < std::numeric_limits<double>::epsilon()) {
        resized(3) = 1.0;
    }
    return resized;
}
} // namespace

StateManual::StateManual(
    ControllerInterfaces &controller_interfaces,
    const std::shared_ptr<CtrlComponent> &ctrl_component) :
    FSMState(FSMStateName::MANUAL, "Manual State", controller_interfaces),
    node_(ctrl_component->node_), ctrl_component_(ctrl_component) {
    mpc_frequency_ = declareMpcFrequency(node_);
    enforce_path_constraints_ = declareManualPathConstraintMode(node_);
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
            cmd_vel_ = *msg;
        });

    controller_running_ = true;
}
StateManual::~StateManual() {
    controller_running_ = false;
    mpc_running_ = false;
    if (mpc_thread_.joinable()) {
        mpc_thread_.join();
    }
}

void StateManual::initSolverIfNeeded() {
    if (acados_interface_) {
        return;
    }

    acados_interface_ = std::make_shared<AcadosInterface>(node_);
    acados_interface_->setPathConstraintsEnabled(enforce_path_constraints_);
    ctrl_component_->observation_.state =
        resizeStateWithDefaults(ctrl_component_->observation_.state, acados_interface_->stateDim());
    if (static_cast<int>(ctrl_component_->observation_.input.size()) != acados_interface_->inputDim()) {
        ctrl_component_->observation_.input = vector_t::Zero(acados_interface_->inputDim());
    }

    u0 = vector_t::Zero(acados_interface_->inputDim());
    default_u0 = vector_t::Zero(acados_interface_->inputDim());
    current_u0 = vector_t::Zero(acados_interface_->inputDim());
    target_state_ = vector_t::Zero(acados_interface_->stateDim());
    initializeManualTrajectoryCache();
    output_mapper_ = std::make_unique<ControlOutputMapper>(node_, acados_interface_->inputDim());

    RCLCPP_INFO(node_->get_logger(),
                "Manual Acados initialized. Path constraints are %s.",
                enforce_path_constraints_ ? "enabled" : "disabled");
}

void StateManual::initializeManualTrajectoryCache() {
    const int horizon_steps = acados_interface_->horizon() + 1;
    vector_t initial_state = vector_t::Zero(acados_interface_->stateDim());
    initial_state = resizeStateWithDefaults(initial_state, acados_interface_->stateDim());

    manual_trajectory_ = std::make_shared<TargetTrajectories>(
        vector_t::Zero(horizon_steps),
        vector_array_t(static_cast<size_t>(horizon_steps), initial_state),
        vector_array_t(static_cast<size_t>(horizon_steps), vector_t::Zero(acados_interface_->inputDim())));
}

TargetTrajectories &StateManual::updateManualTrajectoryCache() {
    const int horizon_steps = acados_interface_->horizon() + 1;
    if (!manual_trajectory_ || static_cast<int>(manual_trajectory_->state().size()) != horizon_steps || static_cast<int>(manual_trajectory_->time().size()) != horizon_steps) {
        initializeManualTrajectoryCache();
    }

    auto &states = manual_trajectory_->state();
    for (auto &state : states) {
        state = target_state_;
    }
    return *manual_trajectory_;
}

geometry_msgs::msg::Twist StateManual::readCmdVel() const {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    return cmd_vel_;
}

void StateManual::enter() {
    initSolverIfNeeded();
    controller_running_ = true;
    target_state_ = resizeStateWithDefaults(
        ctrl_component_->observation_.state,
        acados_interface_->stateDim());

    mpc_thread_ = std::thread([&] {
        while (controller_running_) {
            try {
                executeAndSleep(
                    [&] {
                        if (mpc_running_) {
                            // 人的操控直觉：输入的是机体坐标系 (Body FRD) 下的速度
                            const auto cmd_vel = readCmdVel();
                            double u = cmd_vel.linear.x; // 机体前向 Surge (北向/Forward)
                            double v = cmd_vel.linear.y; // 机体侧向 Sway (东向/Right)
                            double w = cmd_vel.linear.z; // 机体下潜 Heave (地心/Down)
                            double r = cmd_vel.angular.z;

                            const double dt = 1.0 / mpc_frequency_;

                            // 获取当前的偏航角 (Global Yaw)，用来把机体速度积分到全局位置
                            vector3_t euler = quatToZyx(quaternion_t(target_state_(3), target_state_(4), target_state_(5), target_state_(6)));
                            double current_yaw = euler(0);

                            current_yaw += r * dt;

                            // 从机体(Body)坐标系旋转到全局(Global NED)坐标系
                            double dx_global = u * std::cos(current_yaw) - v * std::sin(current_yaw);
                            double dy_global = u * std::sin(current_yaw) + v * std::cos(current_yaw);
                            double dz_global = w;

                            // 在全局 NED 中积分位置
                            target_state_(0) += dx_global * dt;
                            target_state_(1) += dy_global * dt;
                            target_state_(2) += dz_global * dt;

                            if (target_state_(2) < 0.0) target_state_(2) = 0.0; // limit at surface

                            Eigen::Quaterniond q;
                            q = Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

                            // Prevent quaternion discontinuity (wrap around)
                            Eigen::Quaterniond q_prev(target_state_(3), target_state_(4), target_state_(5), target_state_(6));
                            if (q.dot(q_prev) < 0) {
                                q.w() = -q.w();
                                q.x() = -q.x();
                                q.y() = -q.y();
                                q.z() = -q.z();
                            }

                            target_state_(3) = q.w();
                            target_state_(4) = q.x();
                            target_state_(5) = q.y();
                            target_state_(6) = q.z();

                            // 目标速度就是输入的机体速度 (Body FRD velocities)
                            target_state_(7) = u; // u (Surge)
                            target_state_(8) = v; // v (Sway)
                            target_state_(9) = w; // w (Heave)

                            target_state_(10) = 0;
                            target_state_(11) = 0;
                            target_state_(12) = r;

                            auto &manual_traj = updateManualTrajectoryCache();
                            acados_interface_->create(ctrl_component_->observation_, manual_traj);
                            const bool solve_ok =
                                acados_interface_->solve() && acados_interface_->getControl().allFinite();
                            if (!solve_ok) {
                                RCLCPP_WARN_THROTTLE(
                                    node_->get_logger(),
                                    *node_->get_clock(),
                                    1000,
                                    "Manual solve failed. obs pos=[%.3f %.3f %.3f], target pos=[%.3f %.3f %.3f], path_constraints=%s",
                                    ctrl_component_->observation_.state(0),
                                    ctrl_component_->observation_.state(1),
                                    ctrl_component_->observation_.state(2),
                                    target_state_(0),
                                    target_state_(1),
                                    target_state_(2),
                                    enforce_path_constraints_ ? "enabled" : "disabled");
                            }
                            std::lock_guard<std::mutex> lock(u0_mutex_);
                            opt_failed_ = !solve_ok;
                            u0 = solve_ok ? acados_interface_->getControl() : default_u0;
                            u0_updated_ = true;
                            ctrl_component_->observation_.input = u0;
                        }
                    },
                    mpc_frequency_);
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
    RCLCPP_INFO(node_->get_logger(), "Manual MPC thread started.");
    setThreadPriority(60,
                      mpc_thread_);

    if (mpc_running_ == false) {
        auto &init_trajectories = updateManualTrajectoryCache();
        acados_interface_->create(ctrl_component_->observation_, init_trajectories);
        const bool initial_solve_ok = acados_interface_->solve() && acados_interface_->getControl().allFinite();
        if (initial_solve_ok) {
            u0 = acados_interface_->getControl();
            RCLCPP_INFO(node_->get_logger(), "Manual Initial policy has been received.");
        } else {
            u0 = default_u0;
            RCLCPP_WARN(node_->get_logger(),
                        "Manual initial solve failed. Starting Manual with default control input.");
        }
        mpc_running_ = true;
    }
}
void StateManual::run(const rclcpp::Time & /**time**/,
                      const rclcpp::Duration &period) {
    if (mpc_running_ == false) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(u0_mutex_);
        if (u0_updated_) {
            current_u0 = u0;
            u0_updated_ = false;
        } else if (opt_failed_) {
            current_u0 = default_u0;
        } else {
            current_u0 = u0;
        }
    }

    output_mapper_->write(current_u0, ctrl_component_->observation_.state, period, controller_interfaces_);
}

void StateManual::exit() {
    mpc_running_ = false;
    controller_running_ = false;
    if (mpc_thread_.joinable()) {
        mpc_thread_.join();
    }
    const int input_dim = acados_interface_ ? acados_interface_->inputDim() : static_cast<int>(current_u0.size());
    u0 = vector_t::Zero(input_dim);
    ctrl_component_->observation_.input = u0;
    RCLCPP_INFO(node_->get_logger(), "Manual thread stopped.");
}
FSMStateName StateManual::checkChange() {
    if (!current_u0.allFinite()) {
        RCLCPP_ERROR(node_->get_logger(), "Control input is not finite, stopping the controller.");
        return FSMStateName::NO_OUTPUT;
    }

    switch (controller_interfaces_.ctrl_state_command_.data) {
    case 1:
        return FSMStateName::NO_OUTPUT;
    case 2:
        return FSMStateName::AUTO;
    default:
        return FSMStateName::MANUAL;
    }
}
