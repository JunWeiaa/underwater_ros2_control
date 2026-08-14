#ifndef _STATEMANUAL_HPP
#define _STATEMANUAL_HPP
#include "acados_nmpc_controller/FSM/StateFSM.hpp"
#include "acados_nmpc_controller/control/ControlOutputMapper.hpp"
#include "acados_nmpc_controller/control/CtrlComponent.h"
#include "acados_nmpc_controller/interfaces/acadosInterface.hpp"
#include "acados_nmpc_controller/utils/Types.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

class StateManual final : public FSMState {
public:
    ~StateManual();

    StateManual(ControllerInterfaces &ctrl_interfaces,
                const std::shared_ptr<CtrlComponent> &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;

protected:
    void initSolverIfNeeded();
    void initializeManualTrajectoryCache();
    TargetTrajectories &updateManualTrajectoryCache();
    geometry_msgs::msg::Twist readCmdVel() const;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::shared_ptr<CtrlComponent> ctrl_component_;
    // Nonlinear MPC
    std::shared_ptr<AcadosInterface> acados_interface_;
    std::unique_ptr<ControlOutputMapper> output_mapper_;
    std::thread mpc_thread_;
    geometry_msgs::msg::Twist cmd_vel_;
    mutable std::mutex cmd_vel_mutex_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    vector_t target_state_;
    std::shared_ptr<TargetTrajectories> manual_trajectory_;

    std::atomic_bool controller_running_{}, mpc_running_{};

    std::mutex u0_mutex_;
    vector_t u0;
    vector_t default_u0;
    vector_t current_u0;
    double mpc_frequency_{200.0};
    bool enforce_path_constraints_{false};
    bool u0_updated_{false};
    bool opt_failed_{false};
};
#endif
