#ifndef _STATEAUTO_HPP
#define _STATEAUTO_HPP
#include "acados_nmpc_controller/FSM/StateFSM.hpp"
#include "acados_nmpc_controller/control/ControlOutputMapper.hpp"
#include "acados_nmpc_controller/control/CtrlComponent.h"
#include "acados_nmpc_controller/interfaces/acadosInterface.hpp"
#include "acados_nmpc_controller/utils/Types.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

class StateAuto final : public FSMState {
public:
    ~StateAuto();

    StateAuto(ControllerInterfaces &ctrl_interfaces,
              const std::shared_ptr<CtrlComponent> &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;

protected:
    void initSolverIfNeeded();

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::shared_ptr<CtrlComponent> ctrl_component_;
    // Nonlinear MPC
    std::shared_ptr<AcadosInterface> acados_interface_;
    std::unique_ptr<ControlOutputMapper> output_mapper_;
    std::thread mpc_thread_;
    std::atomic_bool controller_running_{}, mpc_running_{};

    std::mutex u0_mutex_;
    vector_t u0;
    vector_t default_u0;
    vector_t current_u0;
    double mpc_frequency_{200.0};
    bool u0_updated_{false};
    bool opt_failed_{false};
};
#endif
