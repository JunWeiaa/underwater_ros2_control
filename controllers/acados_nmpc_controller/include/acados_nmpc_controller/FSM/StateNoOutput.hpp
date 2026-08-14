#ifndef _STATENOOUTPUT_HPP
#define _STATENOOUTPUT_HPP
#include "acados_nmpc_controller/FSM/StateFSM.hpp"

class StateNoOutput final : public FSMState {
public:
  explicit StateNoOutput(ControllerInterfaces &ctrl_interfaces);

  void enter() override;

  void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  void exit() override;

  FSMStateName checkChange() override;
};
#endif
