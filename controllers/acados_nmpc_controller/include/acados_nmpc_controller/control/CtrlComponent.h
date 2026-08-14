//
// Created by biao on 3/15/25.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <memory>
#include <string>

#include <acados_nmpc_controller/estimator/StateEstimateBase.h>
#include "acados_nmpc_controller/control/TargetManager.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

struct SystemObservation {
    scalar_t time = 0.0;
    vector_t state;
    vector_t input;
};
class CtrlComponent {
public:
    explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                           ControllerInterfaces &ctrl_interfaces);
    ~CtrlComponent() = default;
    void setupStateEstimate(const std::string &estimator_type);
    void updateState(const rclcpp::Time &time, const rclcpp::Duration &period);

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

    TargetManager &getTargetManager() {
        return *target_manager_;
    }
    SystemObservation observation_;

private:
    ControllerInterfaces &ctrl_interfaces_;
    std::unique_ptr<StateEstimateBase> estimator_;
    std::unique_ptr<TargetManager> target_manager_;
};

#endif // CTRLCOMPONENT_H
