//
// Created by biao on 3/15/25.
//

#include "acados_nmpc_controller/control/CtrlComponent.h"

#include <acados_nmpc_controller/estimator/GroundTruth.h>
#include <acados_nmpc_controller/estimator/FromOdomTopic.h>
#include <cstdint>

namespace {
template <typename T>
T declareOrGet(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
               const std::string &name,
               const T &default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
    }
    T value = default_value;
    node->get_parameter(name, value);
    return value;
}
} // namespace

CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                             ControllerInterfaces &ctrl_interfaces) :
    node_(node), ctrl_interfaces_(ctrl_interfaces) {
    target_manager_ = std::make_unique<TargetManager>(node_);
    // Init observation
    const int state_dim = static_cast<int>(declareOrGet<int64_t>(node_, "state_dim", 13));
    const int input_dim = static_cast<int>(declareOrGet<int64_t>(node_, "input_dim", 8));
    observation_.state.setZero(static_cast<long>(state_dim));
    observation_.input.setZero(static_cast<long>(input_dim));
    if (state_dim >= 7) {
        observation_.state(3) = 1.0;
    }
}

void CtrlComponent::setupStateEstimate(const std::string &estimator_type) {
    if (estimator_type == "ground_truth") {
        estimator_ = std::make_unique<GroundTruth>(
            ctrl_interfaces_,
            node_);
        RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
    } else {
        estimator_ = std::make_unique<FromOdomTopic>(ctrl_interfaces_, node_);
        RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
    }
    observation_.time = 0;
}

void CtrlComponent::updateState(const rclcpp::Time &time, const rclcpp::Duration &period) {
    // Update State Estimation
    observation_.time += period.seconds();
    observation_.state = estimator_->update(time, period);
}
