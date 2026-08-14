//
// Created by qiayuan on 2022/7/24.
//
#pragma once

#include "StateEstimateBase.h"

class GroundTruth final : public StateEstimateBase {
public:
    GroundTruth(ControllerInterfaces &ctrl_component,
                const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

    vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
    nav_msgs::msg::Odometry getOdomMsg(const vector3_t &position,
                                       const vector3_t &linear_velocity) const;
};
