//
// Created by biao on 25-2-23.
//

#pragma once
#include "StateEstimateBase.h"
#include "nav_msgs/msg/odometry.hpp"
#include <cstdint>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
class FromOdomTopic final : public StateEstimateBase {
public:
    FromOdomTopic(ControllerInterfaces &ctrl_interfaces,
                  const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

    vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
    bool shouldPublishObservation(const rclcpp::Time &stamp);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr observation_pub_;
    realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
    std_msgs::msg::Float64MultiArray observation_msg_;
    bool publish_observation_{false};
    double observation_publish_rate_{20.0};
    int64_t last_observation_publish_time_ns_{0};
};
