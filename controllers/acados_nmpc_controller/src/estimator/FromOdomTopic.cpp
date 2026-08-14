//
// Created by biao on 25-2-23.
//

#include "acados_nmpc_controller/estimator/FromOdomTopic.h"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>
#include <string>

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

FromOdomTopic::FromOdomTopic(ControllerInterfaces &ctrl_interfaces,
                             const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) :
    StateEstimateBase(
        ctrl_interfaces,
        node) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            buffer_.writeFromNonRT(*msg);
        });
    publish_observation_ = declareOrGet<bool>(node_, "debug.publish_observation", false);
    observation_publish_rate_ = declareOrGet<double>(node_, "debug.observation_publish_rate", 20.0);
    if (publish_observation_ && observation_publish_rate_ > 0.0) {
        observation_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("observation", 10);
        observation_msg_.data.resize(static_cast<size_t>(rbd_state_.size()));
    } else if (publish_observation_) {
        RCLCPP_WARN(node_->get_logger(),
                    "debug.publish_observation is true but debug.observation_publish_rate %.3f is not positive; observation publish disabled",
                    observation_publish_rate_);
        publish_observation_ = false;
    }
}

bool FromOdomTopic::shouldPublishObservation(const rclcpp::Time &stamp) {
    if (!observation_pub_ || !publish_observation_ || observation_publish_rate_ <= 0.0) {
        return false;
    }

    const int64_t now_ns = stamp.nanoseconds();
    const auto publish_period_ns =
        static_cast<int64_t>(std::llround(1e9 / observation_publish_rate_));
    if (last_observation_publish_time_ns_ == 0 ||
        now_ns < last_observation_publish_time_ns_ ||
        now_ns - last_observation_publish_time_ns_ >= publish_period_ns) {
        last_observation_publish_time_ns_ = now_ns;
        return true;
    }
    return false;
}

vector_t FromOdomTopic::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)period;
    const auto odom = buffer_.readFromRT();
    if (odom == nullptr) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(),
                             *node_->get_clock(),
                             1000,
                             "Waiting for odometry before updating state estimate");
        return rbd_state_;
    }

    updateJointStates();
    updateImu();
    updateLinear(Eigen::Matrix<scalar_t, 3, 1>(
                     odom->pose.pose.position.x,
                     odom->pose.pose.position.y,
                     odom->pose.pose.position.z),
                 Eigen::Matrix<scalar_t, 3, 1>(
                     odom->twist.twist.linear.x,
                     odom->twist.twist.linear.y,
                     odom->twist.twist.linear.z));
    // RCLCPP_INFO(node_->get_logger(), "distance to origin: %f", sqrt(rbd_state_(0) * rbd_state_(0) + rbd_state_(1) * rbd_state_(1)));
    if (shouldPublishObservation(time)) {
        if (observation_msg_.data.size() != static_cast<size_t>(rbd_state_.size())) {
            observation_msg_.data.resize(static_cast<size_t>(rbd_state_.size()));
        }
        for (int i = 0; i < rbd_state_.size(); ++i) {
            observation_msg_.data[static_cast<size_t>(i)] = rbd_state_(i);
        }
        observation_pub_->publish(observation_msg_);
    }
    return rbd_state_;
}
