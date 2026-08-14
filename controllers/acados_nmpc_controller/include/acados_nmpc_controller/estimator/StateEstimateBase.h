//
// Created by qiayuan on 2021/11/15.
//
#pragma once
#include "acados_nmpc_controller/interfaces/ControllerInterfaces.hpp"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "acados_nmpc_controller/utils/Types.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vector>

class StateEstimateBase {
public:
    virtual ~StateEstimateBase() = default;

    StateEstimateBase(ControllerInterfaces &controller_interfaces,
                      rclcpp_lifecycle::LifecycleNode::SharedPtr node);

    virtual void updateJointStates();

    virtual void updateImu();

    virtual vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

protected:
    struct StateExtraMapEntry {
        std::string joint;
        std::string interface;
        int state_index{-1};
        double default_value{0.0};
    };

    void initPublishers();

    void updateAngular(const quaternion_t &quat, const vector_t &angularVel);

    void updateLinear(const vector_t &pos, const vector_t &linearVel);

    void publishMsgs(const nav_msgs::msg::Odometry &odom) const;

    ControllerInterfaces &controller_interfaces_;
    std::vector<StateExtraMapEntry> state_extra_map_;

    vector_t rbd_state_;
    quaternion_t quat_;
    vector3_t angular_vel_local_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

template <typename T>
T square(T a) {
    return a * a;
}
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q) {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                        square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                        square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}
