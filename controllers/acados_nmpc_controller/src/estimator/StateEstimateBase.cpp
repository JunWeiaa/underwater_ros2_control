//
// Created by qiayuan on 2021/11/15.
//

#include "acados_nmpc_controller/estimator/StateEstimateBase.h"
#include "acados_nmpc_controller/utils/Types.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>

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

StateEstimateBase::StateEstimateBase(
    ControllerInterfaces &ctrl_component,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) :
    controller_interfaces_(ctrl_component),
    node_(std::move(node)) {
    const int state_dim = std::max<int>(
        13,
        static_cast<int>(declareOrGet<int64_t>(node_, "state_dim", 13)));
    rbd_state_.setZero(state_dim);
    rbd_state_(3) = 1.0;

    const auto joints = declareOrGet<std::vector<std::string>>(node_, "state_extra_map.joints", std::vector<std::string>{});
    auto interfaces = declareOrGet<std::vector<std::string>>(
        node_,
        "state_extra_map.interfaces",
        std::vector<std::string>(joints.size(), "position"));
    auto indices = declareOrGet<std::vector<int64_t>>(node_, "state_extra_map.indices", std::vector<int64_t>{});
    auto defaults = declareOrGet<std::vector<double>>(
        node_,
        "state_extra_map.defaults",
        std::vector<double>(joints.size(), 0.0));

    if (indices.empty() && !joints.empty()) {
        indices.resize(joints.size());
        for (size_t i = 0; i < joints.size(); ++i) {
            indices[i] = static_cast<int64_t>(13 + i);
        }
    }

    if (!joints.empty() && (interfaces.size() != joints.size() || indices.size() != joints.size() || defaults.size() != joints.size())) {
        RCLCPP_WARN(node_->get_logger(),
                    "state_extra_map arrays must match joints size. Extra state mapping disabled.");
        return;
    }

    state_extra_map_.reserve(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        StateExtraMapEntry entry;
        entry.joint = joints[i];
        entry.interface = interfaces[i];
        entry.state_index = static_cast<int>(indices[i]);
        entry.default_value = defaults[i];
        state_extra_map_.push_back(entry);
    }
}

void StateEstimateBase::updateJointStates() {
    for (const auto &entry : state_extra_map_) {
        if (entry.state_index < 0 || entry.state_index >= rbd_state_.size()) {
            RCLCPP_WARN(node_->get_logger(),
                        "state_extra_map index %d is outside state size %ld",
                        entry.state_index,
                        rbd_state_.size());
            continue;
        }

        const std::string key = entry.joint + "/" + entry.interface;
        double value = entry.default_value;
        const auto interface_it = controller_interfaces_.state_interfaces_by_name_.find(key);
        if (interface_it != controller_interfaces_.state_interfaces_by_name_.end()) {
            value = interface_it->second->get_optional().value_or(entry.default_value);
        }
        rbd_state_(entry.state_index) = value;
    }
}

void StateEstimateBase::updateImu() {
    quat_ = {
        controller_interfaces_.imu_state_interface_[0].get().get_optional().value(),
        controller_interfaces_.imu_state_interface_[1].get().get_optional().value(),
        controller_interfaces_.imu_state_interface_[2].get().get_optional().value(),
        controller_interfaces_.imu_state_interface_[3].get().get_optional().value()};

    angular_vel_local_ = {
        controller_interfaces_.imu_state_interface_[4].get().get_optional().value(),
        controller_interfaces_.imu_state_interface_[5].get().get_optional().value(),
        controller_interfaces_.imu_state_interface_[6].get().get_optional().value()};

    updateAngular(quat_, angular_vel_local_);
}

void StateEstimateBase::initPublishers() {
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
}

void StateEstimateBase::updateAngular(const quaternion_t &quat, const vector_t &angularVel) {
    // Convert pose from ENU/FLU to NED/FRD.

    quaternion_t q_enu_to_ned;
    q_enu_to_ned.w() = 0.0;
    q_enu_to_ned.x() = 0.7071; // sqrt(2)/2
    q_enu_to_ned.y() = 0.7071; // sqrt(2)/2
    q_enu_to_ned.z() = 0.0;

    // FLU to FRD rotation quaternion: 180 degrees about X.
    quaternion_t q_flu_to_frd(0.0, 1.0, 0.0, 0.0);

    // Compose q_ned_frd = q_enu_to_ned * q_enu_flu * q_flu_to_frd.
    quaternion_t quat_ned_frd = q_enu_to_ned * quat * q_flu_to_frd;

    // Keep the scalar part positive as the normalization convention.
    if (quat_ned_frd.w() < 0) {
        quat_ned_frd.coeffs() = -quat_ned_frd.coeffs();
    }

    // Store the FRD body quaternion in the NED frame.
    rbd_state_.segment<4>(3) << quat_ned_frd.w(), quat_ned_frd.x(), quat_ned_frd.y(), quat_ned_frd.z();

    // Convert angular velocity from FLU body frame to FRD body frame.
    // X_frd = X_flu, Y_frd = -Y_flu, Z_frd = -Z_flu
    vector_t angularVel_frd(3);
    angularVel_frd(0) = angularVel(0);  // p (roll rate)
    angularVel_frd(1) = -angularVel(1); // q (pitch rate)
    angularVel_frd(2) = -angularVel(2); // r (yaw rate)

    rbd_state_.segment<3>(10) = angularVel_frd;
}

void StateEstimateBase::updateLinear(const vector_t &pos, const vector_t &linearVel) {
    // Convert position from ENU to NED.
    // X_ned = Y_enu (North = East)
    // Y_ned = X_enu (East = North)
    // Z_ned = -Z_enu (Down = -Up)
    vector_t pos_ned(3);

    pos_ned(0) = pos(1);  // North = East
    pos_ned(1) = pos(0);  // East = North
    pos_ned(2) = -pos(2); // Down = -Up

    // Convert linear velocity from FLU body frame to FRD body frame.
    vector_t linearVel_frd(3);
    linearVel_frd(0) = linearVel(0);  // X: forward unchanged
    linearVel_frd(1) = -linearVel(1); // Y: left to right
    linearVel_frd(2) = -linearVel(2); // Z: up to down

    rbd_state_.segment<3>(0) = pos_ned;
    rbd_state_.segment<3>(7) = linearVel_frd;
}

void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry &odom) const {
    rclcpp::Time time = odom.header.stamp;
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = odom.header;
    pose.pose.pose = odom.pose.pose;
    pose_pub_->publish(pose);
}
