//
// Created by qiayuan on 2022/7/24.
//

#include "acados_nmpc_controller/estimator/GroundTruth.h"

GroundTruth::GroundTruth(ControllerInterfaces &ctrl_component,
                         const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) :
    StateEstimateBase(ctrl_component, node) {
    initPublishers();
}
vector_t GroundTruth::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)period;
    updateJointStates();
    updateImu();

    const vector3_t position = {
        controller_interfaces_.odom_state_interface_[0].get().get_optional().value(),
        controller_interfaces_.odom_state_interface_[1].get().get_optional().value(),
        controller_interfaces_.odom_state_interface_[2].get().get_optional().value()};

    const vector3_t linear_velocity = {
        controller_interfaces_.odom_state_interface_[3].get().get_optional().value(),
        controller_interfaces_.odom_state_interface_[4].get().get_optional().value(),
        controller_interfaces_.odom_state_interface_[5].get().get_optional().value()};

    updateLinear(position, linear_velocity);

    auto odom = getOdomMsg(position, linear_velocity);
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base";
    publishMsgs(odom);

    return rbd_state_;
}

nav_msgs::msg::Odometry GroundTruth::getOdomMsg(const vector3_t &position,
                                                const vector3_t &linear_velocity) const {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = position(0);
    odom.pose.pose.position.y = position(1);
    odom.pose.pose.position.z = position(2);
    odom.pose.pose.orientation.x = quat_.x();
    odom.pose.pose.orientation.y = quat_.y();
    odom.pose.pose.orientation.z = quat_.z();
    odom.pose.pose.orientation.w = quat_.w();

    odom.twist.twist.linear.x = linear_velocity(0);
    odom.twist.twist.linear.y = linear_velocity(1);
    odom.twist.twist.linear.z = linear_velocity(2);
    odom.twist.twist.angular.x = angular_vel_local_.x();
    odom.twist.twist.angular.y = angular_vel_local_.y();
    odom.twist.twist.angular.z = angular_vel_local_.z();

    return odom;
}
