#ifndef KEYBOARDINPUT_H
#define KEYBOARDINPUT_H
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>

class KeyboardInput final : public rclcpp::Node {
public:
    KeyboardInput();

    ~KeyboardInput() override {
        if (terminal_configured_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
        }
    }

private:
    struct TargetPublishers {
        std::string robot_namespace;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_input_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    };

    void timer_callback();

    void check_command(char key);
    bool check_value(char key);
    void initialize_targets();
    void add_targets(const std::vector<std::string> &robot_namespaces, bool explicit_targets);
    std::vector<std::string> discover_robot_namespaces() const;
    bool refresh_discovered_targets();
    void switch_target(int direction);
    void publish_control_input(const std_msgs::msg::Int8 &input);
    void publish_cmd_vel(const geometry_msgs::msg::Twist &cmd_vel);
    void publish_zero_cmd_vel(size_t target_index);
    const TargetPublishers &active_target() const;
    std::string active_target_name() const;

    static bool kbhit();

    std_msgs::msg::Int8 input_;
    geometry_msgs::msg::Twist cmd_vel_;
    std::vector<TargetPublishers> targets_;
    size_t active_target_index_{0};
    rclcpp::TimerBase::SharedPtr timer_;

    bool just_published_ = false;
    int reset_count_ = 0;
    int reset_ticks_ = 6;
    int discovery_count_ = 0;
    int discovery_ticks_ = 50;
    double linear_speed_ = 0.6;
    double vertical_speed_ = 0.6;
    double yaw_rate_ = 0.5;
    bool auto_discover_ = false;
    bool preferred_active_selected_ = false;
    bool terminal_configured_ = false;
    std::string preferred_active_robot_;

    termios old_tio_{}, new_tio_{};
};

#endif // KEYBOARDINPUT_H
