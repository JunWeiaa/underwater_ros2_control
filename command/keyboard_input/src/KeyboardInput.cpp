#include <keyboard_input/KeyboardInput.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <map>
#include <set>
#include <sstream>

namespace {
constexpr double kDefaultRepeatRate = 50.0;
constexpr double kDefaultCommandTimeout = 0.12;

bool hasVelocityCommand(const geometry_msgs::msg::Twist &cmd_vel) {
    return cmd_vel.linear.x != 0.0 || cmd_vel.linear.y != 0.0 || cmd_vel.linear.z != 0.0 || cmd_vel.angular.x != 0.0 || cmd_vel.angular.y != 0.0 || cmd_vel.angular.z != 0.0;
}

std::string trim(const std::string &value) {
    const auto begin = value.find_first_not_of(" \t\n\r/");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\n\r/");
    return value.substr(begin, end - begin + 1);
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::vector<std::string> parseRobotList(std::string robots_text) {
    std::replace(robots_text.begin(), robots_text.end(), ';', ',');
    std::vector<std::string> robots;
    std::stringstream stream(robots_text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        token = trim(token);
        if (!token.empty()) {
            robots.push_back(token);
        }
    }
    return robots;
}

bool namespaceFromTopic(const std::string &topic_name,
                        const std::string &leaf_name,
                        std::string &robot_namespace) {
    const std::string suffix = "/" + leaf_name;
    if (topic_name == suffix) {
        robot_namespace.clear();
        return true;
    }
    if (topic_name.size() <= suffix.size() || topic_name.compare(topic_name.size() - suffix.size(), suffix.size(), suffix) != 0) {
        return false;
    }
    robot_namespace = trim(topic_name.substr(0, topic_name.size() - suffix.size()));
    return true;
}

std::string topicForRobot(const std::string &robot_namespace, const std::string &topic_name) {
    if (robot_namespace.empty()) {
        return topic_name;
    }
    return "/" + robot_namespace + "/" + topic_name;
}
} // namespace

KeyboardInput::KeyboardInput() :
    Node("keyboard_input_node") {
    linear_speed_ = declare_parameter<double>("linear_speed", 0.6);
    vertical_speed_ = declare_parameter<double>("vertical_speed", 0.6);
    yaw_rate_ = declare_parameter<double>("yaw_rate", 0.5);
    double repeat_rate = declare_parameter<double>("repeat_rate", kDefaultRepeatRate);
    const double command_timeout = declare_parameter<double>("command_timeout", kDefaultCommandTimeout);
    if (repeat_rate <= 0.0) {
        RCLCPP_WARN(get_logger(),
                    "Invalid repeat_rate %.3f; using %.3f Hz",
                    repeat_rate,
                    kDefaultRepeatRate);
        repeat_rate = kDefaultRepeatRate;
    }
    reset_ticks_ = std::max(1, static_cast<int>(std::ceil(command_timeout * repeat_rate)));
    discovery_ticks_ = std::max(1, static_cast<int>(std::ceil(repeat_rate)));

    initialize_targets();

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / repeat_rate));
    timer_ = create_wall_timer(timer_period, std::bind(&KeyboardInput::timer_callback, this));
    input_ = std_msgs::msg::Int8();

    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &old_tio_) == 0) {
        new_tio_ = old_tio_;
        new_tio_.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        terminal_configured_ = tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_) == 0;
    }
    if (!terminal_configured_) {
        RCLCPP_WARN(get_logger(),
                    "Keyboard input node does not have an interactive terminal. Run it from a terminal, not inside a non-interactive launch process.");
    }

    RCLCPP_INFO(get_logger(), "Keyboard input node started.");
    RCLCPP_INFO(get_logger(), "Active robot: %s", active_target_name().c_str());
    RCLCPP_INFO(get_logger(), "Press [/] or ,/. to switch robot target.");
    RCLCPP_INFO(get_logger(), "Press 1 for stop, 2 for auto, 3 for manual.");
    RCLCPP_INFO(get_logger(), "Use W/S/A/D to move horizontally, Space for up, C or X for down, J/L or Q/E for yaw.");
    RCLCPP_INFO(get_logger(), "Please input keys, press Ctrl+C to quit.");
}

void KeyboardInput::timer_callback() {
    if (!terminal_configured_) {
        return;
    }

    if (auto_discover_) {
        discovery_count_ += 1;
        if (discovery_count_ >= discovery_ticks_) {
            discovery_count_ = 0;
            refresh_discovered_targets();
        }
    }

    if (kbhit()) {
        char key = getchar();
        if (key == '[' || key == ',') {
            switch_target(-1);
            return;
        }
        if (key == ']' || key == '.' || key == '\t') {
            switch_target(1);
            return;
        }

        switch (key) {
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
            check_command(key);
            publish_control_input(input_);
            just_published_ = true;
            reset_count_ = reset_ticks_;
            break;
        default:
            if (check_value(key)) {
                publish_cmd_vel(cmd_vel_);
                reset_count_ = reset_ticks_;
            }
            break;
        }

    } else {
        if (reset_count_ > 0) {
            reset_count_ -= 1;
            if (hasVelocityCommand(cmd_vel_)) {
                publish_cmd_vel(cmd_vel_);
            }
        } else {
            if (just_published_ && input_.data != 0) {
                just_published_ = false;
                input_.data = 0;
                publish_control_input(input_);
            }
            if (hasVelocityCommand(cmd_vel_)) {
                cmd_vel_.linear.x = 0;
                cmd_vel_.linear.y = 0;
                cmd_vel_.linear.z = 0;
                cmd_vel_.angular.x = 0;
                cmd_vel_.angular.y = 0;
                cmd_vel_.angular.z = 0;
                publish_cmd_vel(cmd_vel_);
            }
        }
    }
}

void KeyboardInput::initialize_targets() {
    const auto robot_namespaces = declare_parameter<std::vector<std::string>>(
        "robot_namespaces", std::vector<std::string>{});
    const std::string robots_text = declare_parameter<std::string>("robots", "");
    const std::string target_namespace = declare_parameter<std::string>("target_namespace", "");
    preferred_active_robot_ = trim(declare_parameter<std::string>("active_robot", ""));
    auto_discover_ = declare_parameter<bool>("auto_discover", false);

    std::vector<std::string> robots;
    if (toLower(trim(robots_text)) == "auto") {
        auto_discover_ = true;
    } else {
        robots = parseRobotList(robots_text);
    }
    for (const auto &robot_namespace : robot_namespaces) {
        const auto normalized = trim(robot_namespace);
        if (!normalized.empty()) {
            robots.push_back(normalized);
        }
    }

    bool explicit_targets = true;
    if (robots.empty()) {
        if (auto_discover_) {
            robots = discover_robot_namespaces();
        }
    }

    if (robots.empty()) {
        const auto normalized_target = trim(target_namespace);
        if (!normalized_target.empty()) {
            robots.push_back(normalized_target);
        } else if (!auto_discover_) {
            explicit_targets = false;
            robots.push_back(trim(get_namespace()));
        }
    }

    targets_.clear();
    active_target_index_ = 0;
    add_targets(robots, explicit_targets);

    if (targets_.empty() && auto_discover_) {
        RCLCPP_INFO(get_logger(), "No keyboard targets found yet. Waiting for /<robot>/control_input and /<robot>/cmd_vel topics.");
    }
}

void KeyboardInput::add_targets(const std::vector<std::string> &robot_namespaces, bool explicit_targets) {
    const auto before_count = targets_.size();
    targets_.reserve(targets_.size() + robot_namespaces.size());
    for (const auto &robot_namespace : robot_namespaces) {
        const auto already_configured =
            std::find_if(targets_.begin(), targets_.end(), [&robot_namespace](const TargetPublishers &target) {
                return target.robot_namespace == robot_namespace;
            })
            != targets_.end();
        if (already_configured) {
            continue;
        }

        TargetPublishers target;
        target.robot_namespace = robot_namespace;
        const std::string control_topic =
            explicit_targets ? topicForRobot(robot_namespace, "control_input") : "control_input";
        const std::string cmd_topic =
            explicit_targets ? topicForRobot(robot_namespace, "cmd_vel") : "cmd_vel";
        target.control_input_pub = create_publisher<std_msgs::msg::Int8>(control_topic, 10);
        target.cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
        targets_.push_back(target);
        RCLCPP_INFO(get_logger(),
                    "Keyboard target '%s': control_input=%s cmd_vel=%s",
                    robot_namespace.empty() ? "/" : robot_namespace.c_str(),
                    control_topic.c_str(),
                    cmd_topic.c_str());
    }

    if (targets_.empty()) {
        return;
    }
    if (active_target_index_ >= targets_.size()) {
        active_target_index_ = 0;
    }

    if (!preferred_active_robot_.empty() && !preferred_active_selected_) {
        for (size_t i = 0; i < targets_.size(); ++i) {
            if (targets_[i].robot_namespace == preferred_active_robot_) {
                active_target_index_ = i;
                preferred_active_selected_ = true;
                break;
            }
        }
    }

    if (targets_.size() != before_count) {
        RCLCPP_INFO(get_logger(), "Active robot: %s", active_target_name().c_str());
    }
}

std::vector<std::string> KeyboardInput::discover_robot_namespaces() const {
    std::map<std::string, std::set<std::string>> by_namespace;
    for (const auto &topic_and_types : get_topic_names_and_types()) {
        for (const auto &leaf_name : {"control_input", "cmd_vel"}) {
            std::string robot_namespace;
            if (namespaceFromTopic(topic_and_types.first, leaf_name, robot_namespace)) {
                by_namespace[robot_namespace].insert(leaf_name);
            }
        }
    }

    std::vector<std::string> robot_namespaces;
    for (const auto &namespace_and_leaves : by_namespace) {
        if (namespace_and_leaves.second.count("control_input") > 0 && namespace_and_leaves.second.count("cmd_vel") > 0) {
            robot_namespaces.push_back(namespace_and_leaves.first);
        }
    }
    return robot_namespaces;
}

bool KeyboardInput::refresh_discovered_targets() {
    if (!auto_discover_) {
        return false;
    }

    const auto before_count = targets_.size();
    add_targets(discover_robot_namespaces(), true);
    return targets_.size() != before_count;
}

void KeyboardInput::switch_target(int direction) {
    refresh_discovered_targets();

    if (targets_.empty()) {
        RCLCPP_INFO_THROTTLE(get_logger(),
                             *get_clock(),
                             2000,
                             "No keyboard targets are configured yet.");
        return;
    }

    if (targets_.size() <= 1) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Only one keyboard target is configured.");
        return;
    }

    publish_zero_cmd_vel(active_target_index_);
    cmd_vel_ = geometry_msgs::msg::Twist();
    reset_count_ = 0;

    const auto target_count = static_cast<int>(targets_.size());
    auto next_index = static_cast<int>(active_target_index_) + direction;
    next_index = (next_index % target_count + target_count) % target_count;
    active_target_index_ = static_cast<size_t>(next_index);
    just_published_ = false;

    RCLCPP_INFO(get_logger(), "Active robot: %s", active_target_name().c_str());
}

void KeyboardInput::publish_control_input(const std_msgs::msg::Int8 &input) {
    if (targets_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No keyboard target is available for control_input.");
        return;
    }
    active_target().control_input_pub->publish(input);
}

void KeyboardInput::publish_cmd_vel(const geometry_msgs::msg::Twist &cmd_vel) {
    if (targets_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No keyboard target is available for cmd_vel.");
        return;
    }
    active_target().cmd_vel_pub->publish(cmd_vel);
}

void KeyboardInput::publish_zero_cmd_vel(size_t target_index) {
    if (target_index >= targets_.size()) {
        return;
    }
    geometry_msgs::msg::Twist zero_cmd;
    targets_[target_index].cmd_vel_pub->publish(zero_cmd);
}

const KeyboardInput::TargetPublishers &KeyboardInput::active_target() const {
    return targets_[active_target_index_];
}

std::string KeyboardInput::active_target_name() const {
    if (targets_.empty()) {
        return "(none)";
    }
    const auto &target = active_target();
    if (target.robot_namespace.empty()) {
        return "/";
    }
    return target.robot_namespace;
}

void KeyboardInput::check_command(const char key) {
    switch (key) {
    case '1': input_.data = 1; break;
    case '2': input_.data = 2; break;
    case '3': input_.data = 3; break;
    case '4': input_.data = 4; break;
    case '5': input_.data = 5; break;
    case '6': input_.data = 6; break;
    case '7': input_.data = 7; break;
    case '8': input_.data = 8; break;
    case '9': input_.data = 9; break;
    case '0': input_.data = 10; break;
    default: input_.data = 0; break;
    }
}

bool KeyboardInput::check_value(char key) {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;

    switch (key) {
    case 'w':
    case 'W': cmd_vel_.linear.x = linear_speed_; break;
    case 's':
    case 'S': cmd_vel_.linear.x = -linear_speed_; break;
    case 'a':
    case 'A': cmd_vel_.linear.y = -linear_speed_; break;
    case 'd':
    case 'D': cmd_vel_.linear.y = linear_speed_; break;
    case ' ': cmd_vel_.linear.z = -vertical_speed_; break;
    case 'c':
    case 'C':
    case 'x':
    case 'X': cmd_vel_.linear.z = vertical_speed_; break;
    case 'j':
    case 'J':
    case 'q':
    case 'Q': cmd_vel_.angular.z = -yaw_rate_; break;
    case 'l':
    case 'L':
    case 'e':
    case 'E': cmd_vel_.angular.z = yaw_rate_; break;
    default: return false;
    }
    return true;
}

bool KeyboardInput::kbhit() {
    timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInput>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
