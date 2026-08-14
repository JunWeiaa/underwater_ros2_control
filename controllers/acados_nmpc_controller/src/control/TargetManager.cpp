#include "acados_nmpc_controller/control/TargetManager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <string>
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

std::string lowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string trimCopy(const std::string &value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
        return std::isspace(c);
    });
    if (first == value.end()) {
        return {};
    }
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
        return std::isspace(c);
    }).base();
    return std::string(first, last);
}

std::string defaultPathFrame(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
    std::string node_namespace = node ? node->get_namespace() : "";
    node_namespace = trimCopy(node_namespace);
    while (!node_namespace.empty() && node_namespace.front() == '/') {
        node_namespace.erase(node_namespace.begin());
    }
    while (!node_namespace.empty() && node_namespace.back() == '/') {
        node_namespace.pop_back();
    }
    return node_namespace.empty() ? "odom" : node_namespace + "/odom";
}

bool isMonotonicTime(const vector_t &time) {
    for (int i = 1; i < time.size(); ++i) {
        if (time(i) < time(i - 1)) {
            return false;
        }
    }
    return true;
}
} // namespace

TargetManager::TargetManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node) :
    node_(node) {
    state_dim_ = std::max<int>(13, static_cast<int>(declareOrGet<int64_t>(node_, "state_dim", 13)));
    input_dim_ = std::max<int>(1, static_cast<int>(declareOrGet<int64_t>(node_, "input_dim", 8)));

    RCLCPP_INFO(node_->get_logger(), "TargetManager initialize start");
    currpath_pub_ = node_->create_publisher<nav_msgs::msg::Path>("curr_traj", 1);
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "trajectory",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    const std::string robot_pkg =
        declareOrGet<std::string>(node_, "robot_pkg", "");
    std::string default_target_file;
    if (!robot_pkg.empty()) {
        try {
            default_target_file =
                ament_index_cpp::get_package_share_directory(robot_pkg) + "/config/target.info";
        } catch (const std::exception &e) {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot resolve default target.info for robot_pkg '%s': %s",
                        robot_pkg.c_str(),
                        e.what());
        }
    }

    const std::string trajectory_source =
        declareOrGet<std::string>(node_, "trajectory.source", "file");
    target_file_ =
        declareOrGet<std::string>(node_, "trajectory.target_file", default_target_file);
    if (target_file_.empty()) {
        target_file_ = default_target_file;
    }
    target_name_ =
        declareOrGet<std::string>(node_, "trajectory.target_name", "");
    path_frame_ =
        declareOrGet<std::string>(node_, "trajectory.path_frame", defaultPathFrame(node_));
    if (path_frame_.empty()) {
        path_frame_ = defaultPathFrame(node_);
    }
    dt_ = declareOrGet<double>(node_, "trajectory.dt", 0.001);
    dt_p = declareOrGet<double>(node_, "trajectory.prediction_dt", 0.025);
    horizon_steps_ = std::max<int>(1, static_cast<int>(declareOrGet<int64_t>(node_, "trajectory.horizon_steps", 20)));
    k_p = std::max(1, static_cast<int>(std::round(dt_p / dt_)));
    const double path_publish_dt =
        declareOrGet<double>(node_, "trajectory.path_publish_dt", dt_p);
    path_publish_step_ = path_publish_dt > 0.0
                             ? std::max(1, static_cast<int>(std::round(path_publish_dt / dt_)))
                             : k_p;
    curr_path_publish_rate_ =
        declareOrGet<double>(node_, "trajectory.curr_path_publish_rate", 20.0);
    if (curr_path_publish_rate_ < 0.0) {
        RCLCPP_WARN(node_->get_logger(),
                    "trajectory.curr_path_publish_rate %.3f is negative; disabling curr_traj publish",
                    curr_path_publish_rate_);
        curr_path_publish_rate_ = 0.0;
    }
    initializeTrajectoryWindows();

    const bool enable_topic_targets =
        declareOrGet<bool>(node_, "trajectory.enable_topic_targets", true);
    const std::string bspline_topic =
        declareOrGet<std::string>(node_, "trajectory.bspline_topic", "target_bspline");
    bspline_degree_ = std::max<int>(1, static_cast<int>(declareOrGet<int64_t>(node_, "trajectory.bspline_degree", 3)));
    bspline_duration_ = declareOrGet<double>(node_, "trajectory.bspline_duration", 0.0);
    bspline_frame_ = declareOrGet<std::string>(node_, "trajectory.bspline_frame", "enu");

    if (enable_topic_targets) {
        target_bspline_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            bspline_topic, 1,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                targetBsplineCallback(msg);
            });
        RCLCPP_INFO(node_->get_logger(),
                    "TargetManager listening for B-spline target on '%s'",
                    bspline_topic.c_str());
    }

    initializeConfiguredTrajectory(trajectory_source);
}

TrajectoryGenerator TargetManager::makeTrajectoryGenerator() const {
    TrajectoryGeneratorConfig config;
    config.state_dim = state_dim_;
    config.input_dim = input_dim_;
    config.horizon_steps = horizon_steps_;
    config.prediction_step_ratio = k_p;
    config.dt = dt_;
    config.prediction_dt = dt_p;
    return TrajectoryGenerator(config);
}

std::shared_ptr<TargetTrajectories> TargetManager::makeTrajectoryWindow() const {
    vector_t initial_state = vector_t::Zero(state_dim_);
    if (initial_state.size() >= 7) {
        initial_state(3) = 1.0;
    }

    return std::make_shared<TargetTrajectories>(
        vector_t::Zero(horizon_steps_),
        vector_array_t(static_cast<size_t>(horizon_steps_), initial_state),
        vector_array_t(static_cast<size_t>(horizon_steps_), vector_t::Zero(input_dim_)));
}

void TargetManager::initializeTrajectoryWindows() {
    trajectory_windows_[0] = makeTrajectoryWindow();
    trajectory_windows_[1] = makeTrajectoryWindow();
    active_window_index_ = 0;
    trajectory_buffer_.store(trajectory_windows_[active_window_index_]);

    current_path_msg_.header.frame_id = path_frame_;
    current_path_msg_.poses.clear();
    current_path_msg_.poses.reserve(static_cast<size_t>(horizon_steps_));
    last_curr_path_publish_time_ns_ = 0;
}

bool TargetManager::shouldPublishCurrentPath(const rclcpp::Time &stamp) {
    if (!currpath_pub_ || curr_path_publish_rate_ <= 0.0) {
        return false;
    }

    const int64_t now_ns = stamp.nanoseconds();
    const auto publish_period_ns =
        static_cast<int64_t>(std::llround(1e9 / curr_path_publish_rate_));
    if (last_curr_path_publish_time_ns_ == 0 ||
        now_ns < last_curr_path_publish_time_ns_ ||
        now_ns - last_curr_path_publish_time_ns_ >= publish_period_ns) {
        last_curr_path_publish_time_ns_ = now_ns;
        return true;
    }
    return false;
}

void TargetManager::initializeConfiguredTrajectory(const std::string &trajectory_source) {
    const std::string source = lowerCopy(trajectory_source);
    if (source == "topic" || source == "none") {
        RCLCPP_INFO(node_->get_logger(),
                    "TargetManager waiting for B-spline target topic input");
        return;
    }

    if (source == "file") {
        if (!loadTargetInfo(target_file_, target_name_)) {
            RCLCPP_WARN(node_->get_logger(),
                        "No valid target.info trajectory loaded; waiting for B-spline target topic input");
        }
        return;
    }

    RCLCPP_WARN(node_->get_logger(),
                "Unsupported trajectory.source '%s'. Use 'file', 'topic', or 'none'.",
                trajectory_source.c_str());
}

bool TargetManager::loadTargetInfo(const std::string &target_file,
                                   const std::string &target_name) {
    std::string error;
    std::vector<std::string> warnings;
    const auto definition =
        target_info_loader_.load(target_file, target_name, &error, &warnings);
    for (const auto &warning : warnings) {
        RCLCPP_WARN(node_->get_logger(), "%s", warning.c_str());
    }
    if (!definition) {
        RCLCPP_WARN(node_->get_logger(), "%s", error.c_str());
        return false;
    }

    const auto generator = makeTrajectoryGenerator();
    std::optional<TargetTrajectories> trajectory;
    error.clear();

    if (definition->type == "circle") {
        trajectory = generator.buildCircle(definition->radius,
                                           definition->center,
                                           definition->duration,
                                           definition->start_angle,
                                           definition->frame,
                                           &error);
    } else if (definition->type == "figure8" || definition->type == "figure_8") {
        trajectory = generator.buildFigure8(definition->x_amplitude,
                                            definition->y_amplitude,
                                            definition->z_amplitude,
                                            definition->center,
                                            definition->duration,
                                            definition->start_phase,
                                            definition->frame,
                                            &error);
    } else if (definition->type == "points" || definition->type == "trajectory") {
        trajectory = generator.buildPoints(definition->states,
                                           definition->inputs,
                                           definition->time,
                                           definition->frame,
                                           &error);
    } else {
        RCLCPP_WARN(node_->get_logger(),
                    "Unsupported target '%s' type '%s' in '%s'",
                    definition->selected_target.c_str(),
                    definition->type.c_str(),
                    target_file.c_str());
        return false;
    }

    if (!trajectory) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected target trajectory from %s: %s",
                    definition->source_label.c_str(),
                    error.c_str());
        return false;
    }

    return setFullTrajectory(*trajectory, definition->source_label);
}

bool TargetManager::setFullTrajectory(const TargetTrajectories &trajectory,
                                      const std::string &source_label) {
    const auto &time = trajectory.time();
    const auto &states = trajectory.state();
    const auto &inputs = trajectory.input();

    if (states.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Rejected empty target trajectory from %s", source_label.c_str());
        return false;
    }
    if (time.size() != static_cast<int>(states.size()) || inputs.size() != states.size()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected target trajectory from %s: time/state/input sizes are %ld/%zu/%zu",
                    source_label.c_str(),
                    time.size(),
                    states.size(),
                    inputs.size());
        return false;
    }
    if (!isMonotonicTime(time)) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected target trajectory from %s: trajectory.time must be monotonic",
                    source_label.c_str());
        return false;
    }

    vector_array_t normalized_states;
    vector_array_t normalized_inputs;
    normalized_states.reserve(states.size());
    normalized_inputs.reserve(inputs.size());
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i].size() != state_dim_) {
            RCLCPP_WARN(node_->get_logger(),
                        "Rejected target trajectory from %s: state[%zu] size %ld != state_dim %d",
                        source_label.c_str(),
                        i,
                        states[i].size(),
                        state_dim_);
            return false;
        }
        if (inputs[i].size() != input_dim_) {
            RCLCPP_WARN(node_->get_logger(),
                        "Rejected target trajectory from %s: input[%zu] size %ld != input_dim %d",
                        source_label.c_str(),
                        i,
                        inputs[i].size(),
                        input_dim_);
            return false;
        }

        vector_t state = states[i];
        TrajectoryGenerator::normalizeQuaternionOrIdentity(state);
        TrajectoryGenerator::keepQuaternionContinuous(state, normalized_states);
        normalized_states.push_back(state);
        normalized_inputs.push_back(inputs[i]);
    }

    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    full_time_ = time;
    full_trajectory_ = std::move(normalized_states);
    full_inputs_ = std::move(normalized_inputs);
    is_trajectory_updated_ = true;
    is_traj_track_started_ = false;
    current_index_ = 0;
    curr_time = node_->now().nanoseconds();
    curr_duration = 0;
    last_curr_path_publish_time_ns_ = 0;

    RCLCPP_INFO(node_->get_logger(),
                "TargetManager loaded target trajectory from %s with %zu points",
                source_label.c_str(),
                full_trajectory_.size());
    return true;
}

void TargetManager::targetBsplineCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::string error;
    const auto request = bspline_source_.parseMessage(*msg,
                                                      bspline_degree_,
                                                      bspline_duration_,
                                                      bspline_frame_,
                                                      &error);
    if (!request) {
        RCLCPP_WARN(node_->get_logger(),
                    "Ignored target_bspline: %s",
                    error.c_str());
        return;
    }

    const auto trajectory = makeTrajectoryGenerator().buildBspline(request->control_points,
                                                                   request->degree,
                                                                   request->duration,
                                                                   request->frame,
                                                                   &error);
    if (!trajectory) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected B-spline from target_bspline topic: %s",
                    error.c_str());
        return;
    }

    setFullTrajectory(*trajectory, "target_bspline topic");
}

void TargetManager::publishFullTrajectory() const {
    if (!path_pub_) {
        return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = path_frame_;
    path.header.stamp = node_->now();
    path.poses.reserve(full_trajectory_.size() / static_cast<size_t>(path_publish_step_) + 1);
    size_t last_published_index = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < full_trajectory_.size(); i += static_cast<size_t>(path_publish_step_)) {
        geometry_msgs::msg::PoseStamped pose;
        TrajectoryGenerator::fillPoseStampedFromState(full_trajectory_[i], path.header.stamp, path_frame_, pose);
        path.poses.push_back(pose);
        last_published_index = i;
    }
    if (last_published_index != full_trajectory_.size() - 1) {
        geometry_msgs::msg::PoseStamped pose;
        TrajectoryGenerator::fillPoseStampedFromState(full_trajectory_.back(), path.header.stamp, path_frame_, pose);
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
}

void TargetManager::publishEmptyPaths() const {
    const auto stamp = node_->now();
    if (path_pub_) {
        nav_msgs::msg::Path path;
        path.header.frame_id = path_frame_;
        path.header.stamp = stamp;
        path_pub_->publish(path);
    }
    if (currpath_pub_) {
        nav_msgs::msg::Path path;
        path.header.frame_id = path_frame_;
        path.header.stamp = stamp;
        currpath_pub_->publish(path);
    }
}

void TargetManager::updateTrajectoryBuffer() {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (full_trajectory_.empty()) {
        return;
    }
    if (!trajectory_windows_[0] || !trajectory_windows_[1]) {
        initializeTrajectoryWindows();
    }

    const rclcpp::Time stamp = node_->now();
    if (!is_trajectory_updated_ && !is_traj_track_started_) {
        return;
    } else if (is_trajectory_updated_) {
        is_traj_track_started_ = true;
        is_trajectory_updated_ = false;
        curr_time = stamp.nanoseconds();
        curr_duration = 0;
        current_index_ = 0;
        publishFullTrajectory();
    } else {
        curr_duration = stamp.nanoseconds() - curr_time;
    }
    current_index_ = static_cast<int>(
        static_cast<double>(curr_duration) / (dt_ * 1e9));
    if (current_index_ >= static_cast<int>(full_trajectory_.size())) {
        current_index_ = static_cast<int>(full_trajectory_.size()) - 1;
    }

    const size_t write_window_index = 1U - active_window_index_;
    auto window = trajectory_windows_[write_window_index];
    auto &state_trajectory = window->state();
    auto &input_trajectory = window->input();
    auto &time_trajectory = window->time();
    const bool publish_current_path = shouldPublishCurrentPath(stamp);

    if (publish_current_path) {
        current_path_msg_.header.frame_id = path_frame_;
        current_path_msg_.header.stamp = stamp;
        current_path_msg_.poses.clear();
    }

    for (int i = 0; i < horizon_steps_; ++i) {
        const int index = current_index_ + i * k_p;
        if (index < static_cast<int>(full_trajectory_.size())) {
            state_trajectory[static_cast<size_t>(i)] = full_trajectory_[static_cast<size_t>(index)];
            input_trajectory[static_cast<size_t>(i)] = full_inputs_[static_cast<size_t>(index)];
            time_trajectory(i) = full_time_(index);

            if (publish_current_path) {
                current_path_msg_.poses.emplace_back();
                TrajectoryGenerator::fillPoseStampedFromState(full_trajectory_[static_cast<size_t>(index)],
                                                              stamp,
                                                              path_frame_,
                                                              current_path_msg_.poses.back());
            }
        } else {
            state_trajectory[static_cast<size_t>(i)] = full_trajectory_.back();
            input_trajectory[static_cast<size_t>(i)] = full_inputs_.back();
            time_trajectory(i) = full_time_(full_time_.size() - 1);
        }
    }

    trajectory_buffer_.store(window);
    active_window_index_ = write_window_index;
    if (publish_current_path) {
        currpath_pub_->publish(current_path_msg_);
    }
}

TargetTrajectories TargetManager::getCurrentTrajectorySegment() {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    auto buffer = trajectory_buffer_.load();
    return *buffer;
}
