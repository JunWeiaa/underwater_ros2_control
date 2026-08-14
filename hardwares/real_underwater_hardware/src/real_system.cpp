#include "real_underwater_hardware/real_system.hpp"

#include "real_underwater_hardware/hardware_param_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <utility>

namespace real_underwater_hardware {
namespace {

using hardware_param_utils::normalize_namespace;
using hardware_param_utils::param_bool;
using hardware_param_utils::param_double;
using hardware_param_utils::param_double_vector;
using hardware_param_utils::param_int;
using hardware_param_utils::param_int_vector;
using hardware_param_utils::param_string;
using hardware_param_utils::param_string_vector;
using hardware_param_utils::resize_with_defaults;
using hardware_param_utils::to_lower;
using hardware_param_utils::topic_in_namespace;

// MAVROS OverrideRCIn: CHAN_RELEASE (0) releases RC override, while
// CHAN_NOCHANGE (65535) leaves the channel untouched. Stopping thrusters uses
// neutral PWM on mapped channels instead of releasing them to a possibly live RC input.
constexpr uint16_t kRcNoChange = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
constexpr double kDefaultPwm = 1500.0;
constexpr uint16_t kMavCmdSetMessageInterval = 511;
constexpr uint16_t kMavlinkMsgHighresImu = 105;
constexpr uint16_t kMavlinkMsgAttitudeQuaternion = 31;
constexpr uint16_t kMavlinkMsgLocalPositionNed = 32;

uint16_t clamp_pwm(double pwm, const std::string &model) {
    if (model == "subcat") {
        return static_cast<uint16_t>(std::clamp(std::round(pwm), 1000.0, 2000.0));
    }
    return static_cast<uint16_t>(std::clamp(std::round(pwm), 1100.0, 1900.0));
}

template <size_t Size>
double evaluate_polynomial(const std::array<double, Size> &coefficients, double x) {
    static_assert(Size > 0);
    double value = coefficients[0];
    for (size_t i = 1; i < Size; ++i) {
        value = value * x + coefficients[i];
    }
    return value;
}

double bluerov2_thrust_to_pwm(double x) {
    if (x < -0.3629) {
        constexpr std::array<double, 9> kReverseCoefficients = {
            4.19120215e-09,
            7.25765393e-07,
            5.12089798e-05,
            1.89821056e-03,
            3.97888906e-02,
            4.76560866e-01,
            3.31574457e+00,
            2.41195184e+01,
            1.47508532e+03,
        };
        return evaluate_polynomial(kReverseCoefficients, x);
    }

    if (x >= 0.4082) {
        constexpr std::array<double, 9> kForwardCoefficients = {
            -3.89443834e-10,
            8.97208792e-08,
            8.42626420e-06,
            4.16470611e-04,
            1.16830379e-02,
            1.88609133e-01,
            1.79496731e+00,
            1.83042688e+01,
            1.52666998e+03,
        };
        return evaluate_polynomial(kForwardCoefficients, x);
    }

    return kDefaultPwm;
}

double subcat_thrust_to_pwm(double x) {
    if (x < -0.3) {
        constexpr std::array<double, 7> kReverseCoefficients = {
            3.78328199e-08,
            1.24896612e-05,
            1.24135724e-03,
            5.29618237e-02,
            1.05292003e+00,
            1.69210385e+01,
            1.48493125e+03,
        };
        return evaluate_polynomial(kReverseCoefficients, x);
    }

    if (x > 0.3) {
        constexpr std::array<double, 7> kForwardCoefficients = {
            3.07095665e-08,
            -6.55375382e-06,
            5.30480570e-04,
            -1.94854716e-02,
            2.80240130e-01,
            5.06028546e+00,
            1.52849853e+03,
        };
        return evaluate_polynomial(kForwardCoefficients, x);
    }

    return kDefaultPwm;
}

void normalize_quaternion(ImuData &imu) {
    const double norm = std::sqrt(
        imu.orientation_w * imu.orientation_w +
        imu.orientation_x * imu.orientation_x +
        imu.orientation_y * imu.orientation_y +
        imu.orientation_z * imu.orientation_z);
    if (norm > 1.0e-9) {
        imu.orientation_w /= norm;
        imu.orientation_x /= norm;
        imu.orientation_y /= norm;
        imu.orientation_z /= norm;
    } else {
        imu.orientation_w = 1.0;
        imu.orientation_x = 0.0;
        imu.orientation_y = 0.0;
        imu.orientation_z = 0.0;
    }
}

} // namespace

hardware_interface::CallbackReturn
RealSystem::on_init(const hardware_interface::HardwareInfo &system_info) {
    if (hardware_interface::SystemInterface::on_init(system_info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto &kv : info_.hardware_parameters) {
        RCLCPP_INFO(rclcpp::get_logger("RealSystem"),
                    "hardware_parameter: %s = %s",
                    kv.first.c_str(),
                    kv.second.c_str());
    }

    configure_command_metadata();
    configure_hardware_parameters();
    configure_thrust_channels();
    configure_servo_parameters();

    hw_commands_.assign(command_infos_.size(), std::numeric_limits<double>::quiet_NaN());
    joint_states_.assign(info_.joints.size(), JointStateData{});
    imu_.clear();

    RCLCPP_INFO(rclcpp::get_logger("RealSystem"),
                "RealSystem '%s': mavros_namespace=%s commands=%zu thrust=%zu position=%zu",
                robot_name_.c_str(),
                mavros_namespace_.c_str(),
                command_infos_.size(),
                thrust_command_indices_.size(),
                position_command_indices_.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

void RealSystem::configure_command_metadata() {
    command_infos_.clear();
    thrust_command_indices_.clear();
    position_command_indices_.clear();

    size_t command_index = 0;
    for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
        const auto &joint = info_.joints[joint_index];
        for (const auto &interface : joint.command_interfaces) {
            CommandInfo command_info;
            command_info.joint_index = joint_index;
            command_info.joint_name = joint.name;
            command_info.interface_name = interface.name;
            command_info.command_index = command_index;

            if (interface.name == "thrust") {
                thrust_command_indices_.push_back(command_index);
            } else if (interface.name == hardware_interface::HW_IF_POSITION) {
                position_command_indices_.push_back(command_index);
            }

            command_infos_.push_back(std::move(command_info));
            ++command_index;
        }
    }
}

void RealSystem::configure_hardware_parameters() {
    const auto &params = info_.hardware_parameters;
    robot_name_ = param_string(params, "robot_name", info_.name.empty() ? "underwater_robot" : info_.name);
    mavros_namespace_ = normalize_namespace(param_string(params, "mavros_namespace", "/mavros"));
    thrust_pwm_model_ = to_lower(param_string(params, "thrust_pwm_model", "bluerov2"));
    thrust_pwm_per_newton_ = param_double(params, "thrust_pwm_per_newton", 20.0);
}

void RealSystem::configure_thrust_channels() {
    const auto thrust_count = thrust_command_indices_.size();
    thrust_channels_.clear();
    thrust_channels_.reserve(thrust_count);
    for (size_t i = 0; i < thrust_count; ++i) {
        thrust_channels_.push_back(static_cast<int>(i + 1));
    }

    thrust_channels_ =
        param_int_vector(info_.hardware_parameters, "thrust_channels", thrust_channels_);
    thrust_channel_scales_ =
        param_double_vector(info_.hardware_parameters,
                            "thrust_channel_scales",
                            std::vector<double>(thrust_count, 1.0));

    resize_with_defaults(thrust_channels_, thrust_count, 0);
    resize_with_defaults(thrust_channel_scales_, thrust_count, 1.0);

    for (size_t i = 0; i < thrust_channels_.size(); ++i) {
        if (thrust_channels_[i] < 1 || thrust_channels_[i] > 18) {
            RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                        "Invalid thrust channel %d at index %zu; releasing this command",
                        thrust_channels_[i],
                        i);
            thrust_channels_[i] = 0;
        }
    }
}

void RealSystem::configure_servo_parameters() {
    const auto &params = info_.hardware_parameters;
    servo_count_ = param_int(
        params,
        "servo_count",
        static_cast<int>(position_command_indices_.size()));
    servo_count_ = std::max(0, servo_count_);
    servo_enabled_ = param_bool(params, "servo_enabled", servo_count_ > 0);

    servo_timeout_ms_ = param_int(params, "servo_timeout_ms", 100);
    servo_duration_ms_ = param_int(params, "servo_duration_ms", 10);

    servo_ports_.clear();
    servo_ids_.clear();
    servo_baudrates_.clear();
    for (int i = 0; i < servo_count_; ++i) {
        servo_ports_.push_back(param_string(
            params,
            "servo" + std::to_string(i) + "_port",
            "/dev/ttyACM" + std::to_string(i)));
        servo_ids_.push_back(param_int(params, "servo" + std::to_string(i) + "_id", 0));
        servo_baudrates_.push_back(param_int(
            params,
            "servo" + std::to_string(i) + "_baudrate",
            115200));
    }

    servo_ports_ = param_string_vector(params, "servo_ports", servo_ports_);
    servo_ids_ = param_int_vector(params, "servo_ids", servo_ids_);
    servo_baudrates_ = param_int_vector(params, "servo_baudrates", servo_baudrates_);

    servo_command_map_.clear();
    servo_command_map_.reserve(position_command_indices_.size());
    for (size_t i = 0; i < position_command_indices_.size(); ++i) {
        servo_command_map_.push_back(static_cast<int>(i));
    }
    servo_command_map_ =
        param_int_vector(params, "servo_command_map", servo_command_map_);

    servo_offsets_ =
        param_double_vector(params,
                            "servo_offsets",
                            std::vector<double>(position_command_indices_.size(), 0.0));
    servo_scales_ =
        param_double_vector(params,
                            "servo_scales",
                            std::vector<double>(position_command_indices_.size(), 1.0));

    resize_with_defaults(servo_ports_, servo_count_, std::string{});
    resize_with_defaults(servo_ids_, servo_count_, 0);
    resize_with_defaults(servo_baudrates_, servo_count_, 115200);
    resize_with_defaults(servo_command_map_, position_command_indices_.size(), 0);
    resize_with_defaults(servo_offsets_, position_command_indices_.size(), 0.0);
    resize_with_defaults(servo_scales_, position_command_indices_.size(), 1.0);
}

hardware_interface::CallbackReturn
RealSystem::on_configure(const rclcpp_lifecycle::State &) {
    if (!configure_mavros_backend()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (servo_enabled_) {
        servo_controller_ = std::make_unique<ServoController>();
        for (int i = 0; i < servo_count_; ++i) {
            if (servo_ports_[i].empty()) {
                RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                            "Servo %d has an empty port; skipping",
                            i);
                continue;
            }

            try {
                if (servo_controller_->add_servo(
                        i,
                        servo_ports_[i],
                        static_cast<uint8_t>(servo_ids_[i]),
                        servo_baudrates_[i],
                        servo_timeout_ms_)) {
                    RCLCPP_INFO(rclcpp::get_logger("RealSystem"),
                                "Servo %d connected: port=%s id=%d baudrate=%d",
                                i,
                                servo_ports_[i].c_str(),
                                servo_ids_[i],
                                servo_baudrates_[i]);
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                                "Servo %d connection failed: %s",
                                i,
                                servo_ports_[i].c_str());
                }
            } catch (const ServoProtocolError &error) {
                RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                            "Servo %d initialization failed: %s",
                            i,
                            error.what());
            }
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

bool RealSystem::configure_mavros_backend() {
    const auto node_name = robot_name_ + "_real_hw_mavros";
    mavros_client_node_ = rclcpp::Node::make_shared(node_name);

    cmd_long_client_ =
        mavros_client_node_->create_client<mavros_msgs::srv::CommandLong>(
            topic_in_namespace(mavros_namespace_, "cmd/command"));

    auto rc_publisher =
        mavros_client_node_->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            topic_in_namespace(mavros_namespace_, "rc/override"),
            rclcpp::SystemDefaultsQoS());
    rt_rc_override_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(
            rc_publisher);

    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    imu_subscription_ =
        mavros_client_node_->create_subscription<sensor_msgs::msg::Imu>(
            topic_in_namespace(mavros_namespace_, "imu/data"),
            qos,
            std::bind(&RealSystem::imu_callback, this, std::placeholders::_1));

    if (!cmd_long_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                    "MAVROS command service '%s' is not available; continuing without stream configuration",
                    topic_in_namespace(mavros_namespace_, "cmd/command").c_str());
        return true;
    }

    configure_mavros_streams();
    return true;
}

void RealSystem::configure_mavros_streams() {
    auto send_message_interval = [this](uint16_t message_id, float interval_us) {
        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->broadcast = false;
        request->command = kMavCmdSetMessageInterval;
        request->confirmation = 0;
        request->param1 = static_cast<float>(message_id);
        request->param2 = interval_us;
        request->param3 = 0.0F;
        request->param4 = 0.0F;
        request->param5 = 0.0F;
        request->param6 = 0.0F;
        request->param7 = 0.0F;

        auto future = cmd_long_client_->async_send_request(request);
        const auto result = rclcpp::spin_until_future_complete(
            mavros_client_node_,
            future,
            std::chrono::seconds(2));
        if (result != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                        "MAVROS message interval command timed out for msgid=%u",
                        message_id);
            return;
        }
        const auto response = future.get();
        if (!response->success) {
            RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                        "MAVROS message interval command failed for msgid=%u result=%d",
                        message_id,
                        response->result);
        }
    };

    send_message_interval(kMavlinkMsgHighresImu, 5000.0F);
    send_message_interval(kMavlinkMsgAttitudeQuaternion, 5000.0F);
    send_message_interval(kMavlinkMsgLocalPositionNed, 10000.0F);

    RCLCPP_INFO(rclcpp::get_logger("RealSystem"),
                "Requested MAVROS HIGHRES_IMU/ATTITUDE_QUATERNION at 200 Hz and LOCAL_POSITION_NED at 100 Hz");
}

hardware_interface::CallbackReturn
RealSystem::on_cleanup(const rclcpp_lifecycle::State &) {
    if (servo_controller_) {
        servo_controller_->close_all();
        servo_controller_.reset();
    }
    rt_rc_override_publisher_.reset();
    imu_subscription_.reset();
    cmd_long_client_.reset();
    mavros_client_node_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RealSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back("imu_sensor", "orientation.w", &imu_.orientation_w);
    state_interfaces.emplace_back("imu_sensor", "orientation.x", &imu_.orientation_x);
    state_interfaces.emplace_back("imu_sensor", "orientation.y", &imu_.orientation_y);
    state_interfaces.emplace_back("imu_sensor", "orientation.z", &imu_.orientation_z);
    state_interfaces.emplace_back("imu_sensor", "angular_velocity.x", &imu_.angular_velocity_x);
    state_interfaces.emplace_back("imu_sensor", "angular_velocity.y", &imu_.angular_velocity_y);
    state_interfaces.emplace_back("imu_sensor", "angular_velocity.z", &imu_.angular_velocity_z);
    state_interfaces.emplace_back("imu_sensor", "linear_acceleration.x", &imu_.linear_acceleration_x);
    state_interfaces.emplace_back("imu_sensor", "linear_acceleration.y", &imu_.linear_acceleration_y);
    state_interfaces.emplace_back("imu_sensor", "linear_acceleration.z", &imu_.linear_acceleration_z);

    for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
        const auto &joint = info_.joints[joint_index];
        auto &state = joint_states_[joint_index];
        for (const auto &interface : joint.state_interfaces) {
            if (interface.name == hardware_interface::HW_IF_POSITION) {
                state_interfaces.emplace_back(joint.name, interface.name, &state.position);
            } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
                state_interfaces.emplace_back(joint.name, interface.name, &state.velocity);
            } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
                state_interfaces.emplace_back(joint.name, interface.name, &state.effort);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                            "Unsupported state interface '%s' on joint '%s'",
                            interface.name.c_str(),
                            joint.name.c_str());
            }
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RealSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(command_infos_.size());

    for (const auto &command_info : command_infos_) {
        command_interfaces.emplace_back(
            command_info.joint_name,
            command_info.interface_name,
            &hw_commands_[command_info.command_index]);
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn
RealSystem::on_activate(const rclcpp_lifecycle::State &) {
    for (auto &cmd : hw_commands_) {
        if (std::isnan(cmd)) {
            cmd = 0.0;
        }
    }
    for (auto &state : joint_states_) {
        state = JointStateData{};
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RealSystem::on_deactivate(const rclcpp_lifecycle::State &) {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    send_rc_override(build_neutral_rc_override_channels());
    send_servo_positions();
    return hardware_interface::CallbackReturn::SUCCESS;
}

void RealSystem::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_.orientation_w = msg->orientation.w;
    imu_.orientation_x = msg->orientation.x;
    imu_.orientation_y = msg->orientation.y;
    imu_.orientation_z = msg->orientation.z;
    normalize_quaternion(imu_);

    imu_.angular_velocity_x = msg->angular_velocity.x;
    imu_.angular_velocity_y = msg->angular_velocity.y;
    imu_.angular_velocity_z = msg->angular_velocity.z;

    imu_.linear_acceleration_x = msg->linear_acceleration.x;
    imu_.linear_acceleration_y = msg->linear_acceleration.y;
    imu_.linear_acceleration_z = msg->linear_acceleration.z;
    imu_.timestamp = msg->header.stamp;
}

hardware_interface::return_type
RealSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
    if (mavros_client_node_) {
        rclcpp::spin_some(mavros_client_node_);
    }

    for (auto &state : joint_states_) {
        state.velocity = 0.0;
        state.effort = 0.0;
    }

    for (const auto command_index : thrust_command_indices_) {
        const auto &command_info = command_infos_[command_index];
        const double thrust = hw_commands_[command_index];
        auto &state = joint_states_[command_info.joint_index];
        state.velocity = std::isnan(thrust) ? 0.0 : thrust_to_vel(thrust);
        state.effort = std::isnan(thrust) ? 0.0 : thrust;
    }

    for (const auto command_index : position_command_indices_) {
        const auto &command_info = command_infos_[command_index];
        const double position = hw_commands_[command_index];
        auto &state = joint_states_[command_info.joint_index];
        state.position = std::isnan(position) ? 0.0 : position;
    }

    return hardware_interface::return_type::OK;
}

std::array<uint16_t, 18> RealSystem::build_runtime_rc_override_channels() const {
    std::array<uint16_t, 18> channels{};
    channels.fill(kRcNoChange);

    for (size_t i = 0; i < thrust_command_indices_.size(); ++i) {
        if (i >= thrust_channels_.size()) {
            continue;
        }
        const int channel = thrust_channels_[i];
        if (channel < 1 || channel > static_cast<int>(channels.size())) {
            continue;
        }

        const double command = hw_commands_[thrust_command_indices_[i]];
        if (std::isnan(command)) {
            continue;
        }

        const double raw_pwm = static_cast<double>(thrust_to_pwm(command));
        const double scale = i < thrust_channel_scales_.size() ? thrust_channel_scales_[i] : 1.0;
        const double scaled_pwm = kDefaultPwm + scale * (raw_pwm - kDefaultPwm);
        channels[static_cast<size_t>(channel - 1)] = clamp_pwm(scaled_pwm, thrust_pwm_model_);
    }

    return channels;
}

std::array<uint16_t, 18> RealSystem::build_neutral_rc_override_channels() const {
    std::array<uint16_t, 18> channels{};
    channels.fill(kRcNoChange);

    for (const int channel : thrust_channels_) {
        if (channel < 1 || channel > static_cast<int>(channels.size())) {
            continue;
        }
        channels[static_cast<size_t>(channel - 1)] = clamp_pwm(kDefaultPwm, thrust_pwm_model_);
    }

    return channels;
}

void RealSystem::send_rc_override(const std::array<uint16_t, 18> &channels) {
    if (!rt_rc_override_publisher_ || !rt_rc_override_publisher_->trylock()) {
        return;
    }

    for (size_t i = 0; i < channels.size(); ++i) {
        rt_rc_override_publisher_->msg_.channels[i] = channels[i];
    }
    rt_rc_override_publisher_->unlockAndPublish();
}

void RealSystem::send_servo_positions() {
    if (!servo_controller_ || position_command_indices_.empty()) {
        return;
    }

    std::vector<double> angles(static_cast<size_t>(servo_count_), 0.0);
    bool has_position_command = false;

    for (size_t i = 0; i < position_command_indices_.size(); ++i) {
        const double command = hw_commands_[position_command_indices_[i]];
        if (std::isnan(command)) {
            continue;
        }

        const int servo_index =
            i < servo_command_map_.size() ? servo_command_map_[i] : static_cast<int>(i);
        if (servo_index < 0 || servo_index >= servo_count_) {
            RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                        "Servo command map index %d is out of range [0, %d)",
                        servo_index,
                        servo_count_);
            continue;
        }

        const double offset = i < servo_offsets_.size() ? servo_offsets_[i] : 0.0;
        const double scale = i < servo_scales_.size() ? servo_scales_[i] : 1.0;
        angles[static_cast<size_t>(servo_index)] = scale * (command + offset);
        has_position_command = true;
    }

    if (!has_position_command) {
        return;
    }

    try {
        servo_controller_->set_all_angles(angles, servo_duration_ms_);
    } catch (const ServoProtocolError &error) {
        static size_t error_count = 0;
        if ((error_count % 1000) == 0) {
            RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                        "Servo control error: %s",
                        error.what());
        }
        ++error_count;
    }
}

void RealSystem::send_actuator_control_target(const rclcpp::Time &) {
    send_rc_override(build_runtime_rc_override_channels());
    send_servo_positions();
}

hardware_interface::return_type
RealSystem::write(const rclcpp::Time &time, const rclcpp::Duration &) {
    send_actuator_control_target(time);
    return hardware_interface::return_type::OK;
}

double RealSystem::thrust_to_pwm(double x) const {
    double pwm = kDefaultPwm;
    if (thrust_pwm_model_ == "bluerov2" || thrust_pwm_model_ == "bluerov2_heavy") {
        pwm = bluerov2_thrust_to_pwm(x);
    } else if (thrust_pwm_model_ == "subcat") {
        pwm = subcat_thrust_to_pwm(x);
    } else {
        pwm = kDefaultPwm + thrust_pwm_per_newton_ * x;
    }

    return clamp_pwm(pwm, thrust_pwm_model_);
}

double RealSystem::thrust_to_vel(double x) const {
    if (x < -0.3629) {
        double velocity = -5.15240756e-09;
        velocity = velocity * x - 8.91478070e-07;
        velocity = velocity * x - 6.39245225e-05;
        velocity = velocity * x - 2.46391960e-03;
        velocity = velocity * x - 5.55230605e-02;
        velocity = velocity * x - 7.51311965e-01;
        velocity = velocity * x - 6.23455922e+00;
        velocity = velocity * x - 4.01245448e+01;
        velocity = velocity * x + 2.26612660e+01;
        return velocity;
    }
    if (x >= 0.4082) {
        double velocity = -5.51363691e-10;
        velocity = velocity * x + 1.26209789e-07;
        velocity = velocity * x - 1.19924118e-05;
        velocity = velocity * x + 6.13477892e-04;
        velocity = velocity * x - 1.83765021e-02;
        velocity = velocity * x + 3.31031438e-01;
        velocity = velocity * x - 3.65254191e+00;
        velocity = velocity * x + 3.09673112e+01;
        velocity = velocity * x + 2.41929975e+01;
        return velocity;
    }
    return 0.0;
}

} // namespace real_underwater_hardware

PLUGINLIB_EXPORT_CLASS(real_underwater_hardware::RealSystem,
                       hardware_interface::SystemInterface)
