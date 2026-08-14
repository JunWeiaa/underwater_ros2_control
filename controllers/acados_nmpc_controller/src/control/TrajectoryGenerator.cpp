#include "acados_nmpc_controller/control/TrajectoryGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <sstream>
#include <utility>

namespace {
void assignError(std::string *error, const std::string &message) {
    if (error != nullptr) {
        *error = message;
    }
}

std::string lowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

void setIdentityQuaternion(vector_t &state) {
    if (state.size() >= 7) {
        state(3) = 1.0;
        state(4) = 0.0;
        state(5) = 0.0;
        state(6) = 0.0;
    }
}

vector_t toEigenVector(const std::vector<double> &values) {
    vector_t result(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        result(static_cast<long>(i)) = values[i];
    }
    return result;
}

std::vector<double> makeClampedUniformKnots(size_t control_point_count,
                                            int degree,
                                            double duration) {
    const size_t knot_count = control_point_count + static_cast<size_t>(degree) + 1;
    std::vector<double> knots(knot_count, 0.0);
    const size_t last = knot_count - 1;

    for (int i = 0; i <= degree; ++i) {
        knots[static_cast<size_t>(i)] = 0.0;
        knots[last - static_cast<size_t>(i)] = duration;
    }

    const size_t interior_count = control_point_count - static_cast<size_t>(degree) - 1;
    for (size_t i = 1; i <= interior_count; ++i) {
        knots[static_cast<size_t>(degree) + i] =
            duration * static_cast<double>(i) / static_cast<double>(interior_count + 1);
    }
    return knots;
}

int findKnotSpan(const std::vector<double> &knots,
                 int degree,
                 size_t control_point_count,
                 double t) {
    const int n = static_cast<int>(control_point_count) - 1;
    if (t >= knots[static_cast<size_t>(n + 1)]) {
        return n;
    }
    if (t <= knots[static_cast<size_t>(degree)]) {
        return degree;
    }

    int low = degree;
    int high = n + 1;
    int mid = (low + high) / 2;
    while (t < knots[static_cast<size_t>(mid)] ||
           t >= knots[static_cast<size_t>(mid + 1)]) {
        if (t < knots[static_cast<size_t>(mid)]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    return mid;
}

vector_t evaluateBspline(const vector_array_t &control_points,
                         const std::vector<double> &knots,
                         int degree,
                         double t) {
    const int span = findKnotSpan(knots, degree, control_points.size(), t);
    std::vector<vector_t> d(static_cast<size_t>(degree) + 1);
    for (int j = 0; j <= degree; ++j) {
        d[static_cast<size_t>(j)] =
            control_points[static_cast<size_t>(span - degree + j)];
    }

    for (int r = 1; r <= degree; ++r) {
        for (int j = degree; j >= r; --j) {
            const int knot_left_index = span - degree + j;
            const int knot_right_index = span + 1 + j - r;
            const double denominator =
                knots[static_cast<size_t>(knot_right_index)] -
                knots[static_cast<size_t>(knot_left_index)];
            const double alpha =
                std::abs(denominator) < std::numeric_limits<double>::epsilon()
                    ? 0.0
                    : (t - knots[static_cast<size_t>(knot_left_index)]) / denominator;
            d[static_cast<size_t>(j)] =
                (1.0 - alpha) * d[static_cast<size_t>(j - 1)] +
                alpha * d[static_cast<size_t>(j)];
        }
    }

    return d[static_cast<size_t>(degree)];
}

vector3_t vector3ToNed(const vector3_t &value, const std::string &frame) {
    if (lowerCopy(frame) == "ned") {
        return value;
    }
    return vector3_t(value(1), value(0), -value(2));
}

vector3_t positionToNed(const vector_t &point, const std::string &frame) {
    return vector3ToNed(vector3_t(point(0), point(1), point(2)), frame);
}

quaternion_t enuFluToNedFrd(const quaternion_t &q_enu_flu) {
    const double inv_sqrt2 = std::sqrt(0.5);
    const quaternion_t q_enu_to_ned(0.0, inv_sqrt2, inv_sqrt2, 0.0);
    const quaternion_t q_flu_to_frd(0.0, 1.0, 0.0, 0.0);
    quaternion_t q_ned_frd = q_enu_to_ned * q_enu_flu * q_flu_to_frd;
    q_ned_frd.normalize();
    if (q_ned_frd.w() < 0.0) {
        q_ned_frd.coeffs() = -q_ned_frd.coeffs();
    }
    return q_ned_frd;
}

quaternion_t nedFrdToEnuFlu(const quaternion_t &q_ned_frd) {
    const double inv_sqrt2 = std::sqrt(0.5);
    const quaternion_t q_enu_to_ned(0.0, inv_sqrt2, inv_sqrt2, 0.0);
    const quaternion_t q_flu_to_frd(0.0, 1.0, 0.0, 0.0);
    quaternion_t q_enu_flu = q_enu_to_ned.conjugate() * q_ned_frd * q_flu_to_frd.conjugate();
    q_enu_flu.normalize();
    return q_enu_flu;
}

quaternion_t orientationFromNedVelocity(const vector3_t &velocity,
                                        const vector_array_t &states) {
    const double speed_xy = std::hypot(velocity(0), velocity(1));
    const double speed = velocity.norm();
    if (speed < 1e-6) {
        if (!states.empty() && states.back().size() >= 7) {
            quaternion_t previous(states.back()(3),
                                  states.back()(4),
                                  states.back()(5),
                                  states.back()(6));
            if (previous.norm() > std::numeric_limits<double>::epsilon()) {
                previous.normalize();
                return previous;
            }
        }
        return quaternion_t::Identity();
    }

    const double yaw = std::atan2(velocity(1), velocity(0));
    const double pitch = std::atan2(-velocity(2), speed_xy);
    quaternion_t q =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    q.normalize();
    return q;
}
} // namespace

TrajectoryGenerator::TrajectoryGenerator(TrajectoryGeneratorConfig config) :
    config_(config) {
}

int TrajectoryGenerator::totalStepCount(double duration) const {
    const int num_steps = std::max(1, static_cast<int>(std::ceil(duration / config_.dt)));
    return num_steps + config_.prediction_step_ratio * config_.horizon_steps;
}

void TrajectoryGenerator::normalizeQuaternionOrIdentity(vector_t &state) {
    if (state.size() < 7) {
        return;
    }
    quaternion_t q(state(3), state(4), state(5), state(6));
    if (q.norm() < std::numeric_limits<double>::epsilon()) {
        setIdentityQuaternion(state);
        return;
    }
    q.normalize();
    state(3) = q.w();
    state(4) = q.x();
    state(5) = q.y();
    state(6) = q.z();
}

void TrajectoryGenerator::keepQuaternionContinuous(vector_t &state,
                                                   const vector_array_t &trajectory) {
    if (state.size() < 7 || trajectory.empty() || trajectory.back().size() < 7) {
        return;
    }
    Eigen::Quaterniond q(state(3), state(4), state(5), state(6));
    const auto &previous = trajectory.back();
    Eigen::Quaterniond q_previous(previous(3), previous(4), previous(5), previous(6));
    if (q.dot(q_previous) < 0.0) {
        state(3) = -state(3);
        state(4) = -state(4);
        state(5) = -state(5);
        state(6) = -state(6);
    }
}

vector_t TrajectoryGenerator::stateToControllerFrame(const vector_t &source,
                                                     const std::string &frame) {
    vector_t state = source;
    if (lowerCopy(frame) == "ned") {
        normalizeQuaternionOrIdentity(state);
        return state;
    }

    if (state.size() >= 3) {
        const vector3_t position = positionToNed(state, "enu");
        state.segment<3>(0) = position;
    }
    if (state.size() >= 7) {
        quaternion_t q_enu_flu(state(3), state(4), state(5), state(6));
        if (q_enu_flu.norm() < std::numeric_limits<double>::epsilon()) {
            q_enu_flu = quaternion_t::Identity();
        } else {
            q_enu_flu.normalize();
        }
        const quaternion_t q_ned_frd = enuFluToNedFrd(q_enu_flu);
        state(3) = q_ned_frd.w();
        state(4) = q_ned_frd.x();
        state(5) = q_ned_frd.y();
        state(6) = q_ned_frd.z();
    }
    if (state.size() >= 10) {
        state(7) = source(7);
        state(8) = -source(8);
        state(9) = -source(9);
    }
    if (state.size() >= 13) {
        state(10) = source(10);
        state(11) = -source(11);
        state(12) = -source(12);
    }
    return state;
}

void TrajectoryGenerator::fillPoseStampedFromState(const vector_t &state,
                                                   const rclcpp::Time &stamp,
                                                   const std::string &frame_id,
                                                   geometry_msgs::msg::PoseStamped &pose) {
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    if (state.size() >= 3) {
        pose.pose.position.x = state(1);
        pose.pose.position.y = state(0);
        pose.pose.position.z = -state(2);
    }
    if (state.size() >= 7) {
        quaternion_t q_ned_frd(state(3), state(4), state(5), state(6));
        if (q_ned_frd.norm() < std::numeric_limits<double>::epsilon()) {
            q_ned_frd = quaternion_t::Identity();
        } else {
            q_ned_frd.normalize();
        }
        const quaternion_t q_enu_flu = nedFrdToEnuFlu(q_ned_frd);
        pose.pose.orientation.w = q_enu_flu.w();
        pose.pose.orientation.x = q_enu_flu.x();
        pose.pose.orientation.y = q_enu_flu.y();
        pose.pose.orientation.z = q_enu_flu.z();
    } else {
        pose.pose.orientation.w = 1.0;
    }
}

std::optional<TargetTrajectories> TrajectoryGenerator::buildCircle(
    double radius,
    const vector3_t &center,
    double duration,
    double start_angle,
    const std::string &frame,
    std::string *error) const {
    if (radius <= 0.0) {
        assignError(error, "radius must be positive");
        return std::nullopt;
    }
    if (duration <= 0.0) {
        duration = std::max(config_.dt, config_.prediction_dt);
    }

    constexpr double two_pi = 6.28318530717958647692;
    const double omega = two_pi / duration;
    const int total_steps = totalStepCount(duration);

    vector_t time(total_steps);
    vector_array_t states;
    vector_array_t inputs;
    states.reserve(static_cast<size_t>(total_steps));
    inputs.reserve(static_cast<size_t>(total_steps));

    for (int k = 0; k < total_steps; ++k) {
        const double t = std::min(static_cast<double>(k) * config_.dt, duration);
        const double phase = start_angle + omega * t;
        time(k) = t;

        vector3_t position_in_frame = center;
        position_in_frame(0) += radius * std::cos(phase);
        position_in_frame(1) += radius * std::sin(phase);

        vector3_t velocity_in_frame = vector3_t::Zero();
        velocity_in_frame(0) = -radius * omega * std::sin(phase);
        velocity_in_frame(1) = radius * omega * std::cos(phase);

        const vector3_t position = vector3ToNed(position_in_frame, frame);
        const vector3_t velocity = vector3ToNed(velocity_in_frame, frame);

        vector_t state = vector_t::Zero(config_.state_dim);
        state.segment<3>(0) = position;

        const quaternion_t q = orientationFromNedVelocity(velocity, states);
        state(3) = q.w();
        state(4) = q.x();
        state(5) = q.y();
        state(6) = q.z();

        if (config_.state_dim >= 10) {
            const vector3_t body_velocity = q.conjugate() * velocity;
            state.segment<3>(7) = body_velocity;
        }

        normalizeQuaternionOrIdentity(state);
        keepQuaternionContinuous(state, states);
        states.push_back(state);
        inputs.push_back(vector_t::Zero(config_.input_dim));
    }

    return TargetTrajectories(std::move(time), std::move(states), std::move(inputs));
}

std::optional<TargetTrajectories> TrajectoryGenerator::buildFigure8(
    double x_amplitude,
    double y_amplitude,
    double z_amplitude,
    const vector3_t &center,
    double duration,
    double start_phase,
    const std::string &frame,
    std::string *error) const {
    if (x_amplitude <= 0.0 || y_amplitude <= 0.0) {
        assignError(error, "x_amplitude and y_amplitude must be positive");
        return std::nullopt;
    }
    if (duration <= 0.0) {
        duration = std::max(config_.dt, config_.prediction_dt);
    }

    constexpr double two_pi = 6.28318530717958647692;
    const double omega = two_pi / duration;
    const int total_steps = totalStepCount(duration);

    vector_t time(total_steps);
    vector_array_t states;
    vector_array_t inputs;
    states.reserve(static_cast<size_t>(total_steps));
    inputs.reserve(static_cast<size_t>(total_steps));

    for (int k = 0; k < total_steps; ++k) {
        const double t = std::min(static_cast<double>(k) * config_.dt, duration);
        const double phase = start_phase + omega * t;
        time(k) = t;

        vector3_t position_in_frame = center;
        position_in_frame(0) += x_amplitude * std::sin(phase);
        position_in_frame(1) += y_amplitude * std::sin(2.0 * phase);
        position_in_frame(2) += z_amplitude * std::sin(phase);

        vector3_t velocity_in_frame = vector3_t::Zero();
        velocity_in_frame(0) = x_amplitude * omega * std::cos(phase);
        velocity_in_frame(1) = 2.0 * y_amplitude * omega * std::cos(2.0 * phase);
        velocity_in_frame(2) = z_amplitude * omega * std::cos(phase);

        const vector3_t position = vector3ToNed(position_in_frame, frame);
        const vector3_t velocity = vector3ToNed(velocity_in_frame, frame);

        vector_t state = vector_t::Zero(config_.state_dim);
        state.segment<3>(0) = position;

        const quaternion_t q = orientationFromNedVelocity(velocity, states);
        state(3) = q.w();
        state(4) = q.x();
        state(5) = q.y();
        state(6) = q.z();

        if (config_.state_dim >= 10) {
            const vector3_t body_velocity = q.conjugate() * velocity;
            state.segment<3>(7) = body_velocity;
        }

        normalizeQuaternionOrIdentity(state);
        keepQuaternionContinuous(state, states);
        states.push_back(state);
        inputs.push_back(vector_t::Zero(config_.input_dim));
    }

    return TargetTrajectories(std::move(time), std::move(states), std::move(inputs));
}

std::optional<TargetTrajectories> TrajectoryGenerator::buildPoints(
    const std::vector<double> &state_values,
    const std::vector<double> &input_values,
    const std::vector<double> &time_values,
    const std::string &frame,
    std::string *error) const {
    if (state_values.empty() ||
        state_values.size() % static_cast<size_t>(config_.state_dim) != 0) {
        assignError(error, "states size is not divisible by state_dim");
        return std::nullopt;
    }

    const size_t point_count = state_values.size() / static_cast<size_t>(config_.state_dim);
    vector_t time(point_count);
    if (time_values.empty()) {
        for (size_t i = 0; i < point_count; ++i) {
            time(static_cast<long>(i)) = static_cast<double>(i) * config_.dt;
        }
    } else if (time_values.size() == point_count) {
        time = toEigenVector(time_values);
    } else {
        assignError(error, "time size does not match point count");
        return std::nullopt;
    }

    vector_array_t states;
    vector_array_t inputs;
    states.reserve(point_count);
    inputs.reserve(point_count);
    for (size_t point = 0; point < point_count; ++point) {
        vector_t state = vector_t::Zero(config_.state_dim);
        for (int i = 0; i < config_.state_dim; ++i) {
            state(i) = state_values[point * static_cast<size_t>(config_.state_dim) + static_cast<size_t>(i)];
        }
        states.push_back(stateToControllerFrame(state, frame));
    }

    if (input_values.empty()) {
        inputs.assign(point_count, vector_t::Zero(config_.input_dim));
    } else if (input_values.size() == point_count * static_cast<size_t>(config_.input_dim)) {
        for (size_t point = 0; point < point_count; ++point) {
            vector_t input = vector_t::Zero(config_.input_dim);
            for (int i = 0; i < config_.input_dim; ++i) {
                input(i) = input_values[point * static_cast<size_t>(config_.input_dim) + static_cast<size_t>(i)];
            }
            inputs.push_back(input);
        }
    } else {
        assignError(error, "inputs size does not match point_count * input_dim");
        return std::nullopt;
    }

    return TargetTrajectories(std::move(time), std::move(states), std::move(inputs));
}

std::optional<TargetTrajectories> TrajectoryGenerator::buildBspline(
    const vector_array_t &control_points,
    int degree,
    double duration,
    const std::string &frame,
    std::string *error) const {
    if (control_points.size() < 2) {
        assignError(error, "not enough control points");
        return std::nullopt;
    }

    degree = std::clamp<int>(degree, 1, static_cast<int>(control_points.size()) - 1);
    if (duration <= 0.0) {
        duration = std::max(config_.dt, static_cast<double>(control_points.size() - 1) * config_.prediction_dt);
    }

    const std::vector<double> knots =
        makeClampedUniformKnots(control_points.size(), degree, duration);
    const int total_steps = totalStepCount(duration);

    vector_t time(total_steps);
    vector_array_t states;
    vector_array_t inputs;
    states.reserve(static_cast<size_t>(total_steps));
    inputs.reserve(static_cast<size_t>(total_steps));

    for (int k = 0; k < total_steps; ++k) {
        const double t = std::min(static_cast<double>(k) * config_.dt, duration);
        time(k) = t;

        const vector_t point = evaluateBspline(control_points, knots, degree, t);
        vector_t state = vector_t::Zero(config_.state_dim);
        setIdentityQuaternion(state);

        if (point.size() >= config_.state_dim) {
            state = stateToControllerFrame(point.head(config_.state_dim), frame);
        } else {
            const vector3_t position = positionToNed(point, frame);
            state.segment<3>(0) = position;

            const double prev_t = std::max(0.0, t - config_.prediction_dt);
            const double next_t = std::min(duration, t + config_.prediction_dt);
            vector3_t velocity = vector3_t::Zero();
            if (next_t > prev_t) {
                const vector3_t prev_position =
                    positionToNed(evaluateBspline(control_points, knots, degree, prev_t), frame);
                const vector3_t next_position =
                    positionToNed(evaluateBspline(control_points, knots, degree, next_t), frame);
                velocity = (next_position - prev_position) / (next_t - prev_t);
            }

            const quaternion_t q = orientationFromNedVelocity(velocity, states);
            state(3) = q.w();
            state(4) = q.x();
            state(5) = q.y();
            state(6) = q.z();

            if (config_.state_dim >= 10) {
                const vector3_t body_velocity = q.conjugate() * velocity;
                state.segment<3>(7) = body_velocity;
            }
        }

        normalizeQuaternionOrIdentity(state);
        keepQuaternionContinuous(state, states);
        states.push_back(state);
        inputs.push_back(vector_t::Zero(config_.input_dim));
    }

    return TargetTrajectories(std::move(time), std::move(states), std::move(inputs));
}
