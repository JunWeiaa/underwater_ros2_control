#pragma once

#include "acados_nmpc_controller/utils/TargetTrajectories.hpp"
#include "acados_nmpc_controller/utils/Types.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"

#include <optional>
#include <string>
#include <vector>

struct TrajectoryGeneratorConfig {
    int state_dim{13};
    int input_dim{8};
    int horizon_steps{20};
    int prediction_step_ratio{1};
    double dt{0.001};
    double prediction_dt{0.025};
};

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(TrajectoryGeneratorConfig config);

    std::optional<TargetTrajectories> buildCircle(double radius,
                                                  const vector3_t &center,
                                                  double duration,
                                                  double start_angle,
                                                  const std::string &frame,
                                                  std::string *error = nullptr) const;

    std::optional<TargetTrajectories> buildFigure8(double x_amplitude,
                                                   double y_amplitude,
                                                   double z_amplitude,
                                                   const vector3_t &center,
                                                   double duration,
                                                   double start_phase,
                                                   const std::string &frame,
                                                   std::string *error = nullptr) const;

    std::optional<TargetTrajectories> buildPoints(const std::vector<double> &state_values,
                                                  const std::vector<double> &input_values,
                                                  const std::vector<double> &time_values,
                                                  const std::string &frame,
                                                  std::string *error = nullptr) const;

    std::optional<TargetTrajectories> buildBspline(const vector_array_t &control_points,
                                                   int degree,
                                                   double duration,
                                                   const std::string &frame,
                                                   std::string *error = nullptr) const;

    static vector_t stateToControllerFrame(const vector_t &source,
                                           const std::string &frame);
    static void normalizeQuaternionOrIdentity(vector_t &state);
    static void keepQuaternionContinuous(vector_t &state,
                                         const vector_array_t &trajectory);
    static void fillPoseStampedFromState(const vector_t &state,
                                         const rclcpp::Time &stamp,
                                         const std::string &frame_id,
                                         geometry_msgs::msg::PoseStamped &pose);

private:
    int totalStepCount(double duration) const;

    TrajectoryGeneratorConfig config_;
};
