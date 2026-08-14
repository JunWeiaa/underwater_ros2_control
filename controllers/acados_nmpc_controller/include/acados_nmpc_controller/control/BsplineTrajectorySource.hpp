#pragma once

#include "acados_nmpc_controller/utils/Types.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include <optional>
#include <string>

struct BsplineTrajectoryRequest {
    vector_array_t control_points;
    int degree{3};
    double duration{0.0};
    std::string frame{"enu"};
};

class BsplineTrajectorySource {
public:
    std::optional<BsplineTrajectoryRequest> parseMessage(
        const std_msgs::msg::Float64MultiArray &msg,
        int default_degree,
        double default_duration,
        const std::string &default_frame,
        std::string *error = nullptr) const;
};
