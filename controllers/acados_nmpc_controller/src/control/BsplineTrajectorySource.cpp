#include "acados_nmpc_controller/control/BsplineTrajectorySource.hpp"

#include <algorithm>
#include <cmath>

namespace {
void assignError(std::string *error, const std::string &message) {
    if (error != nullptr) {
        *error = message;
    }
}
} // namespace

std::optional<BsplineTrajectoryRequest> BsplineTrajectorySource::parseMessage(
    const std_msgs::msg::Float64MultiArray &msg,
    int default_degree,
    double default_duration,
    const std::string &default_frame,
    std::string *error) const {
    const auto &data = msg.data;
    const size_t offset = msg.layout.data_offset;
    if (offset >= data.size()) {
        assignError(error, "data_offset is outside data");
        return std::nullopt;
    }

    BsplineTrajectoryRequest request;
    request.degree = default_degree;
    request.duration = default_duration;
    request.frame = default_frame;

    size_t point_dim = 0;
    size_t point_count = 0;
    size_t data_start = offset;
    const size_t available = data.size() - offset;

    if (msg.layout.dim.size() >= 2 &&
        msg.layout.dim[0].size > 0 &&
        msg.layout.dim[1].size > 0) {
        point_count = static_cast<size_t>(msg.layout.dim[0].size);
        point_dim = static_cast<size_t>(msg.layout.dim[1].size);
    } else if (available >= 6) {
        request.degree = std::max(1, static_cast<int>(std::llround(data[offset])));
        request.duration = data[offset + 1];
        point_dim = static_cast<size_t>(std::llround(data[offset + 2]));
        data_start = offset + 3;
        if (point_dim > 0) {
            point_count = (data.size() - data_start) / point_dim;
        }
    }

    if (point_dim < 3 || point_count < 2) {
        assignError(error, "expected control points with at least 2 rows and 3 columns");
        return std::nullopt;
    }
    if (data.size() - data_start != point_count * point_dim) {
        assignError(error, "data size does not match control point layout");
        return std::nullopt;
    }

    request.control_points.reserve(point_count);
    for (size_t point = 0; point < point_count; ++point) {
        vector_t control_point = vector_t::Zero(static_cast<int>(point_dim));
        for (size_t i = 0; i < point_dim; ++i) {
            control_point(static_cast<long>(i)) = data[data_start + point * point_dim + i];
        }
        request.control_points.push_back(control_point);
    }

    return request;
}
