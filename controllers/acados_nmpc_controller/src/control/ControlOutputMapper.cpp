#include "acados_nmpc_controller/control/ControlOutputMapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace {
std::vector<int64_t> defaultIndices(int input_dim) {
    std::vector<int64_t> values(static_cast<size_t>(input_dim));
    for (int i = 0; i < input_dim; ++i) {
        values[static_cast<size_t>(i)] = i;
    }
    return values;
}

std::vector<std::string> defaultStrings(size_t size, const std::string &value) {
    return std::vector<std::string>(size, value);
}

std::vector<int64_t> defaultInt64s(size_t size, int64_t value) {
    return std::vector<int64_t>(size, value);
}

std::vector<double> defaultDoubles(size_t size, double value) {
    return std::vector<double>(size, value);
}

template <typename T>
bool sizeMatches(const std::vector<T> &values, size_t expected) {
    return values.empty() || values.size() == expected;
}
} // namespace

ControlOutputMapper::ControlOutputMapper(rclcpp_lifecycle::LifecycleNode::SharedPtr node, int input_dim) :
    node_(std::move(node)) {
    loadFromParameters(input_dim);
}

void ControlOutputMapper::loadFromParameters(int input_dim) {
    auto indices = declareOrGet<std::vector<int64_t>>("control_output_map.input_indices", defaultIndices(input_dim));
    auto joints = declareOrGet<std::vector<std::string>>("control_output_map.joints", std::vector<std::string>{});
    auto interfaces = declareOrGet<std::vector<std::string>>(
        "control_output_map.interfaces", defaultStrings(indices.size(), "thrust"));
    auto modes = declareOrGet<std::vector<std::string>>(
        "control_output_map.modes", defaultStrings(indices.size(), "absolute"));
    auto state_indices = declareOrGet<std::vector<int64_t>>(
        "control_output_map.state_indices", defaultInt64s(indices.size(), -1));
    auto scales = declareOrGet<std::vector<double>>(
        "control_output_map.scales", defaultDoubles(indices.size(), 1.0));
    auto offsets = declareOrGet<std::vector<double>>(
        "control_output_map.offsets", defaultDoubles(indices.size(), 0.0));
    auto mins = declareOrGet<std::vector<double>>(
        "control_output_map.min", defaultDoubles(indices.size(), -std::numeric_limits<double>::infinity()));
    auto maxs = declareOrGet<std::vector<double>>(
        "control_output_map.max", defaultDoubles(indices.size(), std::numeric_limits<double>::infinity()));
    auto dts = declareOrGet<std::vector<double>>(
        "control_output_map.dt", defaultDoubles(indices.size(), 0.0));

    if (!sizeMatches(joints, indices.size()) ||
        !sizeMatches(interfaces, indices.size()) ||
        !sizeMatches(modes, indices.size()) ||
        !sizeMatches(state_indices, indices.size()) ||
        !sizeMatches(scales, indices.size()) ||
        !sizeMatches(offsets, indices.size()) ||
        !sizeMatches(mins, indices.size()) ||
        !sizeMatches(maxs, indices.size()) ||
        !sizeMatches(dts, indices.size())) {
        throw std::runtime_error("control_output_map arrays must either be empty or match input_indices size");
    }

    use_named_interfaces_ = !joints.empty();
    entries_.clear();
    entries_.reserve(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        OutputEntry entry;
        entry.input_index = static_cast<int>(indices[i]);
        entry.joint = joints.empty() ? "" : joints[i];
        entry.interface = interfaces.empty() ? "thrust" : interfaces[i];
        entry.mode = parseMode(modes.empty() ? "absolute" : modes[i]);
        entry.state_index = static_cast<int>(state_indices.empty() ? -1 : state_indices[i]);
        entry.scale = scales.empty() ? 1.0 : scales[i];
        entry.offset = offsets.empty() ? 0.0 : offsets[i];
        entry.min = mins.empty() ? -std::numeric_limits<double>::infinity() : mins[i];
        entry.max = maxs.empty() ? std::numeric_limits<double>::infinity() : maxs[i];
        entry.dt = dts.empty() ? 0.0 : dts[i];
        entries_.push_back(entry);
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded %zu control output map entries", entries_.size());
}

void ControlOutputMapper::write(const vector_t &control,
                                const vector_t &state,
                                const rclcpp::Duration &period,
                                ControllerInterfaces &controller_interfaces) const {
    for (size_t output_order = 0; output_order < entries_.size(); ++output_order) {
        const auto &entry = entries_[output_order];
        if (entry.input_index < 0 || entry.input_index >= control.size()) {
            RCLCPP_WARN(node_->get_logger(),
                        "Skipping control output %zu: input index %d is outside control size %ld",
                        output_order,
                        entry.input_index,
                        control.size());
            continue;
        }

        double value = control(entry.input_index) * entry.scale + entry.offset;
        if (entry.mode == OutputMode::INTEGRATE) {
            const double base = (entry.state_index >= 0 && entry.state_index < state.size()) ? state(entry.state_index) : 0.0;
            const double dt = entry.dt > 0.0 ? entry.dt : period.seconds();
            value = base + control(entry.input_index) * entry.scale * dt + entry.offset;
        }
        value = std::clamp(value, entry.min, entry.max);

        if (use_named_interfaces_) {
            const auto command_it = controller_interfaces.command_interfaces_by_name_.find(commandKey(entry));
            if (command_it == controller_interfaces.command_interfaces_by_name_.end()) {
                RCLCPP_WARN(node_->get_logger(),
                            "Command interface '%s/%s' is not available",
                            entry.joint.c_str(),
                            entry.interface.c_str());
                continue;
            }
            std::ignore = command_it->second->set_value(value);
            continue;
        }

        auto *group = interfaceGroup(entry.interface, controller_interfaces);
        if (group == nullptr || output_order >= group->size()) {
            RCLCPP_WARN(node_->get_logger(),
                        "Command interface group '%s' does not have output index %zu",
                        entry.interface.c_str(),
                        output_order);
            continue;
        }
        std::ignore = (*group)[output_order].get().set_value(value);
    }
}

std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *
ControlOutputMapper::interfaceGroup(const std::string &interface_name,
                                    ControllerInterfaces &controller_interfaces) const {
    if (interface_name == "thrust") {
        return &controller_interfaces.joint_thrust_command_interface_;
    }
    if (interface_name == "position") {
        return &controller_interfaces.joint_position_command_interface_;
    }
    if (interface_name == "velocity") {
        return &controller_interfaces.joint_velocity_command_interface_;
    }
    if (interface_name == "effort") {
        return &controller_interfaces.joint_torque_command_interface_;
    }
    return nullptr;
}

ControlOutputMapper::OutputMode ControlOutputMapper::parseMode(const std::string &mode) {
    if (mode == "absolute") {
        return OutputMode::ABSOLUTE;
    }
    if (mode == "integrate") {
        return OutputMode::INTEGRATE;
    }
    throw std::runtime_error("Unsupported control_output_map mode '" + mode + "'");
}

std::string ControlOutputMapper::commandKey(const OutputEntry &entry) {
    return entry.joint + "/" + entry.interface;
}
