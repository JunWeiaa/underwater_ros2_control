#ifndef ACADOS_NMPC_CONTROLLER_CONTROL_CONTROL_OUTPUT_MAPPER_HPP
#define ACADOS_NMPC_CONTROLLER_CONTROL_CONTROL_OUTPUT_MAPPER_HPP

#include "acados_nmpc_controller/interfaces/ControllerInterfaces.hpp"
#include "acados_nmpc_controller/utils/Types.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

class ControlOutputMapper {
public:
    ControlOutputMapper(rclcpp_lifecycle::LifecycleNode::SharedPtr node, int input_dim);

    void write(const vector_t &control,
               const vector_t &state,
               const rclcpp::Duration &period,
               ControllerInterfaces &controller_interfaces) const;

private:
    enum class OutputMode {
        ABSOLUTE,
        INTEGRATE
    };

    struct OutputEntry {
        std::string joint;
        std::string interface;
        int input_index{0};
        OutputMode mode{OutputMode::ABSOLUTE};
        int state_index{-1};
        double scale{1.0};
        double offset{0.0};
        double min{-std::numeric_limits<double>::infinity()};
        double max{std::numeric_limits<double>::infinity()};
        double dt{0.0};
    };

    template <typename T>
    T declareOrGet(const std::string &name, const T &default_value) const {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<T>(name, default_value);
        }
        T value = default_value;
        node_->get_parameter(name, value);
        return value;
    }

    void loadFromParameters(int input_dim);
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *interfaceGroup(
        const std::string &interface_name,
        ControllerInterfaces &controller_interfaces) const;
    static OutputMode parseMode(const std::string &mode);
    static std::string commandKey(const OutputEntry &entry);

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::vector<OutputEntry> entries_;
    bool use_named_interfaces_{false};
};

#endif // ACADOS_NMPC_CONTROLLER_CONTROL_CONTROL_OUTPUT_MAPPER_HPP
