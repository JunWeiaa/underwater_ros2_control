#pragma once

#include "acados_nmpc_controller/utils/Types.hpp"

#include <optional>
#include <string>
#include <vector>

struct TargetDefinition {
    std::string selected_target;
    std::string type;
    std::string frame{"enu"};
    std::string source_label;

    double duration{0.0};
    double radius{1.0};
    double start_angle{0.0};
    double x_amplitude{1.0};
    double y_amplitude{1.0};
    double z_amplitude{0.0};
    double start_phase{0.0};
    vector3_t center{vector3_t::Zero()};

    std::vector<double> states;
    std::vector<double> inputs;
    std::vector<double> time;
};

class TargetInfoLoader {
public:
    std::optional<TargetDefinition> load(const std::string &target_file,
                                         const std::string &target_name,
                                         std::string *error = nullptr,
                                         std::vector<std::string> *warnings = nullptr) const;
};
