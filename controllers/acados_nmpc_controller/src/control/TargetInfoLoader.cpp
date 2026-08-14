#include "acados_nmpc_controller/control/TargetInfoLoader.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <sstream>
#include <unordered_map>

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

std::string stripComment(const std::string &line) {
    const size_t comment = line.find('#');
    return comment == std::string::npos ? line : line.substr(0, comment);
}

std::string stripQuotes(std::string value) {
    value = trimCopy(value);
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') ||
         (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

std::vector<double> parseDoubleList(std::string value) {
    for (char &c : value) {
        if (c == '[' || c == ']' || c == ',' || c == ';') {
            c = ' ';
        }
    }

    std::vector<double> values;
    std::istringstream stream(value);
    double number = 0.0;
    while (stream >> number) {
        values.push_back(number);
    }
    return values;
}

std::unordered_map<std::string, std::string> parseInfoFile(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return {};
    }

    std::unordered_map<std::string, std::string> values;
    std::string section;
    std::string pending_key;
    std::string pending_value;
    bool reading_array = false;

    std::string raw_line;
    while (std::getline(file, raw_line)) {
        std::string line = trimCopy(stripComment(raw_line));
        if (line.empty()) {
            continue;
        }

        if (reading_array) {
            pending_value += " " + line;
            if (line.find(']') != std::string::npos) {
                values[pending_key] = pending_value;
                pending_key.clear();
                pending_value.clear();
                reading_array = false;
            }
            continue;
        }

        if (line.front() == '[' && line.back() == ']') {
            section = trimCopy(line.substr(1, line.size() - 2));
            continue;
        }

        const size_t separator = line.find_first_of(":=");
        if (separator == std::string::npos) {
            continue;
        }

        std::string key = trimCopy(line.substr(0, separator));
        const std::string value = trimCopy(line.substr(separator + 1));
        if (!section.empty()) {
            key = section + "." + key;
        }

        if (value.find('[') != std::string::npos &&
            value.find(']') == std::string::npos) {
            pending_key = key;
            pending_value = value;
            reading_array = true;
        } else {
            values[key] = value;
        }
    }
    return values;
}

bool hasInfoValue(const std::unordered_map<std::string, std::string> &values,
                  const std::string &key) {
    return values.find(key) != values.end();
}

std::string getInfoString(const std::unordered_map<std::string, std::string> &values,
                          const std::string &key,
                          const std::string &default_value) {
    const auto it = values.find(key);
    return it == values.end() ? default_value : stripQuotes(it->second);
}

double getInfoDouble(const std::unordered_map<std::string, std::string> &values,
                     const std::string &key,
                     double default_value) {
    const auto it = values.find(key);
    if (it == values.end()) {
        return default_value;
    }
    try {
        return std::stod(stripQuotes(it->second));
    } catch (const std::exception &) {
        return default_value;
    }
}

vector3_t getInfoVector3(const std::unordered_map<std::string, std::string> &values,
                         const std::string &key,
                         const vector3_t &default_value) {
    const auto it = values.find(key);
    if (it == values.end()) {
        return default_value;
    }
    const auto parsed = parseDoubleList(it->second);
    if (parsed.size() < 3) {
        return default_value;
    }
    return vector3_t(parsed[0], parsed[1], parsed[2]);
}
} // namespace

std::optional<TargetDefinition> TargetInfoLoader::load(
    const std::string &target_file,
    const std::string &target_name,
    std::string *error,
    std::vector<std::string> *warnings) const {
    if (target_file.empty()) {
        assignError(error, "trajectory.target_file is empty");
        return std::nullopt;
    }

    const auto info = parseInfoFile(target_file);
    if (info.empty()) {
        assignError(error, "failed to read target info file '" + target_file + "'");
        return std::nullopt;
    }

    TargetDefinition definition;
    definition.selected_target = trimCopy(target_name);
    if (definition.selected_target.empty()) {
        definition.selected_target = "circle";
        if (warnings != nullptr) {
            warnings->push_back(
                "trajectory.target_name is empty; using default target 'circle'. "
                "target.info only defines presets and does not select the active target.");
        }
    }

    const std::string prefix = definition.selected_target + ".";
    definition.type = lowerCopy(getInfoString(info, prefix + "type", ""));
    definition.frame = getInfoString(info, prefix + "frame", "enu");
    definition.duration = getInfoDouble(info, prefix + "duration", 0.0);
    definition.source_label = target_file + ":" + definition.selected_target;

    if (definition.type == "circle") {
        definition.radius = getInfoDouble(info, prefix + "radius", 1.0);
        definition.start_angle = getInfoDouble(info, prefix + "start_angle", 0.0);
        definition.center = getInfoVector3(info, prefix + "center", vector3_t::Zero());
        return definition;
    }

    if (definition.type == "figure8" || definition.type == "figure_8") {
        definition.radius = getInfoDouble(info, prefix + "radius", 1.0);
        definition.x_amplitude =
            getInfoDouble(info, prefix + "x_amplitude", definition.radius);
        definition.y_amplitude =
            getInfoDouble(info, prefix + "y_amplitude", definition.radius);
        definition.z_amplitude = getInfoDouble(info, prefix + "z_amplitude", 0.0);
        definition.start_phase = getInfoDouble(info, prefix + "start_phase", 0.0);
        definition.center = getInfoVector3(info, prefix + "center", vector3_t::Zero());
        return definition;
    }

    if (definition.type == "points" || definition.type == "trajectory") {
        if (!hasInfoValue(info, prefix + "states")) {
            assignError(error,
                        "target '" + definition.selected_target + "' in '" +
                            target_file + "' has no states");
            return std::nullopt;
        }

        definition.states = parseDoubleList(info.at(prefix + "states"));
        definition.inputs = hasInfoValue(info, prefix + "inputs")
                                ? parseDoubleList(info.at(prefix + "inputs"))
                                : std::vector<double>{};
        definition.time = hasInfoValue(info, prefix + "time")
                              ? parseDoubleList(info.at(prefix + "time"))
                              : std::vector<double>{};
        return definition;
    }

    return definition;
}
