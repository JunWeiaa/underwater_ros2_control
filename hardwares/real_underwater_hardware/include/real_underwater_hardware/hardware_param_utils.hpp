#ifndef REAL_UNDERWATER_HARDWARE_HARDWARE_PARAM_UTILS_HPP_
#define REAL_UNDERWATER_HARDWARE_HARDWARE_PARAM_UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <exception>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace real_underwater_hardware::hardware_param_utils {

using HardwareParams = std::unordered_map<std::string, std::string>;

inline std::string trim(const std::string &value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

inline std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

inline std::string param_string(const HardwareParams &params,
                                const std::string &name,
                                const std::string &fallback) {
    const auto it = params.find(name);
    if (it == params.end()) {
        return fallback;
    }
    const auto value = trim(it->second);
    return value.empty() ? fallback : value;
}

inline int param_int(const HardwareParams &params,
                     const std::string &name,
                     int fallback) {
    const auto it = params.find(name);
    if (it == params.end()) {
        return fallback;
    }
    try {
        return std::stoi(trim(it->second));
    } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                    "Invalid int hardware parameter '%s'='%s'; using %d",
                    name.c_str(),
                    it->second.c_str(),
                    fallback);
        return fallback;
    }
}

inline double param_double(const HardwareParams &params,
                           const std::string &name,
                           double fallback) {
    const auto it = params.find(name);
    if (it == params.end()) {
        return fallback;
    }
    try {
        return std::stod(trim(it->second));
    } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                    "Invalid double hardware parameter '%s'='%s'; using %.3f",
                    name.c_str(),
                    it->second.c_str(),
                    fallback);
        return fallback;
    }
}

inline bool param_bool(const HardwareParams &params,
                       const std::string &name,
                       bool fallback) {
    const auto it = params.find(name);
    if (it == params.end()) {
        return fallback;
    }
    const auto value = to_lower(trim(it->second));
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                "Invalid bool hardware parameter '%s'='%s'; using %s",
                name.c_str(),
                it->second.c_str(),
                fallback ? "true" : "false");
    return fallback;
}

template <typename ParseValue>
void split_values(const std::string &raw, ParseValue parse_value) {
    std::string normalized = raw;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::replace(normalized.begin(), normalized.end(), ';', ' ');
    std::stringstream stream(normalized);
    std::string token;
    while (stream >> token) {
        parse_value(token);
    }
}

inline std::vector<int> param_int_vector(const HardwareParams &params,
                                         const std::string &name,
                                         const std::vector<int> &fallback) {
    const auto it = params.find(name);
    if (it == params.end()) {
        return fallback;
    }

    std::vector<int> values;
    try {
        split_values(it->second, [&values](const std::string &token) {
            values.push_back(std::stoi(token));
        });
    } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                    "Invalid int list hardware parameter '%s'='%s'; using defaults",
                    name.c_str(),
                    it->second.c_str());
        return fallback;
    }
    return values.empty() ? fallback : values;
}

inline std::vector<double> param_double_vector(const HardwareParams &params,
                                               const std::string &name,
                                               const std::vector<double> &fallback) {
    const auto it = params.find(name);
    if (it == params.end()) {
        return fallback;
    }

    std::vector<double> values;
    try {
        split_values(it->second, [&values](const std::string &token) {
            values.push_back(std::stod(token));
        });
    } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("RealSystem"),
                    "Invalid double list hardware parameter '%s'='%s'; using defaults",
                    name.c_str(),
                    it->second.c_str());
        return fallback;
    }
    return values.empty() ? fallback : values;
}

inline std::vector<std::string> param_string_vector(const HardwareParams &params,
                                                    const std::string &name,
                                                    const std::vector<std::string> &fallback) {
    const auto it = params.find(name);
    if (it == params.end()) {
        return fallback;
    }

    std::vector<std::string> values;
    split_values(it->second, [&values](const std::string &token) {
        values.push_back(token);
    });
    return values.empty() ? fallback : values;
}

template <typename T>
void resize_with_defaults(std::vector<T> &values,
                          std::size_t size,
                          const T &default_value) {
    if (values.size() < size) {
        values.resize(size, default_value);
    } else if (values.size() > size) {
        values.resize(size);
    }
}

inline std::string normalize_namespace(std::string ns) {
    ns = trim(ns);
    if (ns.empty()) {
        return "";
    }
    if (ns.front() != '/') {
        ns.insert(ns.begin(), '/');
    }
    while (ns.size() > 1 && ns.back() == '/') {
        ns.pop_back();
    }
    return ns;
}

inline std::string topic_in_namespace(const std::string &ns, const std::string &topic) {
    const auto normalized_ns = normalize_namespace(ns);
    const auto leaf = topic.empty() || topic.front() != '/' ? topic : topic.substr(1);
    if (normalized_ns.empty() || normalized_ns == "/") {
        return "/" + leaf;
    }
    return normalized_ns + "/" + leaf;
}

} // namespace real_underwater_hardware::hardware_param_utils

#endif
