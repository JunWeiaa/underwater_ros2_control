#include "real_underwater_hardware/servo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

namespace real_underwater_hardware {

constexpr uint8_t ServoController::REQ_HDR[2];
constexpr uint8_t ServoController::RSP_HDR[2];

ServoController::~ServoController() {
    close_all();
}

bool ServoController::add_servo(int servo_index,
                                const std::string &port,
                                uint8_t servo_id,
                                int baudrate,
                                int timeout_ms) {
    if (servo_index < 0) {
        std::cerr << "ServoController: invalid servo index " << servo_index << std::endl;
        return false;
    }

    ServoInfo servo_info;
    servo_info.port = port;
    servo_info.servo_id = servo_id;
    servo_info.baudrate = baudrate;
    servo_info.timeout_ms = timeout_ms;

    servos_[servo_index] = servo_info;
    return open_serial(servos_[servo_index]);
}

bool ServoController::ping(int servo_index) {
    auto it = servos_.find(servo_index);
    if (it == servos_.end() || !it->second.is_open) {
        return false;
    }

    ServoInfo &servo = it->second;
    const auto frame = build_request(CMD_PING, {servo.servo_id});
    if (write(servo.serial_fd, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
        return false;
    }

    try {
        const auto response = read_response(servo.serial_fd, CMD_PING, 1);
        return !response.empty() && response[0] == servo.servo_id;
    } catch (const ServoProtocolError &) {
        return false;
    }
}

bool ServoController::set_angle_single(int servo_index,
                                       double angle_rad,
                                       int duration_ms,
                                       int power,
                                       bool wait_ack) {
    auto it = servos_.find(servo_index);
    if (it == servos_.end() || !it->second.is_open) {
        return false;
    }

    ServoInfo &servo = it->second;
    const int16_t angle_ticks = rad_to_protocol_angle(angle_rad);
    const uint16_t duration = static_cast<uint16_t>(std::clamp(duration_ms, 0, 65535));
    const uint16_t power_value = static_cast<uint16_t>(std::clamp(power, 0, 65535));

    std::vector<uint8_t> content;
    content.push_back(servo.servo_id);
    content.push_back(static_cast<uint8_t>(angle_ticks & 0xFF));
    content.push_back(static_cast<uint8_t>((angle_ticks >> 8) & 0xFF));
    content.push_back(static_cast<uint8_t>(duration & 0xFF));
    content.push_back(static_cast<uint8_t>((duration >> 8) & 0xFF));
    content.push_back(static_cast<uint8_t>(power_value & 0xFF));
    content.push_back(static_cast<uint8_t>((power_value >> 8) & 0xFF));

    const auto frame = build_request(CMD_SET_ANGLE_SINGLE, content);
    if (write(servo.serial_fd, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
        return false;
    }

    if (!wait_ack) {
        return true;
    }

    try {
        const auto response = read_response(servo.serial_fd, CMD_SET_ANGLE_SINGLE, 2);
        return response.size() >= 2 && response[0] == servo.servo_id && response[1] == 0x01;
    } catch (const ServoProtocolError &) {
        return false;
    }
}

double ServoController::get_angle_single(int servo_index) {
    auto it = servos_.find(servo_index);
    if (it == servos_.end() || !it->second.is_open) {
        throw ServoProtocolError("Servo not found or not connected");
    }

    ServoInfo &servo = it->second;
    const auto frame = build_request(CMD_GET_ANGLE_SINGLE, {servo.servo_id});
    if (write(servo.serial_fd, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
        throw ServoProtocolError("Failed to write get angle command");
    }

    const auto response = read_response(servo.serial_fd, CMD_GET_ANGLE_SINGLE, 3);
    if (response.size() < 3 || response[0] != servo.servo_id) {
        throw ServoProtocolError("Invalid response for get angle");
    }

    const int16_t angle_ticks = static_cast<int16_t>(response[1] | (response[2] << 8));
    return protocol_angle_to_rad(angle_ticks);
}

void ServoController::set_all_angles(const std::vector<double> &angles_rad,
                                     int duration_ms) {
    for (size_t i = 0; i < angles_rad.size(); ++i) {
        const auto servo_index = static_cast<int>(i);
        if (servos_.find(servo_index) != servos_.end()) {
            set_angle_single(servo_index, angles_rad[i], duration_ms, 0, false);
        }
    }
}

void ServoController::close_all() {
    for (auto &servo : servos_) {
        close_serial(servo.second);
    }
    servos_.clear();
}

bool ServoController::is_servo_connected(int servo_index) const {
    const auto it = servos_.find(servo_index);
    return it != servos_.end() && it->second.is_open;
}

uint8_t ServoController::checksum(const std::vector<uint8_t> &payload) {
    uint32_t sum = 0;
    for (uint8_t byte : payload) {
        sum += byte;
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

std::vector<uint8_t> ServoController::build_request(uint8_t cmd_id,
                                                    const std::vector<uint8_t> &content) {
    if (content.size() > 255) {
        throw ServoProtocolError("Content too long");
    }

    std::vector<uint8_t> frame;
    frame.reserve(content.size() + 5);
    frame.push_back(REQ_HDR[0]);
    frame.push_back(REQ_HDR[1]);
    frame.push_back(cmd_id);
    frame.push_back(static_cast<uint8_t>(content.size()));
    frame.insert(frame.end(), content.begin(), content.end());
    frame.push_back(checksum(frame));
    return frame;
}

std::vector<uint8_t> ServoController::read_response(int serial_fd,
                                                    uint8_t expect_cmd,
                                                    size_t min_len) {
    uint8_t header[2];
    if (read(serial_fd, header, 2) != 2) {
        throw ServoProtocolError("Failed to read response header");
    }
    if (header[0] != RSP_HDR[0] || header[1] != RSP_HDR[1]) {
        throw ServoProtocolError("Invalid response header");
    }

    uint8_t cmd_len[2];
    if (read(serial_fd, cmd_len, 2) != 2) {
        throw ServoProtocolError("Failed to read command and length");
    }

    const uint8_t cmd_id = cmd_len[0];
    const uint8_t content_len = cmd_len[1];
    if (cmd_id != expect_cmd) {
        throw ServoProtocolError("Unexpected command in response");
    }
    if (content_len < min_len) {
        throw ServoProtocolError("Response content too short");
    }

    std::vector<uint8_t> content(content_len);
    if (content_len > 0 &&
        read(serial_fd, content.data(), content_len) != static_cast<ssize_t>(content_len)) {
        throw ServoProtocolError("Failed to read response content");
    }

    uint8_t received_checksum = 0;
    if (read(serial_fd, &received_checksum, 1) != 1) {
        throw ServoProtocolError("Failed to read checksum");
    }

    std::vector<uint8_t> frame_for_checksum;
    frame_for_checksum.insert(frame_for_checksum.end(), header, header + 2);
    frame_for_checksum.push_back(cmd_id);
    frame_for_checksum.push_back(content_len);
    frame_for_checksum.insert(frame_for_checksum.end(), content.begin(), content.end());
    if (checksum(frame_for_checksum) != received_checksum) {
        throw ServoProtocolError("Checksum mismatch");
    }

    return content;
}

int16_t ServoController::rad_to_protocol_angle(double rad) {
    constexpr double radians_to_degrees = 180.0 / M_PI;
    const double degrees = rad * radians_to_degrees;
    const auto ticks = static_cast<int16_t>(std::round(degrees * 10.0));
    return std::clamp<int16_t>(ticks, -1800, 1800);
}

double ServoController::protocol_angle_to_rad(int16_t protocol_angle) {
    constexpr double degrees_to_radians = M_PI / 180.0;
    return static_cast<double>(protocol_angle) / 10.0 * degrees_to_radians;
}

bool ServoController::open_serial(ServoInfo &servo_info) {
    servo_info.serial_fd = open(servo_info.port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (servo_info.serial_fd == -1) {
        std::cerr << "ServoController: failed to open " << servo_info.port
                  << " - " << strerror(errno) << std::endl;
        return false;
    }

    termios options;
    if (tcgetattr(servo_info.serial_fd, &options) != 0) {
        ::close(servo_info.serial_fd);
        servo_info.serial_fd = -1;
        std::cerr << "ServoController: failed to get serial attributes" << std::endl;
        return false;
    }

    speed_t baud;
    switch (servo_info.baudrate) {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:
        ::close(servo_info.serial_fd);
        servo_info.serial_fd = -1;
        std::cerr << "ServoController: unsupported baud rate " << servo_info.baudrate << std::endl;
        return false;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = static_cast<cc_t>(std::max(1, servo_info.timeout_ms / 100));

    if (tcsetattr(servo_info.serial_fd, TCSANOW, &options) != 0) {
        ::close(servo_info.serial_fd);
        servo_info.serial_fd = -1;
        std::cerr << "ServoController: failed to set serial attributes" << std::endl;
        return false;
    }

    tcflush(servo_info.serial_fd, TCIOFLUSH);
    servo_info.is_open = true;
    std::cout << "ServoController: opened " << servo_info.port << std::endl;
    return true;
}

void ServoController::close_serial(ServoInfo &servo_info) {
    if (servo_info.serial_fd >= 0) {
        ::close(servo_info.serial_fd);
        servo_info.serial_fd = -1;
    }
    servo_info.is_open = false;
}

} // namespace real_underwater_hardware
