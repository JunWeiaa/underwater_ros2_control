#pragma once

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace real_underwater_hardware {

class ServoProtocolError : public std::runtime_error {
public:
    explicit ServoProtocolError(const std::string &message) :
        std::runtime_error(message) {
    }
};

class ServoController {
public:
    static constexpr uint8_t REQ_HDR[2] = {0x12, 0x4C};
    static constexpr uint8_t RSP_HDR[2] = {0x05, 0x1C};

    static constexpr uint8_t CMD_PING = 0x01;
    static constexpr uint8_t CMD_SET_ANGLE_SINGLE = 0x08;
    static constexpr uint8_t CMD_GET_ANGLE_SINGLE = 0x0A;

    ServoController() = default;
    ~ServoController();

    bool add_servo(int servo_index,
                   const std::string &port,
                   uint8_t servo_id = 0,
                   int baudrate = 115200,
                   int timeout_ms = 100);
    bool ping(int servo_index);
    bool set_angle_single(int servo_index,
                          double angle_rad,
                          int duration_ms = 500,
                          int power = 0,
                          bool wait_ack = false);
    double get_angle_single(int servo_index);
    void set_all_angles(const std::vector<double> &angles_rad,
                        int duration_ms = 500);
    void close_all();
    bool is_servo_connected(int servo_index) const;

private:
    struct ServoInfo {
        std::string port;
        uint8_t servo_id{0};
        int serial_fd{-1};
        int baudrate{115200};
        int timeout_ms{100};
        bool is_open{false};
    };

    static uint8_t checksum(const std::vector<uint8_t> &payload);
    static int16_t rad_to_protocol_angle(double rad);
    static double protocol_angle_to_rad(int16_t protocol_angle);

    std::vector<uint8_t> build_request(uint8_t cmd_id,
                                       const std::vector<uint8_t> &content);
    std::vector<uint8_t> read_response(int serial_fd,
                                       uint8_t expect_cmd,
                                       size_t min_len = 0);
    bool open_serial(ServoInfo &servo_info);
    void close_serial(ServoInfo &servo_info);

    std::map<int, ServoInfo> servos_;
};

} // namespace real_underwater_hardware
