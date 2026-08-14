#ifndef _STATEFSM_HPP
#define _STATEFSM_HPP
#include "acados_nmpc_controller/interfaces/ControllerInterfaces.hpp"
#include <rclcpp/time.hpp>
#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include <thread>
enum class FSMStateName {
    // EXIT,
    INVALID,
    NO_OUTPUT,
    AUTO,
    MANUAL,

};
template <typename Functor>
void executeAndSleep(Functor f, const double frequency) {
    if (frequency <= 0.0) {
        f();
        return;
    }

    using clock = std::chrono::steady_clock;
    const auto start = clock::now();

    f();

    const std::chrono::duration<double> period(1.0 / frequency);
    std::this_thread::sleep_until(start + period);
}

inline void setThreadPriority(int priority, std::thread &thread) {
    sched_param sched{};
    sched.sched_priority = priority;

    if (priority != 0) {
        if (pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sched) != 0) {
            std::cerr
                << "WARNING: Failed to set threads priority (one possible reason "
                   "could be "
                   "that the user and the group permissions are not set properly.)"
                << std::endl;
        }
    }
}
enum class FSMMode { NORMAL,
                     CHANGE };

class FSMState {
public:
    virtual ~FSMState() = default;

    FSMState(const FSMStateName &state_name, std::string state_name_string, ControllerInterfaces &controller_interfaces) :
        state_name(state_name), state_name_string(std::move(state_name_string)),
        controller_interfaces_(controller_interfaces) {
    }

    virtual void enter() = 0;

    virtual void run(const rclcpp::Time &time,
                     const rclcpp::Duration &period) = 0;

    virtual void exit() = 0;

    virtual FSMStateName checkChange() {
        return FSMStateName::INVALID;
    }

    FSMStateName state_name;
    std::string state_name_string;

protected:
    ControllerInterfaces &controller_interfaces_;
};
#endif
