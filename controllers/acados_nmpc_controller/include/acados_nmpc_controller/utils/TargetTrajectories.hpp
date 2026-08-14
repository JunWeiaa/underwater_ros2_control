#ifndef _TARGETTRAJECTORIES_HPP
#define _TARGETTRAJECTORIES_HPP

#include "Types.hpp"
#include <utility>

class TargetTrajectories {
public:
    TargetTrajectories(vector_t time,
                       vector_array_t state,
                       vector_array_t input) :
        time_(std::move(time)), state_(std::move(state)), input_(std::move(input)) {
    }

    vector_t &time() {
        return time_;
    }
    const vector_t &time() const {
        return time_;
    }
    vector_array_t &state() {
        return state_;
    }
    const vector_array_t &state() const {
        return state_;
    }
    vector_array_t &input() {
        return input_;
    }
    const vector_array_t &input() const {
        return input_;
    }

private:
    vector_t time_;
    vector_array_t state_;
    vector_array_t input_;
};

#endif // _TARGETTRAJECTORIES_HPP
