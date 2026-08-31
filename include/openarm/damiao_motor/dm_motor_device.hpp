// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>

#include "../canbus/can_device.hpp"
#include "../canbus/can_socket.hpp"
#include "dm_motor.hpp"
#include "dm_motor_control.hpp"

namespace openarm::damiao_motor {

// Delivery counts and liveness for one axis.
//
// CAN has no notion of a node being absent: a frame is acknowledged by any node
// that hears it, so unplugging one motor out of eight raises no bus error at
// all. Only the application knows that a given id was supposed to answer, which
// makes counting what came back the only way to see that an axis went quiet.
//
// Nothing here decides what counts as too long. The library reports, and the
// caller picks the threshold and what to do about it.
struct MotorLinkStats {
    uint64_t commands_sent = 0;
    uint64_t responses = 0;
    std::chrono::steady_clock::time_point last_response{};

    bool ever_responded() const { return responses > 0; }

    // Derived on demand rather than tracked per cycle, so the receive path
    // carries no bookkeeping beyond two increments and a timestamp.
    double miss_rate() const {
        if (commands_sent == 0) return 0.0;
        double rate = 1.0 - static_cast<double>(responses) / static_cast<double>(commands_sent);
        return rate < 0.0 ? 0.0 : rate;
    }

    std::chrono::microseconds since_last_response() const {
        if (responses == 0) return std::chrono::microseconds::max();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - last_response);
    }

    // An axis that never answered is stale by definition.
    bool is_stale(std::chrono::microseconds timeout) const {
        return responses == 0 || since_last_response() > timeout;
    }
};
enum CallbackMode {
    STATE,
    PARAM,
    // discard
    IGNORE
};

class DMCANDevice : public canbus::CANDevice {
public:
    explicit DMCANDevice(Motor& motor, canid_t recv_can_mask, bool use_fd);
    void callback(const can_frame& frame);
    void callback(const canfd_frame& frame);

    // Create frame from data array
    can_frame create_can_frame(canid_t send_can_id, std::vector<uint8_t> data);
    canfd_frame create_canfd_frame(canid_t send_can_id, std::vector<uint8_t> data);
    // Getter method to access motor state
    Motor& get_motor() { return motor_; }
    void set_callback_mode(CallbackMode callback_mode) { callback_mode_ = callback_mode; }

    const MotorLinkStats& get_link_stats() const { return link_stats_; }
    // Called by the collection for every command addressed to this device.
    void record_command_sent() { link_stats_.commands_sent++; }
    ControlMode get_control_mode() const { return control_mode_; }
    void set_control_mode(ControlMode control_mode) { control_mode_ = control_mode; }

private:
    std::vector<uint8_t> get_data_from_frame(const can_frame& frame);
    std::vector<uint8_t> get_data_from_frame(const canfd_frame& frame);
    Motor& motor_;
    CallbackMode callback_mode_;
    MotorLinkStats link_stats_;
    bool use_fd_;  // Track if using CAN-FD
    ControlMode control_mode_ = ControlMode::MIT;
};
}  // namespace openarm::damiao_motor
