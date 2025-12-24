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

#include "../canbus/can_device.hpp"
#include "../canbus/can_socket.hpp"
#include "dm_motor.hpp"
#include "dm_motor_control.hpp"

namespace openarm::damiao_motor {
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
    ControlMode get_control_mode() const { return control_mode_; }
    void set_control_mode(ControlMode control_mode) { control_mode_ = control_mode; }

private:
    std::vector<uint8_t> get_data_from_frame(const can_frame& frame);
    std::vector<uint8_t> get_data_from_frame(const canfd_frame& frame);
    Motor& motor_;
    CallbackMode callback_mode_;
    bool use_fd_;  // Track if using CAN-FD
    ControlMode control_mode_ = ControlMode::MIT;
};
}  // namespace openarm::damiao_motor
