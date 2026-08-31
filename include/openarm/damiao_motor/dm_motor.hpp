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

#include <cstdint>
#include <cstring>
#include <map>

#include "dm_motor_constants.hpp"

namespace openarm::damiao_motor {
class Motor {
    friend class DMCANDevice;  // Allow MotorDeviceCan to access protected
                               // members
    friend class DMControl;

public:
    // Constructor
    Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id);

    // State getters
    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_torque() const { return state_tau_; }
    int get_state_tmos() const { return state_tmos_; }
    int get_state_trotor() const { return state_trotor_; }

    // Motor property getters
    uint32_t get_send_can_id() const { return send_can_id_; }
    uint32_t get_recv_can_id() const { return recv_can_id_; }
    MotorType get_motor_type() const { return motor_type_; }

    // Enable status getters
    bool is_enabled() const { return enabled_; }

    // Status/error code reported by the motor in D[0] of every state frame.
    // Raw nibble rather than MotorError, because 0x2-0x7 and 0xF are unassigned
    // and casting an unknown code to the enum would lose it.
    uint8_t get_error_code() const { return error_code_; }
    bool has_error() const { return error_code_ >= MOTOR_ERROR_THRESHOLD; }

    // Parameter methods
    double get_param(int RID) const;

    // Static methods for motor properties
    static LimitParam get_limit_param(MotorType motor_type);

protected:
    // State update methods
    void update_state(double q, double dq, double tau, int tmos, int trotor);
    void set_state_tmos(int tmos);
    void set_state_trotor(int trotor);
    void set_enabled(bool enabled);
    void set_error_code(uint8_t code);
    void set_temp_param(int RID, double val);

    // Motor identifiers
    uint32_t send_can_id_;
    uint32_t recv_can_id_;
    MotorType motor_type_;

    // Enable status
    bool enabled_;

    // Latest status/error code from D[0]. Placed here so it occupies existing
    // padding, keeping sizeof(Motor) and every other member offset unchanged.
    uint8_t error_code_;

    // Current state
    double state_q_, state_dq_, state_tau_;
    int state_tmos_, state_trotor_;

    // Parameter storage
    std::map<int, double> temp_param_dict_;
};
}  // namespace openarm::damiao_motor
