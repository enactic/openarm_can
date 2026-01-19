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
    int get_state_id() const { return state_id_; }
    int get_state_err() const { return state_err_; }
    uint16_t get_raw_position() const { return state_raw_q_; }
    uint16_t get_raw_velocity() const { return state_raw_dq_; }
    uint16_t get_raw_torque() const { return state_raw_tau_; }
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

    // Parameter methods
    double get_param(int RID) const;

    // Static methods for motor properties
    static LimitParam get_limit_param(MotorType motor_type);

protected:
    // State update methods
    void update_state(int id, int err, uint16_t raw_q, uint16_t raw_dq, uint16_t raw_tau,
                      double q, double dq, double tau, int tmos, int trotor);

    void set_state_tmos(int tmos);
    void set_state_trotor(int trotor);
    void set_enabled(bool enabled);
    void set_temp_param(int RID, int val);

    // Motor identifiers
    uint32_t send_can_id_;
    uint32_t recv_can_id_;
    MotorType motor_type_;

    // Enable status
    bool enabled_;

    // Current state
    int state_id_, state_err_;
    uint16_t state_raw_q_, state_raw_dq_, state_raw_tau_;
    double state_q_, state_dq_, state_tau_;
    int state_tmos_, state_trotor_;

    // Parameter storage
    std::map<int, double> temp_param_dict_;
};
}  // namespace openarm::damiao_motor
