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

#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <stdexcept>
#include <string>

namespace openarm::damiao_motor {

// Constructor
Motor::Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id)
    : send_can_id_(send_can_id),
      recv_can_id_(recv_can_id),
      motor_type_(motor_type),
      enabled_(false),
      state_id_(0),
      state_err_(0),
      state_raw_q_(0),
      state_raw_dq_(0),
      state_raw_tau_(0),
      state_q_(0.0),
      state_dq_(0.0),
      state_tau_(0.0),
      state_tmos_(0),
      state_trotor_(0) {}

// Enable methods
void Motor::set_enabled(bool enable) { this->enabled_ = enable; }

// Parameter methods
// TODO: storing temp params in motor object might not be a good idea
// also -1 is not a good default value, consider using a different value
double Motor::get_param(int RID) const {
    auto it = temp_param_dict_.find(RID);
    return (it != temp_param_dict_.end()) ? it->second : -1;
}

void Motor::set_temp_param(int RID, int val) { temp_param_dict_[RID] = val; }

// State update methods
void Motor::update_state(int id, int err, uint16_t raw_q, uint16_t raw_dq, uint16_t raw_tau,
                         double q, double dq, double tau, int tmos, int trotor) {
    state_id_ = id;
    state_err_ = err;
    state_raw_q_ = raw_q;
    state_raw_dq_ = raw_dq;
    state_raw_tau_ = raw_tau;
    state_q_ = q;
    state_dq_ = dq;
    state_tau_ = tau;
    state_tmos_ = tmos;
    state_trotor_ = trotor;
}

void Motor::set_state_tmos(int tmos) { state_tmos_ = tmos; }

void Motor::set_state_trotor(int trotor) { state_trotor_ = trotor; }

// Static methods
LimitParam Motor::get_limit_param(MotorType motor_type) {
    size_t index = static_cast<size_t>(motor_type);
    if (index >= MOTOR_LIMIT_PARAMS.size()) {
        throw std::invalid_argument("Invalid motor type: " +
                                    std::to_string(static_cast<int>(motor_type)));
    }
    return MOTOR_LIMIT_PARAMS[index];
}

}  // namespace openarm::damiao_motor
