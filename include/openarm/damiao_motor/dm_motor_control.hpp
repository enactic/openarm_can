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

#include <linux/can.h>

#include <cstdint>
#include <cstring>  // for memcpy
#include <iostream>
#include <map>
#include <vector>

#include "dm_motor.hpp"
#include "dm_motor_constants.hpp"

namespace openarm::damiao_motor {
// Forward declarations
class Motor;

struct ParamResult {
    int rid;
    double value;
    bool valid;
};

struct StateResult {
    double position;
    double velocity;
    double torque;
    int t_mos;
    int t_rotor;
    bool valid;
};

struct CANPacket {
    uint32_t send_can_id;
    std::vector<uint8_t> data;
};

struct MITParam {
    double kp;
    double kd;
    double q;
    double dq;
    double tau;
};

CANPacket create_enable_command(const Motor& motor);
CANPacket create_disable_command(const Motor& motor);
CANPacket create_set_zero_command(const Motor& motor);
CANPacket create_mit_control_command(const Motor& motor, const MITParam& mit_param);
CANPacket create_query_param_command(const Motor& motor, int RID);
CANPacket create_refresh_command(const Motor& motor);

class CanPacketDecoder {
public:
    static StateResult parse_motor_state_data(const Motor& motor, const std::vector<uint8_t>& data);
    static ParamResult parse_motor_param_data(const std::vector<uint8_t>& data);

private:
    static double uint_to_double(uint16_t x, double min, double max, int bits);
    static float uint8s_to_float(const std::array<uint8_t, 4>& bytes);
    static uint32_t uint8s_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);
    static bool is_in_ranges(int number);
};

}  // namespace openarm::damiao_motor
