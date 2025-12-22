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

#include <array>
#include <cstdint>
#include <cstring>  // for memcpy
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

struct PosVelParam {
    double q;
    double dq;
};

struct PosForceParam {
    double q;   // Position command in rad.
    double dq;  // Absolute speed limit in rad/s, scaled by 100 into uint16 when packed.
    double i;   // Torque current limit per-unit (0-1), scaled by 10000 into uint16 when packed.
};

class CanPacketEncoder {
public:
    static CANPacket create_enable_command(const Motor& motor);
    static CANPacket create_disable_command(const Motor& motor);
    static CANPacket create_set_zero_command(const Motor& motor);
    static CANPacket create_mit_control_command(const Motor& motor, const MITParam& mit_param);
    static CANPacket create_posvel_control_command(const Motor& motor,
                                                   const PosVelParam& posvel_param);
    static CANPacket create_posforce_control_command(const Motor& motor,
                                                     const PosForceParam& posforce_param);
    static CANPacket create_set_control_mode_command(const Motor& motor, ControlMode mode);
    static CANPacket create_query_param_command(const Motor& motor, int RID);
    static CANPacket create_refresh_command(const Motor& motor);

private:
    static std::vector<uint8_t> pack_mit_control_data(MotorType motor_type,
                                                      const MITParam& mit_param);
    static std::vector<uint8_t> pack_posvel_control_data(MotorType motor_type,
                                                         const PosVelParam& posvel_param);
    static std::vector<uint8_t> pack_posforce_control_data(MotorType motor_type,
                                                           const PosForceParam& posforce_param);

    /**
     * @brief pack frame for querying parameter
     * @param send_can_id the send can id
     * @param RID the register id
     * @return the packed frame
     *
     * The frame is packed as follows:
     * ID   	D[0]	    D[1]	    D[2]	D[3]	D[4:7]
     * 0x7FF	CAN_ID_L	CAN_ID_H	0x33	RID	    0x00 0x00 0x00 0x00
     */
    static std::vector<uint8_t> pack_query_param_data(uint32_t send_can_id, int RID);

    /**
     * @brief pack frame for writing parameter
     * @param send_can_id the send can id
     * @param RID the register id
     * @param value the value to write
     * @return the packed frame
     *
     * The frame is packed as follows:
     * ID   	D[0]	    D[1]	    D[2]	D[3]	D[4:7]
     * 0x7FF	CAN_ID_L	CAN_ID_H	0x55	RID	    value
     */
    template <typename T>
    static std::vector<uint8_t> pack_write_param_data(uint32_t send_can_id, int RID, T value);
    static std::vector<uint8_t> pack_command_data(uint8_t cmd);

    static double limit_min_max(double x, double min, double max);
    static uint16_t double_to_uint(double x, double x_min, double x_max, int bits);
    static std::array<uint8_t, 4> float_to_uint8s(float value);
};

template <typename T>
std::vector<uint8_t> CanPacketEncoder::pack_write_param_data(uint32_t send_can_id, int RID,
                                                             T value) {
    static_assert(sizeof(T) <= 4, "Value type must be 4 bytes or less");

    std::array<uint8_t, 4> value_bytes{};
    std::memcpy(value_bytes.data(), &value, sizeof(T));

    return {static_cast<uint8_t>(send_can_id & 0xFF),
            static_cast<uint8_t>((send_can_id >> 8) & 0xFF),
            0x55,
            static_cast<uint8_t>(RID),
            value_bytes[0],
            value_bytes[1],
            value_bytes[2],
            value_bytes[3]};
}

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
