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

#include <cmath>
#include <cstring>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <thread>

namespace openarm::damiao_motor {

// Command creation methods (return data array, can_id handled externally)
CANPacket CanPacketEncoder::create_enable_command(const Motor& motor) {
    return {motor.get_send_can_id(), pack_command_data(0xFC)};
}

CANPacket CanPacketEncoder::create_disable_command(const Motor& motor) {
    return {motor.get_send_can_id(), pack_command_data(0xFD)};
}

CANPacket CanPacketEncoder::create_set_zero_command(const Motor& motor) {
    return {motor.get_send_can_id(), pack_command_data(0xFE)};
}

CANPacket CanPacketEncoder::create_mit_control_command(const Motor& motor,
                                                       const MITParam& mit_param) {
    return {motor.get_send_can_id(), pack_mit_control_data(motor.get_motor_type(), mit_param)};
}

CANPacket CanPacketEncoder::create_posvel_control_command(const Motor& motor,
                                                          const PosVelParam& posvel_param) {
    // pos vel mode needs extra 0x100
    return {motor.get_send_can_id() + 0x100,
            pack_posvel_control_data(motor.get_motor_type(), posvel_param)};
}

CANPacket CanPacketEncoder::create_query_param_command(const Motor& motor, int RID) {
    return {0x7FF, pack_query_param_data(motor.get_send_can_id(), RID)};
}

CANPacket CanPacketEncoder::create_refresh_command(const Motor& motor) {
    uint8_t send_can_id = motor.get_send_can_id();
    std::vector<uint8_t> data = {static_cast<uint8_t>(send_can_id & 0xFF),
                                 static_cast<uint8_t>((send_can_id >> 8) & 0xFF),
                                 0xCC,
                                 0x00,
                                 0x00,
                                 0x00,
                                 0x00,
                                 0x00};
    return {0x7FF, data};
}

// Data interpretation methods (use recv_can_id for received data)
StateResult CanPacketDecoder::parse_motor_state_data(const Motor& motor,
                                                     const std::vector<uint8_t>& data) {
    if (data.size() < 8) {
        std::cerr << "Warning: Skipping motor state data less than 8 bytes" << std::endl;
        return {0, 0, 0, 0, 0, false};
    }

    // Parse state data
    uint16_t q_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    uint16_t dq_uint =
        (static_cast<uint16_t>(data[3]) << 4) | (static_cast<uint16_t>(data[4]) >> 4);
    uint16_t tau_uint = (static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5];
    int t_mos = static_cast<int>(data[6]);
    int t_rotor = static_cast<int>(data[7]);

    // Convert to physical values
    LimitParam limits = MOTOR_LIMIT_PARAMS[static_cast<int>(motor.get_motor_type())];
    double recv_q = CanPacketDecoder::uint_to_double(q_uint, -limits.pMax, limits.pMax, 16);
    double recv_dq = CanPacketDecoder::uint_to_double(dq_uint, -limits.vMax, limits.vMax, 12);
    double recv_tau = CanPacketDecoder::uint_to_double(tau_uint, -limits.tMax, limits.tMax, 12);

    return {recv_q, recv_dq, recv_tau, t_mos, t_rotor, true};
}

ParamResult CanPacketDecoder::parse_motor_param_data(const std::vector<uint8_t>& data) {
    if (data.size() < 8) return {0, NAN, false};

    if ((data[2] == 0x33 || data[2] == 0x55)) {
        uint8_t RID = data[3];
        double num;
        if (CanPacketDecoder::is_in_ranges(RID)) {
            num = CanPacketDecoder::uint8s_to_uint32(data[4], data[5], data[6], data[7]);
        } else {
            std::array<uint8_t, 4> float_bytes = {data[4], data[5], data[6], data[7]};
            num = CanPacketDecoder::uint8s_to_float(float_bytes);
        }
        return {RID, num, true};
    } else {
        std::cerr << "WARNING: INVALID PARAM DATA" << std::endl;
        return {0, NAN, false};
    }
}

// Data packing utility methods
std::vector<uint8_t> CanPacketEncoder::pack_mit_control_data(MotorType motor_type,
                                                             const MITParam& mit_param) {
    uint16_t kp_uint = double_to_uint(mit_param.kp, 0, 500, 12);
    uint16_t kd_uint = double_to_uint(mit_param.kd, 0, 5, 12);

    // Get motor limits based on type
    LimitParam limits = MOTOR_LIMIT_PARAMS[static_cast<int>(motor_type)];
    uint16_t q_uint = double_to_uint(mit_param.q, -(double)limits.pMax, (double)limits.pMax, 16);
    uint16_t dq_uint = double_to_uint(mit_param.dq, -(double)limits.vMax, (double)limits.vMax, 12);
    uint16_t tau_uint =
        double_to_uint(mit_param.tau, -(double)limits.tMax, (double)limits.tMax, 12);

    return {static_cast<uint8_t>((q_uint >> 8) & 0xFF),
            static_cast<uint8_t>(q_uint & 0xFF),
            static_cast<uint8_t>(dq_uint >> 4),
            static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
            static_cast<uint8_t>(kp_uint & 0xFF),
            static_cast<uint8_t>(kd_uint >> 4),
            static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
            static_cast<uint8_t>(tau_uint & 0xFF)};
}

std::vector<uint8_t> CanPacketEncoder::pack_posvel_control_data(MotorType motor_type,
                                                                const PosVelParam& posvel_param) {
    double pos = posvel_param.q;
    double vel = posvel_param.dq;

    auto pb = float_to_uint8s(static_cast<float>(pos));
    auto vb = float_to_uint8s(static_cast<float>(vel));

    // Pack into 8 bytes: [pos(4)][vel(4)]
    return {pb[0], pb[1], pb[2], pb[3], vb[0], vb[1], vb[2], vb[3]};
}

std::vector<uint8_t> CanPacketEncoder::pack_query_param_data(uint32_t send_can_id, int RID) {
    return {static_cast<uint8_t>(send_can_id & 0xFF),
            static_cast<uint8_t>((send_can_id >> 8) & 0xFF),
            0x33,
            static_cast<uint8_t>(RID),
            0x00,
            0x00,
            0x00,
            0x00};
}

std::vector<uint8_t> CanPacketEncoder::pack_command_data(uint8_t cmd) {
    return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
}

// Utility function implementations
double CanPacketEncoder::limit_min_max(double x, double min, double max) {
    return std::max(min, std::min(x, max));
}

uint16_t CanPacketEncoder::double_to_uint(double x, double x_min, double x_max, int bits) {
    x = limit_min_max(x, x_min, x_max);
    double span = x_max - x_min;
    double data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

std::array<uint8_t, 4> CanPacketEncoder::float_to_uint8s(float value) {
    std::array<uint8_t, 4> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

double CanPacketDecoder::uint_to_double(uint16_t x, double min, double max, int bits) {
    double span = max - min;
    double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
    return data_norm * span + min;
}

float CanPacketDecoder::uint8s_to_float(const std::array<uint8_t, 4>& bytes) {
    float value;
    std::memcpy(&value, bytes.data(), sizeof(float));
    return value;
}

uint32_t CanPacketDecoder::uint8s_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3,
                                            uint8_t byte4) {
    uint32_t value;
    uint8_t bytes[4] = {byte1, byte2, byte3, byte4};
    std::memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}

bool CanPacketDecoder::is_in_ranges(int number) {
    return (7 <= number && number <= 10) || (13 <= number && number <= 16) ||
           (35 <= number && number <= 36);
}
}  // namespace openarm::damiao_motor
