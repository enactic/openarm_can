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

#include <array>
#include <cstddef>
#include <cstdint>

namespace openarm::damiao_motor {
enum class MotorType : uint8_t {
    DM3507 = 0,
    DM4310 = 1,
    DM4310_48V = 2,
    DM4340 = 3,
    DM4340_48V = 4,
    DM6006 = 5,
    DM8006 = 6,
    DM8009 = 7,
    DM10010L = 8,
    DM10010 = 9,
    DMH3510 = 10,
    DMH6215 = 11,
    DMG6220 = 12,
    COUNT = 13
};

enum class ControlMode : uint8_t { MIT = 1, POS_VEL = 2, VEL = 3, POS_FORCE = 4 };

// Status/error code carried in the upper nibble of D[0] of every state feedback
// frame: D[0] = ID | (ERR << 4). Codes 0x0 and 0x1 are normal operating states,
// 0x8 and above are faults.
//
// See also:
// https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/
//
// The motor drives its indicator lamp from this same code, so a value read here
// can be cross-checked against the hardware.
enum class MotorError : uint8_t {
    DISABLED = 0x0,
    ENABLED = 0x1,
    OVERVOLTAGE = 0x8,
    UNDERVOLTAGE = 0x9,
    OVERCURRENT = 0xA,
    MOS_OVERHEAT = 0xB,
    COIL_OVERHEAT = 0xC,
    COMMUNICATION_LOST = 0xD,
    OVERLOAD = 0xE
};

// Codes at or above this are faults; below it the motor is only reporting
// whether it is enabled.
inline constexpr uint8_t MOTOR_ERROR_THRESHOLD = 0x8;

// 0x2-0x7 and 0xF are unassigned, so the raw nibble is what gets stored and only
// the defined codes are named here.
inline const char* motor_error_to_string(uint8_t code) {
    switch (code) {
        case 0x0:
            return "DISABLED";
        case 0x1:
            return "ENABLED";
        case 0x8:
            return "OVERVOLTAGE";
        case 0x9:
            return "UNDERVOLTAGE";
        case 0xA:
            return "OVERCURRENT";
        case 0xB:
            return "MOS_OVERHEAT";
        case 0xC:
            return "COIL_OVERHEAT";
        case 0xD:
            return "COMMUNICATION_LOST";
        case 0xE:
            return "OVERLOAD";
        default:
            return "UNKNOWN";
    }
}

enum class RID : uint8_t {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
    COUNT = 82
};

// Limit parameters structure for different motor types
struct LimitParam {
    double pMax;  // Position limit (rad)
    double vMax;  // Velocity limit (rad/s)
    double tMax;  // Torque limit (Nm)
};
// Limit parameters for each motor type [pMax, vMax, tMax]
inline constexpr std::array<LimitParam, static_cast<std::size_t>(MotorType::COUNT)>
    MOTOR_LIMIT_PARAMS = {{
        {12.5, 50, 5},    // DM3507
        {12.5, 30, 10},   // DM4310
        {12.5, 50, 10},   // DM4310_48V
        {12.5, 10, 28},   // DM4340
        {12.5, 10, 28},   // DM4340_48V
        {12.5, 45, 20},   // DM6006
        {12.5, 45, 40},   // DM8006
        {12.5, 45, 54},   // DM8009
        {12.5, 25, 200},  // DM10010L
        {12.5, 20, 200},  // DM10010
        {12.5, 280, 1},   // DMH3510
        {12.5, 45, 10},   // DMH6215
        {12.5, 45, 10}    // DMG6220
    }};
}  // namespace openarm::damiao_motor
