// Copyright 2026 Enactic, Inc.
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

// ============================================================================
// RID metadata: description, R/W, type, range
// ============================================================================
struct RIDInfo {
    std::string description;
    std::string rw;
    std::string type;
    std::string range;
};

static const std::map<openarm::damiao_motor::RID, RIDInfo> RID_INFO = {
    {openarm::damiao_motor::RID::UV_Value,
     {"Under-Voltage Threshold", "RW", "float", "(10.0, fmax]"}},
    {openarm::damiao_motor::RID::KT_Value,
     {"KT Value (Torque Const)", "RW", "float", "[0.0, fmax]"}},
    {openarm::damiao_motor::RID::OT_Value, {"Over-Temp Threshold", "RW", "float", "[80.0, 200)"}},
    {openarm::damiao_motor::RID::OC_Value, {"Over-Current Threshold", "RW", "float", "(0.0, 1.0)"}},
    {openarm::damiao_motor::RID::ACC, {"Acceleration", "RW", "float", "(0.0, fmax)"}},
    {openarm::damiao_motor::RID::DEC, {"Deceleration", "RW", "float", "[-fmax, 0.0)"}},
    {openarm::damiao_motor::RID::MAX_SPD, {"Max Speed", "RW", "float", "(0.0, fmax]"}},
    {openarm::damiao_motor::RID::MST_ID, {"Master ID", "RW", "uint32", "[0, 0x7FF]"}},
    {openarm::damiao_motor::RID::ESC_ID, {"Motor (ESC) ID", "RW", "uint32", "[0, 0x7FF]"}},
    {openarm::damiao_motor::RID::TIMEOUT, {"CAN Timeout", "RW", "uint32", "[0, 2^32-1]"}},
    {openarm::damiao_motor::RID::CTRL_MODE, {"Control Mode", "RW", "uint32", "[0, 4]"}},
    {openarm::damiao_motor::RID::Damp, {"Damping Ratio", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::Inertia, {"Inertia", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::hw_ver, {"Hardware Version", "RO", "uint32", "/"}},
    {openarm::damiao_motor::RID::sw_ver, {"Software Version", "RO", "uint32", "/"}},
    {openarm::damiao_motor::RID::SN, {"Serial Number", "RO", "uint32", "/"}},
    {openarm::damiao_motor::RID::NPP, {"Number of Pole Pairs", "RO", "uint32", "/"}},
    {openarm::damiao_motor::RID::Rs, {"Stator Resistance (Rs)", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::LS, {"Stator Inductance (Ls)", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::Flux, {"Rotor Flux", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::Gr, {"Gear Ratio", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::PMAX, {"Position Limit (PMAX)", "RW", "float", "(0.0, fmax]"}},
    {openarm::damiao_motor::RID::VMAX, {"Velocity Limit (VMAX)", "RW", "float", "(0.0, fmax]"}},
    {openarm::damiao_motor::RID::TMAX, {"Torque Limit (TMAX)", "RW", "float", "(0.0, fmax]"}},
    {openarm::damiao_motor::RID::I_BW, {"Current Loop Bandwidth", "RW", "float", "[100.0, 1.0e4]"}},
    {openarm::damiao_motor::RID::KP_ASR, {"Speed Loop KP", "RW", "float", "[0.0, fmax]"}},
    {openarm::damiao_motor::RID::KI_ASR, {"Speed Loop KI", "RW", "float", "[0.0, fmax]"}},
    {openarm::damiao_motor::RID::KP_APR, {"Position Loop KP", "RW", "float", "[0.0, fmax]"}},
    {openarm::damiao_motor::RID::KI_APR, {"Position Loop KI", "RW", "float", "[0.0, fmax]"}},
    {openarm::damiao_motor::RID::OV_Value, {"Over-Voltage Threshold", "RW", "float", "TBD"}},
    {openarm::damiao_motor::RID::GREF, {"Gear Torque Efficiency", "RW", "float", "(0.0, 1.0]"}},
    {openarm::damiao_motor::RID::Deta, {"Speed Loop Damping", "RW", "float", "[1.0, 30.0]"}},
    {openarm::damiao_motor::RID::V_BW, {"Velocity Loop Bandwidth", "RW", "float", "(0.0, 500.0)"}},
    {openarm::damiao_motor::RID::IQ_c1, {"Current Loop C1", "RW", "float", "[100.0, 1.0e4]"}},
    {openarm::damiao_motor::RID::VL_c1, {"Velocity Loop C1", "RW", "float", "(0.0, 1.0e4]"}},
    {openarm::damiao_motor::RID::can_br, {"CAN Baudrate", "RW", "uint32", "[0, 9]"}},
    {openarm::damiao_motor::RID::sub_ver, {"Sub Version", "RO", "uint32", "/"}},
    {openarm::damiao_motor::RID::u_off, {"I-Phase U Offset", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::v_off, {"I-Phase V Offset", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::k1, {"Gain K1", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::k2, {"Gain K2", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::m_off, {"Mechanical Offset", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::dir, {"Motor Direction", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::p_m, {"Motor Position", "RO", "float", "/"}},
    {openarm::damiao_motor::RID::xout, {"Output Shaft Position", "RO", "float", "/"}},
};

static const std::map<int, std::string> CTRL_MODE_NAMES = {
    {1, "MIT"},
    {2, "POS_VEL"},
    {3, "VEL"},
    {4, "POS_FORCE"},
};

static const std::map<int, std::string> CAN_BR_NAMES = {
    {0, "125K"}, {1, "200K"}, {2, "250K"}, {3, "500K"}, {4, "1M"},  {5, "2M"},
    {6, "2.5M"}, {7, "3.2M"}, {8, "4M"},   {9, "5M"},   {10, "8M"}, {11, "10M"},
};

// ============================================================================
// Helpers
// ============================================================================
static void print_no_response_hint(uint32_t sid, uint32_t recv_id) {
    std::cout << "  [!] NO RESPONSE FROM MOTOR - possible causes:\n";
    std::cout << "      - CAN cable not connected\n";
    std::cout << "      - Motor power not on\n";
    std::cout << "      - Baudrate mismatch (run 'discover' to find correct baudrate, 1Mbps is "
                 "default)\n";
    std::cout << "      - Wrong motor ID (Send: 0x" << std::hex << sid << ", Recv: 0x" << recv_id
              << std::dec << ")\n";
}

// ============================================================================
// Main function
// ============================================================================
int run_read_params(const std::string& interface, bool use_arm_ids,
                    const std::vector<std::string>& custom_ids_str) {
    std::vector<uint32_t> send_ids;

    if (use_arm_ids) {
        for (uint32_t i = 1; i <= 8; ++i) send_ids.push_back(i);
    }
    for (const auto& id_str : custom_ids_str) {
        try {
            send_ids.push_back(std::stoul(id_str, nullptr, 0));
        } catch (...) {
            std::cerr << "✗ Error: Invalid ID '" << id_str << "'\n";
            return 1;
        }
    }

    if (send_ids.empty()) {
        std::cerr << "✗ Error: No target IDs specified.\n";
        return 1;
    }

    try {
        std::cout << ">>> Connecting to " << interface << " (FD Mode)..." << std::endl;
        openarm::can::socket::OpenArm openarm(interface, true);

        std::vector<openarm::damiao_motor::MotorType> types(
            send_ids.size(), openarm::damiao_motor::MotorType::DM4310);
        std::vector<uint32_t> recv_ids;
        for (auto id : send_ids) recv_ids.push_back(id + 0x10);

        openarm.init_arm_motors(types, send_ids, recv_ids);
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

        std::cout << ">>> Querying all registers for " << send_ids.size()
                  << " motors. Please wait...\n";

        for (int r = 0; r < (int)openarm::damiao_motor::RID::COUNT; ++r) {
            openarm.query_param_all(r);
            std::this_thread::sleep_for(std::chrono::milliseconds(35));
            openarm.recv_all();
        }

        const auto& motors = openarm.get_arm().get_motors();
        for (size_t i = 0; i < motors.size(); ++i) {
            uint32_t sid = send_ids[i];
            std::cout << "\n==================================================\n";
            std::cout << " MOTOR ID: 0x" << std::hex << sid << std::dec << " (Response ID: 0x"
                      << std::hex << recv_ids[i] << std::dec << ")\n";
            std::cout << "==================================================\n";

            // Basic connectivity check
            double mst = motors[i].get_param((int)openarm::damiao_motor::RID::MST_ID);
            if (!std::isfinite(mst) || mst == -1.0) {
                print_no_response_hint(sid, recv_ids[i]);
                continue;
            }

            // Header
            std::cout << std::left << std::setw(30) << "Parameter" << std::setw(5) << "R/W"
                      << std::setw(10) << "Type" << std::setw(20) << "Range"
                      << "Value\n";
            std::cout << std::string(80, '-') << "\n";

            int param_count = 0;
            for (int r = 0; r < (int)openarm::damiao_motor::RID::COUNT; ++r) {
                double val = motors[i].get_param(r);
                if (!std::isfinite(val) || val == -1.0) continue;

                param_count++;
                using namespace openarm::damiao_motor;
                RID reg = static_cast<RID>(r);

                // Metadata
                std::string desc = "Register " + std::to_string(r);
                std::string rw = "??";
                std::string type = "??";
                std::string range = "??";

                auto it = RID_INFO.find(reg);
                if (it != RID_INFO.end()) {
                    desc = it->second.description;
                    rw = it->second.rw;
                    type = it->second.type;
                    range = it->second.range;
                }

                std::cout << std::left << std::setw(30) << desc << std::setw(5) << rw
                          << std::setw(10) << type << std::setw(20) << range;

                // Value formatting
                if (reg == RID::CTRL_MODE) {
                    int code = static_cast<int>(val);
                    auto m = CTRL_MODE_NAMES.find(code);
                    std::cout << code << " ("
                              << (m != CTRL_MODE_NAMES.end() ? m->second : "Unknown") << ")";
                } else if (reg == RID::can_br) {
                    int code = static_cast<int>(val);
                    auto m = CAN_BR_NAMES.find(code);
                    std::cout << code << " (" << (m != CAN_BR_NAMES.end() ? m->second : "Unknown")
                              << ")";
                } else if (reg == RID::MST_ID || reg == RID::ESC_ID || reg == RID::TIMEOUT ||
                           reg == RID::hw_ver || reg == RID::sw_ver || reg == RID::SN ||
                           reg == RID::NPP) {
                    std::cout << std::fixed << std::setprecision(0) << val;
                } else if (reg == RID::Inertia || reg == RID::Damp || reg == RID::Rs ||
                           reg == RID::LS || reg == RID::Flux) {
                    std::cout << std::scientific << std::setprecision(6) << val;
                } else {
                    std::cout << std::fixed << std::setprecision(4) << val;
                }

                std::cout << std::defaultfloat << "\n";
            }

            // No params returned despite MST_ID passing
            if (param_count == 0) {
                print_no_response_hint(sid, recv_ids[i]);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "✗ System Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

}  // namespace openarm::cli
