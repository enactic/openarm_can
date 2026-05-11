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
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

struct BaudSetting {
    int bitrate;
    int dbitrate;
    std::string label;
};

// All supported baudrates
static const std::map<int, BaudSetting> ALL_BAUDRATE_MAP = {
    {0, {125000, 125000, "125 kbps"}},        {1, {200000, 200000, "200 kbps"}},
    {2, {250000, 250000, "250 kbps"}},        {3, {500000, 500000, "500 kbps"}},
    {4, {1000000, 1000000, "1 Mbps"}},        {5, {1000000, 2000000, "2 Mbps (FD)"}},
    {6, {1000000, 2500000, "2.5 Mbps (FD)"}}, {7, {1000000, 3200000, "3.2 Mbps (FD)"}},
    {8, {1000000, 4000000, "4 Mbps (FD)"}},   {9, {1000000, 5000000, "5 Mbps (FD)"}},
    {10, {1000000, 8000000, "8 Mbps (FD)"}},  {11, {1000000, 10000000, "10 Mbps (FD)"}}};

// Default: commonly used baudrates only
static const std::vector<int> DEFAULT_BAUD_CODES = {4, 9, 10, 11};  // 1M, 5M, 8M, 10M

struct DiscoveredMotor {
    uint32_t send_id;
    uint32_t recv_id;
    int baud_code;
    std::string baud_label;

    bool operator<(const DiscoveredMotor& other) const {
        if (send_id != other.send_id) return send_id < other.send_id;
        if (recv_id != other.recv_id) return recv_id < other.recv_id;
        return baud_code < other.baud_code;
    }
};

// Helper: Visual progress bar
void print_progress(int current, int total, const std::string& info) {
    float progress = (float)current / total;
    int barWidth = 30;
    std::cout << "\r[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << "% | " << info << std::flush;
}

// Helper: Reconfigure CAN interface baudrate
bool reconfigure_can_interface(const std::string& iface, int br, int dbr) {
    std::string cmd = "sudo ip link set " + iface + " type can bitrate " + std::to_string(br) +
                      " sample-point 0.75";

    if (br != dbr) {
        cmd += " dbitrate " + std::to_string(dbr) + " fd on dsample-point 0.60 dsjw 1";
    }
    cmd += " restart-ms 100 2>/dev/null";

    (void)system(("sudo ip link set " + iface + " down 2>/dev/null").c_str());
    int res = system(cmd.c_str());
    (void)system(("sudo ip link set " + iface + " up 2>/dev/null").c_str());

    return (res == 0);
}

int run_discover(const std::string& interface, int max_id, bool full_scan) {
    std::set<DiscoveredMotor> found_motors;

    // Select baudrates to scan
    std::vector<int> baud_codes;
    if (full_scan) {
        for (const auto& [code, _] : ALL_BAUDRATE_MAP) baud_codes.push_back(code);
    } else {
        baud_codes = DEFAULT_BAUD_CODES;
    }
    const int total_bauds = static_cast<int>(baud_codes.size());

    std::cout << "=========================================================\n";
    std::cout << " OPENARM DEEP DISCOVERY MODE\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << " Mode: " << (full_scan ? "Full scan (12 baudrates)" : "Fast scan (1M/5M/8M/10M)")
              << "\n";
    std::cout << " [Timing] SP: 0.75 / DSP: 0.60 / DSJW: 1\n";
    std::cout << " Scanning Range: 0x01 to " << format_hex_id(max_id) << "\n";
    std::cout << "=========================================================\n\n";

    for (int bi = 0; bi < total_bauds; ++bi) {
        int b = baud_codes[bi];
        auto setting = ALL_BAUDRATE_MAP.at(b);
        print_progress(bi, total_bauds, "Testing " + setting.label + "...");

        if (!reconfigure_can_interface(interface, setting.bitrate, setting.dbitrate)) continue;

        // Wait for interface to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        for (int id = 1; id <= max_id; ++id) {
            uint32_t recv_candidates[2] = {(uint32_t)(id + 0x10), 0x00};

            for (uint32_t rid : recv_candidates) {
                try {
                    openarm::can::socket::OpenArm openarm(interface,
                                                          (setting.bitrate != setting.dbitrate));

                    openarm.init_arm_motors({openarm::damiao_motor::MotorType::DM4310},
                                            {(uint32_t)id}, {rid});
                    openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

                    bool detected = false;
                    for (int retry = 0; retry < 2; ++retry) {
                        openarm.get_arm().query_param_all((int)openarm::damiao_motor::RID::MST_ID);

                        for (int k = 0; k < 2; ++k) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(15));
                            openarm.recv_all();
                        }

                        double val = openarm.get_arm().get_motor(0).get_param(
                            (int)openarm::damiao_motor::RID::MST_ID);
                        if (std::isfinite(val) && val != -1.0) {
                            detected = true;
                            break;
                        }
                    }

                    if (detected) {
                        found_motors.insert({(uint32_t)id, rid, b, setting.label});
                    }
                } catch (...) {
                }
            }
        }
    }

    print_progress(total_bauds, total_bauds, "Scan Complete!            \n\n");

    if (found_motors.empty()) {
        std::cout << "[!] No motors detected. Check wiring and power.\n";
    } else {
        std::cout << "=========================================================\n";
        std::cout << " DISCOVERY SUMMARY (Total: " << found_motors.size() << " motors found)\n";
        std::cout << "---------------------------------------------------------\n";
        std::cout << std::left << std::setw(12) << "Send ID" << std::setw(12) << "Recv ID"
                  << "Internal Baudrate Setting\n";
        std::cout << "---------------------------------------------------------\n";

        for (const auto& m : found_motors) {
            std::cout << std::left << std::setw(12) << format_hex_id(m.send_id) << std::setw(12)
                      << format_hex_id(m.recv_id) << m.baud_label << " (Code: " << m.baud_code
                      << ")\n";
        }
        std::cout << "=========================================================\n";

        std::cout << "=========================================================\n";
        std::cout << "⚠️  WARNING: CAN interface is now configured at 10 Mbps (FD).\n";
        std::cout << "   Run 'can_configure' to restore the baudrate for your motors.\n";
        std::cout << "   e.g. openarm-can -i " << interface << " can_configure\n";
        std::cout << "=========================================================\n";
    }
    return 0;
}

}  // namespace openarm::cli
