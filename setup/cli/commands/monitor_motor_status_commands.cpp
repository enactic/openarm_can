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

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <sstream>
#include <thread>
#include <vector>

#include "cli.hpp"

static std::atomic<bool> g_monitor_running{true};
static void monitor_sigint_handler(int) { g_monitor_running = false; }

namespace openarm::cli {

int run_monitor(const std::string& interface, bool use_arm_ids,
                const std::vector<std::string>& custom_ids_str, int interval_ms, int duration_ms) {
    std::vector<uint32_t> send_ids;
    if (use_arm_ids)
        for (uint32_t i = 1; i <= 8; ++i) send_ids.push_back(i);
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

    // Startup summary
    std::cout << "=========================================================\n";
    std::cout << " OPENARM MONITOR\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << " Interface : " << interface << "\n";
    std::cout << " Motors    :";
    for (auto id : send_ids)
        std::cout << " 0x" << std::hex << std::setfill('0') << std::setw(2) << id;
    std::cout << std::dec << std::setfill(' ') << "\n";
    std::cout << " Interval  : " << interval_ms << " ms\n";
    std::cout << " Duration  : " << duration_ms << " ms\n";
    std::cout << " Ctrl+C    : stop early and disarm motors\n";
    std::cout << "=========================================================\n\n";

    // Format a float; returns "---" padded to width if not finite (motor not responding)
    auto fmtval = [](double v, int width, int prec) -> std::string {
        if (!std::isfinite(v)) {
            std::string s = "---";
            s.resize(width, ' ');
            return s;
        }
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec) << std::left << std::setw(width) << v;
        return ss.str();
    };

    try {
        openarm::can::socket::OpenArm openarm(interface, true);
        std::vector<openarm::damiao_motor::MotorType> types(
            send_ids.size(), openarm::damiao_motor::MotorType::DM4310);
        std::vector<uint32_t> recv_ids;
        for (auto id : send_ids) recv_ids.push_back(id + 0x10);

        openarm.init_arm_motors(types, send_ids, recv_ids);

        // --- STEP 1: Enable motors for monitoring ---
        std::cout << ">>> Enabling motors..." << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();

        g_monitor_running = true;
        std::signal(SIGINT, monitor_sigint_handler);

        auto start_time = std::chrono::steady_clock::now();

        while (g_monitor_running) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
            if (elapsed >= duration_ms) break;

            openarm.refresh_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            openarm.recv_all();

            std::cout << "\033[2J\033[1;1H";

            std::cout << "==========================================================="
                         "===========\n";
            std::cout << "  OPENARM MONITOR | " << interface << " | " << std::fixed
                      << std::setprecision(1) << (double)elapsed / 1000.0 << "s / "
                      << (double)duration_ms / 1000.0 << "s  [Ctrl+C to stop]\n";
            std::cout << "==========================================================="
                         "===========\n";
            std::cout << std::left << std::setw(8) << "ID" << std::setw(14) << "Pos(rad)"
                      << std::setw(14) << "Vel(rad/s)" << std::setw(14) << "Torque(Nm)"
                      << std::setw(10) << "MOS(C)" << "Rtr(C)\n";
            std::cout << "-----------------------------------------------------------"
                         "-----------\n";

            const auto& motors = openarm.get_arm().get_motors();
            for (size_t i = 0; i < motors.size(); ++i) {
                const auto& m = motors[i];
                std::ostringstream id_ss;
                id_ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << send_ids[i];
                std::cout << std::left << std::setfill(' ') << std::setw(8) << id_ss.str()
                          << fmtval(m.get_position(), 14, 3) << fmtval(m.get_velocity(), 14, 3)
                          << fmtval(m.get_torque(), 14, 3) << fmtval(m.get_state_tmos(), 10, 1)
                          << fmtval(m.get_state_trotor(), 8, 1) << "\n";
            }
            std::cout << "==========================================================="
                         "===========\n";
            std::flush(std::cout);

            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, interval_ms - 20)));
        }

        std::signal(SIGINT, SIG_DFL);

        // --- STEP 2: Disable motors before exiting ---
        std::cout << (g_monitor_running ? "\n>>> Monitoring complete." : "\n>>> Interrupted.")
                  << " Disabling motors..." << std::endl;
        openarm.disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::cout << ">>> All motors DISARMED safely.\n";

    } catch (const std::exception& e) {
        std::cerr << "✗ Monitor Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

}  // namespace openarm::cli
