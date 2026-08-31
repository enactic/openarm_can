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

#include <algorithm>
#include <array>
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

static std::atomic<bool> g_diagnose_running{true};
static void diagnose_sigint_handler(int) { g_diagnose_running = false; }

namespace openarm::cli {

namespace {

// Everything accumulated for one axis over the run. Sampling only reads values
// the motor already sends in its regular feedback, so nothing here costs an
// extra CAN frame.
struct AxisReport {
    uint32_t send_id = 0;

    // (1) measurements
    double tmos_min = 0, tmos_max = 0, tmos_last = 0;
    double trot_min = 0, trot_max = 0, trot_last = 0;
    double tau_abs_max = 0;

    // (2) status / error codes, counted per raw D[0] nibble
    std::array<uint32_t, 16> code_counts{};
    uint8_t last_code = 0;
    bool enabled = false;

    bool responded = false;
    // Response count already folded into the tallies above, so a sample is only
    // recorded when the motor actually answered since the last one.
    uint64_t seen_responses = 0;
};

// Prints one fault line, or nothing at all if the fault never happened. Keeping
// silent entries out is what lets a healthy bus report as a single line.
void print_counter(const char* label, const openarm::canbus::ErrorCounter& c,
                   std::chrono::steady_clock::time_point now) {
    if (!c) return;
    double ago = std::chrono::duration<double>(now - c.last).count();
    std::cout << "   " << std::left << std::setw(36) << label << "x" << std::setw(10) << c.count
              << "last " << std::fixed << std::setprecision(1) << ago << "s ago\n";
}

std::string pct(double v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << v * 100.0 << "%";
    return ss.str();
}

std::string ago(const openarm::damiao_motor::MotorLinkStats& st) {
    if (!st.ever_responded()) return "never";
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << std::chrono::duration<double>(st.since_last_response()).count() << "s ago";
    return ss.str();
}

std::string hex_id(uint32_t id) {
    std::ostringstream ss;
    ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << id;
    return ss.str();
}

std::string range_str(double lo, double hi, double last, bool valid) {
    if (!valid) return "   -           ";
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(0) << std::setw(3) << lo << " /" << std::setw(4) << hi
       << " /" << std::setw(4) << last;
    return ss.str();
}

// "ENABLED x997, OVERLOAD x3", most frequent first.
std::string codes_str(const AxisReport& a) {
    if (!a.responded) return "(no response)";

    std::vector<std::pair<uint32_t, uint8_t>> seen;
    for (uint8_t c = 0; c < 16; ++c)
        if (a.code_counts[c] > 0) seen.emplace_back(a.code_counts[c], c);
    if (seen.empty()) return "(no response)";
    std::sort(seen.rbegin(), seen.rend());

    std::ostringstream ss;
    for (size_t i = 0; i < seen.size(); ++i) {
        if (i) ss << ", ";
        ss << openarm::damiao_motor::motor_error_to_string(seen[i].second) << " x"
           << seen[i].first;
    }
    return ss.str();
}

}  // namespace

int run_diagnose(const std::string& interface, bool use_arm_ids,
                 const std::vector<std::string>& custom_ids_str, int duration_ms, int interval_ms) {
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
        std::cerr << "✗ Error: No target IDs specified. Use --arm or --id.\n";
        return 1;
    }

    std::cout << "=========================================================\n";
    std::cout << " OPENARM DIAGNOSE\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << " Interface : " << interface << "\n";
    std::cout << " Motors    :";
    for (auto id : send_ids) std::cout << " " << hex_id(id);
    std::cout << "\n";
    std::cout << " Duration  : " << duration_ms << " ms\n";
    std::cout << " Interval  : " << interval_ms << " ms\n";
    std::cout << "=========================================================\n\n";

    try {
        openarm::can::socket::OpenArm openarm(interface, true);
        std::vector<openarm::damiao_motor::MotorType> types(
            send_ids.size(), openarm::damiao_motor::MotorType::DM4310);
        std::vector<uint32_t> recv_ids;
        for (auto id : send_ids) recv_ids.push_back(id + 0x10);
        openarm.init_arm_motors(types, send_ids, recv_ids);

        std::cout << ">>> Enabling motors..." << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();

        std::vector<AxisReport> report(send_ids.size());
        for (size_t i = 0; i < send_ids.size(); ++i) report[i].send_id = send_ids[i];
        uint32_t samples = 0;

        g_diagnose_running = true;
        std::signal(SIGINT, diagnose_sigint_handler);
        std::cout << ">>> Sampling... (Ctrl+C to stop early)" << std::endl;

        auto start = std::chrono::steady_clock::now();
        while (g_diagnose_running) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
            if (elapsed >= duration_ms) break;

            openarm.refresh_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            openarm.recv_all();

            auto& arm = openarm.get_arm();
            const auto motors = arm.get_motors();
            for (size_t i = 0; i < motors.size() && i < report.size(); ++i) {
                const auto& m = motors[i];
                auto& a = report[i];

                // Record only what the motor actually said. Motor holds its last
                // received values indefinitely, so sampling unconditionally
                // makes an axis that went silent mid-run look like it reported
                // normally for the whole run.
                const auto& st = arm.get_link_stats(static_cast<int>(i));
                if (st.responses == a.seen_responses) continue;
                a.seen_responses = st.responses;

                uint8_t code = m.get_error_code();
                double tmos = m.get_state_tmos();
                double trot = m.get_state_trotor();
                double tau = std::fabs(m.get_torque());

                if (!a.responded) {
                    a.responded = true;
                    a.tmos_min = a.tmos_max = tmos;
                    a.trot_min = a.trot_max = trot;
                }
                a.code_counts[code & 0x0F]++;
                a.last_code = code;
                a.enabled = m.is_enabled();
                a.tmos_min = std::min(a.tmos_min, tmos);
                a.tmos_max = std::max(a.tmos_max, tmos);
                a.trot_min = std::min(a.trot_min, trot);
                a.trot_max = std::max(a.trot_max, trot);
                a.tmos_last = tmos;
                a.trot_last = trot;
                a.tau_abs_max = std::max(a.tau_abs_max, tau);
            }
            samples++;
        }
        std::signal(SIGINT, SIG_DFL);

        std::cout << "\n>>> Disabling motors..." << std::endl;
        openarm.disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();

        // ---------------- report ----------------
        std::cout << "\n";
        std::cout << "--- (1) Measurements: min / max / last -------------------\n";
        std::cout << std::left << std::setw(10) << " ID" << std::setw(20) << "MOS(C)"
                  << std::setw(20) << "Rotor(C)" << "|Tau| max\n";
        for (const auto& a : report) {
            std::cout << " " << std::left << std::setfill(' ') << std::setw(9) << hex_id(a.send_id)
                      << std::setw(20) << range_str(a.tmos_min, a.tmos_max, a.tmos_last, a.responded)
                      << std::setw(20) << range_str(a.trot_min, a.trot_max, a.trot_last, a.responded);
            if (a.responded)
                std::cout << std::fixed << std::setprecision(3) << a.tau_abs_max;
            else
                std::cout << "-";
            std::cout << "\n";
        }

        std::cout << "\n--- (2) Motor status / error codes -----------------------\n";
        std::cout << std::left << std::setw(10) << " ID" << std::setw(10) << "enabled"
                  << std::setw(22) << "current" << "observed during run\n";
        bool any_fault = false;
        for (const auto& a : report) {
            std::cout << " " << std::left << std::setw(9) << hex_id(a.send_id) << std::setw(10)
                      << (!a.responded ? "-" : (a.enabled ? "yes" : "no")) << std::setw(22)
                      << (!a.responded
                              ? "-"
                              : openarm::damiao_motor::motor_error_to_string(a.last_code))
                      << codes_str(a) << "\n";
            for (uint8_t c = openarm::damiao_motor::MOTOR_ERROR_THRESHOLD; c < 16; ++c)
                if (a.code_counts[c] > 0) any_fault = true;
        }

        // ---------------- (2b) per-axis link ----------------
        std::cout << "\n--- (2b) Per-axis link -----------------------------------\n";
        std::cout << std::left << std::setw(9) << " ID" << std::setw(9) << "sent"
                  << std::setw(9) << "recv" << std::setw(9) << "miss" << std::setw(9) << "miss%"
                  << "last seen\n";

        int boundary = -1;  // first silent axis that follows a healthy one
        bool seen_healthy = false;
        for (size_t i = 0; i < report.size(); ++i) {
            const auto& st = openarm.get_arm().get_link_stats(static_cast<int>(i));
            uint64_t miss = st.commands_sent > st.responses ? st.commands_sent - st.responses : 0;
            std::cout << " " << std::left << std::setw(8) << hex_id(report[i].send_id)
                      << std::setw(9) << st.commands_sent << std::setw(9) << st.responses
                      << std::setw(9) << miss << std::setw(9) << pct(st.miss_rate()) << ago(st)
                      << "\n";

            if (st.miss_rate() < 0.01)
                seen_healthy = true;
            else if (seen_healthy && boundary < 0)
                boundary = static_cast<int>(i);
        }
        if (boundary >= 0) {
            // The joints are daisy-chained, so a break silences everything past
            // it while everything before it keeps answering.
            std::cout << "   -> axes up to " << hex_id(report[boundary - 1].send_id)
                      << " answer and " << hex_id(report[boundary].send_id)
                      << " onward do not: look at the link between them.\n";
        }

        // ---------------- (3) bus ----------------
        const auto& bus = openarm.get_bus_status();
        const auto now = std::chrono::steady_clock::now();

        std::cout << "\n--- (3) Bus ----------------------------------------------\n";
        const bool link_running = openarm.is_link_running();
        std::cout << " Interface : " << interface << "  ["
                  << (link_running ? "carrier present" : "NO CARRIER") << "]\n";
        if (!link_running)
            std::cout << "   -> the interface is up but has no carrier: bus-off, or the"
                         " adapter is unplugged.\n"
                         "      write() still succeeds in this state and frames are"
                         " discarded, so nothing is transmitted.\n";
        std::cout << " Writes    : " << bus.writes_ok << " accepted by the kernel\n";
        print_counter("ENETDOWN/ENODEV (interface down)", bus.write_net_down, now);
        print_counter("ENOBUFS (tx queue full)", bus.write_no_buffer, now);
        print_counter("write failed (other errno)", bus.write_other, now);

        std::cout << " Error frames received : " << bus.error_frames << "\n";
        print_counter("BUS-OFF", bus.bus_off, now);
        print_counter("ERROR-PASSIVE", bus.error_passive, now);
        print_counter("ERROR-WARNING", bus.error_warning, now);
        print_counter("ACK ERROR (nobody acknowledged)", bus.ack_error, now);
        print_counter("TX OVERFLOW", bus.tx_overflow, now);
        print_counter("RX OVERFLOW", bus.rx_overflow, now);
        print_counter("TX TIMEOUT", bus.tx_timeout, now);
        print_counter("RESTARTED (recovered from bus-off)", bus.restarted, now);

        if (bus.tec || bus.rec) {
            std::cout << " TEC / REC : " << static_cast<int>(bus.tec) << " / "
                      << static_cast<int>(bus.rec) << "\n";
            // A transmit error counter climbing while the receive counter stays
            // at zero means this node's frames are not being acknowledged, which
            // is what a termination or reflection fault looks like.
            if (bus.tec > 0 && bus.rec == 0)
                std::cout << "   -> TX errors only: this node's frames are not being"
                             " acknowledged (suspect termination / reflection)\n";
        }
        if (bus.healthy() && link_running) std::cout << "   no bus faults recorded\n";
        if (bus.error_frames == 0 && !link_running)
            std::cout << "   note: error frames are edge-triggered, so a fault that happened"
                         " before this run\n"
                         "         produces none. Carrier state is what reports it.\n";

        std::cout << "\n Samples: " << samples << "\n";
        if (any_fault)
            std::cout << " [!] Motor fault codes were reported. Cross-check against the motor's"
                         " indicator lamp.\n";
        if (!bus.healthy() || !link_running)
            std::cout << " [!] Bus faults were recorded. These affect every axis at once and"
                         " are not attributable to one motor.\n";
        std::cout << "=========================================================\n";

    } catch (const std::exception& e) {
        std::cerr << "✗ Diagnose Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

}  // namespace openarm::cli
