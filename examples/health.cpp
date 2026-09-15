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

// Everything openarm_can can report about an arm, printed as it changes.
//
// Torque is never enabled here: refresh works whether or not a motor is armed,
// so the arm cannot move while this runs. That makes it safe to leave running
// and unplug a cable to watch what happens.
//
//     openarm-can-health [interface] [seconds] [motor count] [--toggle]
//
// With --toggle it enables and disables once a second and counts how often the
// motors confirm the command, which is a way to reproduce an enable that does
// not take. That arms the motors: a disabled arm falls under gravity and the
// next enable holds it wherever it landed, so only do it with the arm supported
// or on a single low-consequence axis.
//
// The three sections answer different questions, and none of them substitutes
// for the others:
//
//   motor  -- what each motor reports about itself, including the status code
//             in D[0] that says whether it is armed, too hot, or overloaded.
//   link   -- what came back versus what was sent. CAN acknowledges a frame if
//             any node hears it, so one unplugged motor raises no bus error at
//             all and only shows up as a gap here.
//   bus    -- faults that stop every axis at once and cannot be attributed to
//             any single one.

#include <CLI/CLI.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <openarm/can/socket/openarm.hpp>
#include <string>
#include <thread>
#include <vector>

using namespace openarm;

namespace {

std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

void print_bus(const canbus::BusStatus& bus, bool carrier) {
    printf("  bus   : [%s] healthy=%s  writes_ok=%lu  error_frames=%lu\n",
           carrier ? "carrier" : "NO CARRIER", bus.healthy() ? "yes" : "no", bus.writes_ok,
           bus.error_frames);
    if (!carrier)
        printf(
            "          no carrier: the interface is administratively down, gone bus-off, or\n"
            "          unplugged. A bus-off keeps IFF_UP set and write() keeps succeeding,\n"
            "          so the errno below is what separates the cases.\n");

    const auto now = std::chrono::steady_clock::now();
    auto line = [&](const char* name, const canbus::ErrorCounter& c) {
        if (!c) return;
        printf("          %-22s x%-8lu last %.2fs ago\n", name, c.count,
               std::chrono::duration<double>(now - c.last).count());
    };
    line("BUS-OFF", bus.bus_off);
    line("ERROR-PASSIVE", bus.error_passive);
    line("ERROR-WARNING", bus.error_warning);
    line("ACK-ERROR", bus.ack_error);
    line("TX-OVERFLOW", bus.tx_overflow);
    line("RX-OVERFLOW", bus.rx_overflow);
    line("TX-TIMEOUT", bus.tx_timeout);
    line("RESTARTED", bus.restarted);
    line("write ENETDOWN", bus.write_net_down);
    line("write ENOBUFS", bus.write_no_buffer);
    line("write other errno", bus.write_other);
    if (bus.last_write_errno)
        printf("          last write errno %d (%s)\n", bus.last_write_errno,
               strerror(bus.last_write_errno));
    if (bus.tec || bus.rec) printf("          TEC/REC %u/%u\n", bus.tec, bus.rec);
}

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"Everything openarm_can can report about an arm, printed as it changes."};
    std::string interface = "can0";
    int seconds = 10;
    int motor_count = 8;
    bool toggle = false;
    app.add_option("interface", interface, "SocketCAN interface")->capture_default_str();
    app.add_option("seconds", seconds, "How long to run")
        ->check(CLI::PositiveNumber)
        ->capture_default_str();
    app.add_option("motors", motor_count, "Number of arm motors, send ids 1..N")
        ->check(CLI::Range(1, 8))
        ->capture_default_str();
    app.add_flag("--toggle", toggle,
                 "Enable and disable once a second and count how often the motors confirm."
                 " ARMS THE MOTORS: the arm falls on each disable.");
    CLI11_PARSE(app, argc, argv);

    try {
        can::socket::OpenArm openarm(interface, true);

        std::vector<uint32_t> send_ids, recv_ids;
        std::vector<damiao_motor::MotorType> types;
        for (int i = 1; i <= motor_count; ++i) {
            send_ids.push_back(i);
            recv_ids.push_back(i + 0x10);
            types.push_back(damiao_motor::MotorType::DM4310);
        }
        openarm.init_arm_motors(types, send_ids, recv_ids);
        openarm.set_callback_mode_all(damiao_motor::CallbackMode::STATE);

        if (toggle)
            printf(
                "!!! --toggle arms the motors once a second. The arm will fall on each"
                " disable.\n\n");

        bool commanded_enabled = false;
        int cycles = 0, enable_ok = 0, disable_ok = 0;
        auto last_toggle = std::chrono::steady_clock::now();

        std::signal(SIGINT, handle_sigint);

        const auto start = std::chrono::steady_clock::now();
        while (g_running &&
               std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
            if (toggle &&
                std::chrono::steady_clock::now() - last_toggle >= std::chrono::seconds(1)) {
                last_toggle = std::chrono::steady_clock::now();
                commanded_enabled = !commanded_enabled;
                if (commanded_enabled)
                    openarm.enable_all();
                else
                    openarm.disable_all();
                cycles++;
            }

            // Error frames arrive on the same socket and are separated out by
            // recv_all into the bus status; they never reach a motor callback.
            openarm.refresh_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            openarm.recv_all();

            printf(
                "=== %s  t=%.1fs%s ===\n", interface.c_str(),
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(),
                toggle ? (commanded_enabled ? "  commanded: ENABLE" : "  commanded: DISABLE") : "");
            printf(
                "   ID   pos(rad)  vel(rad/s)   tau(Nm)  MOS  Rtr  status              "
                "recv/sent    miss   rej/mal   last\n");

            const auto& arm = openarm.get_arm();
            for (size_t i = 0; i < send_ids.size(); ++i) {
                const auto motor = arm.get_motor(static_cast<int>(i));
                const auto& link = arm.get_link_stats(static_cast<int>(i));

                printf(" 0x%02X %9.4f %11.4f %9.4f %4d %4d  %-20s %6lu/%-6lu %5.1f%% ", send_ids[i],
                       motor.get_position(), motor.get_velocity(), motor.get_torque(),
                       motor.get_state_tmos(), motor.get_state_trotor(),
                       damiao_motor::motor_error_to_string(motor.get_error_code()), link.responses,
                       link.commands_sent, link.miss_rate() * 100.0);
                printf("%4u/%-5u ", link.rejected_commands, link.malformed_frames);

                if (!link.ever_responded())
                    printf("never  SILENT");
                else
                    printf("%.2fs%s",
                           std::chrono::duration<double>(link.since_last_response()).count(),
                           link.is_stale(std::chrono::milliseconds(100)) ? "  SILENT" : "");
                if (motor.has_error()) printf("  FAULT");
                // The motor reports its armed state in the same D[0] nibble, so
                // a command that did not take shows up here and nowhere else.
                if (toggle && link.ever_responded() && motor.is_enabled() != commanded_enabled)
                    printf("  MISMATCH");
                printf("\n");
            }

            if (toggle) {
                bool all_match = true;
                for (size_t i = 0; i < send_ids.size(); ++i)
                    all_match &=
                        arm.get_motor(static_cast<int>(i)).is_enabled() == commanded_enabled;
                if (all_match) (commanded_enabled ? enable_ok : disable_ok)++;
                printf("  toggle: %d cycles  enable confirmed %d  disable confirmed %d\n", cycles,
                       enable_ok, disable_ok);
            }

            print_bus(openarm.get_bus_status(), openarm.is_link_running());
            // Replies on an id nothing is registered for. Always zero on a bus
            // where every master id was configured, so anything here is a
            // motor answering under the wrong id rather than a silent one.
            for (const auto& [id, count] : openarm.get_unmatched_frames())
                printf(
                    "  unmatched: id 0x%02X x%lu  (a motor replying on an id nothing"
                    " listens for; check RID 7)\n",
                    id, count);
            printf("\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // Leave the motors as they were found. Only --toggle ever arms them, so
        // only disarm if this process actually sent an enable: sending a disable
        // unconditionally would drop an arm that something else is holding up.
        if (commanded_enabled) {
            std::signal(SIGINT, SIG_DFL);
            printf(">>> disabling\n");
            openarm.disable_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            openarm.recv_all();
            for (size_t i = 0; i < send_ids.size(); ++i) {
                const auto motor = openarm.get_arm().get_motor(static_cast<int>(i));
                if (motor.is_enabled())
                    fprintf(stderr, "    0x%02X did not confirm the disable\n", send_ids[i]);
            }
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
