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

// Reading health information out of openarm_can.
//
// Torque is never enabled here: refresh works regardless of whether a motor is
// armed, so the arm cannot move while this runs. That makes it safe to leave
// running and unplug a cable to watch what happens.
//
//     openarm-can-health can0 [seconds]

#include <chrono>
#include <cstdio>
#include <openarm/can/socket/openarm.hpp>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    const std::string interface = argc > 1 ? argv[1] : "can0";
    const int seconds = argc > 2 ? std::atoi(argv[2]) : 10;

    try {
        openarm::can::socket::OpenArm openarm(interface, true);

        std::vector<uint32_t> send_ids;
        std::vector<uint32_t> recv_ids;
        std::vector<openarm::damiao_motor::MotorType> types;
        for (uint32_t id = 1; id <= 8; ++id) {
            send_ids.push_back(id);
            recv_ids.push_back(id + 0x10);
            types.push_back(openarm::damiao_motor::MotorType::DM4310);
        }
        openarm.init_arm_motors(types, send_ids, recv_ids);
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
            // Ask every motor for its state, then read whatever came back. Error
            // frames arrive on the same socket and are separated out by recv_all
            // into the bus status; they never reach a motor callback.
            openarm.refresh_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            openarm.recv_all();

            // ---- bus level: not attributable to any one axis ----
            const auto& bus = openarm.get_bus_status();

            // A bus-off leaves IFF_UP set and only drops the carrier, so write()
            // keeps succeeding while nothing is transmitted. Checking the
            // carrier is the only way to notice a fault that predates this
            // socket, because error frames are edge triggered.
            printf("bus [%s] healthy=%s writes_ok=%lu",
                   openarm.is_link_running() ? "carrier" : "NO CARRIER",
                   bus.healthy() ? "yes" : "no", bus.writes_ok);
            if (bus.bus_off) printf("  BUS-OFF x%lu", bus.bus_off.count);
            if (bus.error_passive) printf("  ERROR-PASSIVE x%lu", bus.error_passive.count);
            if (bus.ack_error) printf("  ACK-ERROR x%lu", bus.ack_error.count);
            if (bus.write_net_down) printf("  ENETDOWN x%lu", bus.write_net_down.count);
            printf("\n");

            // ---- per axis: silence is not a bus error ----
            // Unplugging one motor raises nothing on the bus, because CAN
            // acknowledges a frame if any node hears it. The only way to see it
            // is to compare what was sent against what came back.
            for (size_t i = 0; i < send_ids.size(); ++i) {
                const auto& link = openarm.get_arm().get_link_stats(static_cast<int>(i));
                const auto motor = openarm.get_arm().get_motor(static_cast<int>(i));

                const bool stale = link.is_stale(std::chrono::milliseconds(100));
                printf("  0x%02X recv=%lu/%lu miss=%.1f%% %s",
                       send_ids[i], link.responses, link.commands_sent,
                       link.miss_rate() * 100.0, stale ? "SILENT" : "ok");

                // The motor's own faults are a third, separate thing: a hot or
                // overloaded motor is talking fine, the bus is fine, and only
                // D[0] says anything is wrong.
                if (motor.has_error())
                    printf("  FAULT=%s",
                           openarm::damiao_motor::motor_error_to_string(motor.get_error_code()));
                printf("\n");
            }
            printf("\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
