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

#include <cmath>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

/**
 * @brief Controls the operational state (Enable/Disable) of the arm motors.
 * Sends the command, then verifies each motor responded with valid state data.
 */
int run_motor_state_control(const std::string& interface, bool use_arm_ids,
                            const std::vector<std::string>& custom_ids_str, bool enable) {
    std::vector<uint32_t> send_ids;

    // 1. Populate the list of target CAN IDs
    if (use_arm_ids) {
        // IDs 1-8 are reserved for the standard arm configuration
        for (uint32_t i = 1; i <= 8; ++i) send_ids.push_back(i);
    }
    for (const auto& id_str : custom_ids_str) {
        try {
            // Parses both decimal and hex (0x-prefixed) strings
            send_ids.push_back(std::stoul(id_str, nullptr, 0));
        } catch (...) {
            std::cerr << "✗ Error: Invalid ID format provided: '" << id_str << "'\n";
            return 1;
        }
    }

    if (send_ids.empty()) {
        std::cerr << "✗ Error: No target IDs specified. Use --arm or --id.\n";
        return 1;
    }

    try {
        // 2. Initialize the OpenArm SocketCAN interface
        std::cout << ">>> Opening " << interface << " (CAN-FD Enabled)..." << std::endl;
        openarm::can::socket::OpenArm openarm(interface, true);

        // 3. Register and initialize motor components
        // DaMiao motors use: Response_ID = Send_ID + 0x10
        std::vector<openarm::damiao_motor::MotorType> motor_types(
            send_ids.size(), openarm::damiao_motor::MotorType::DM4310);
        std::vector<uint32_t> recv_ids;
        for (auto id : send_ids) recv_ids.push_back(id + 0x10);

        std::cout << ">>> Initializing " << send_ids.size() << " DAMIAO motor(s)..." << std::endl;
        openarm.init_arm_motors(motor_types, send_ids, recv_ids);

        // 4. Send enable/disable command (IGNORE mode: skip telemetry parsing during TX)
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        if (enable) {
            std::cout << ">>> Action: ENABLING torque output for target motors..." << std::endl;
            openarm.enable_all();
        } else {
            std::cout << ">>> Action: DISABLING torque output for target motors..." << std::endl;
            openarm.disable_all();
        }

        // 5. Switch to STATE mode and wait for motor responses
        // 500ms timeout for first frame - motors should respond within this window
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        openarm.recv_all(500000);

        // 6. Verify each motor responded with valid state data
        const auto& motors = openarm.get_arm().get_motors();
        std::vector<uint32_t> no_response;

        for (size_t i = 0; i < motors.size(); ++i) {
            if (!std::isfinite(motors[i].get_position())) {
                no_response.push_back(send_ids[i]);
            }
        }

        if (!no_response.empty()) {
            std::cerr << "✗ No response from motor(s): ";
            for (auto id : no_response) {
                std::cerr << (id < 16 ? "0x0" : "0x") << std::hex << id << " ";
            }
            std::cerr << std::dec << "\n";
            std::cerr << "  Check wiring, power, and CAN ID configuration.\n";
            return 1;
        }

        std::cout << "✓ State change confirmed for IDs: ";
        for (auto id : send_ids) {
            std::cout << (id < 16 ? "0x0" : "0x") << std::hex << id << " ";
        }
        std::cout << std::dec << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "✗ System Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

}  // namespace openarm::cli
