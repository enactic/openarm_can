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

#include <chrono>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

namespace {
void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <send_can_id> <recv_can_id> [can_interface] [-fd]"
              << std::endl;
    std::cout << "  send_can_id: The send CAN ID of the motor (e.g., 1, 2, 7)" << std::endl;
    std::cout << "  recv_can_id: The receive CAN ID of the motor (e.g., 17, 18, 23)" << std::endl;
    std::cout << "  can_interface: CAN interface name (default: can0)" << std::endl;
    std::cout << "  -fd: Enable CAN-FD (default: disabled)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example: " << program_name << " 1 17" << std::endl;
    std::cout << "Example: " << program_name << " 1 17 can1" << std::endl;
    std::cout << "Example: " << program_name << " 1 17 can1 -fd" << std::endl;
}

void print_motor_status(const openarm::damiao_motor::Motor& motor) {
    std::cout << "Motor ID: " << motor.get_send_can_id() << std::endl;
    std::cout << "  Position: " << motor.get_position() << " rad" << std::endl;
    std::cout << "  Velocity: " << motor.get_velocity() << " rad/s" << std::endl;
    std::cout << "  Torque: " << motor.get_torque() << " Nm" << std::endl;
    std::cout << "  Temperature (MOS): " << motor.get_state_tmos() << " °C" << std::endl;
    std::cout << "  Temperature (Rotor): " << motor.get_state_trotor() << " °C" << std::endl;
}
}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 5) {
        print_usage(argv[0]);
        return 1;
    }

    // Parse send and receive CAN IDs from command line
    uint32_t send_can_id, recv_can_id;
    try {
        send_can_id = std::stoul(argv[1]);
        recv_can_id = std::stoul(argv[2]);
    } catch (const std::exception& e) {
        std::cerr << "Error: Invalid CAN ID format" << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    // Parse optional CAN interface and FD flag
    std::string can_interface = "can0";  // default
    bool use_fd = false;                 // default: disabled

    if (argc >= 4) {
        std::string arg4 = argv[3];
        if (arg4 == "-fd") {
            use_fd = true;
        } else {
            can_interface = arg4;
        }
    }

    if (argc >= 5) {
        std::string arg5 = argv[4];
        if (arg5 == "-fd") {
            use_fd = true;
        } else {
            std::cerr << "Error: Unknown argument '" << arg5 << "'. Use -fd to enable CAN-FD"
                      << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    try {
        std::cout << "=== OpenArm Motor Control Script ===" << std::endl;
        std::cout << "Send CAN ID: " << send_can_id << std::endl;
        std::cout << "Receive CAN ID: " << recv_can_id << std::endl;
        std::cout << "CAN Interface: " << can_interface << std::endl;
        std::cout << "CAN-FD Enabled: " << (use_fd ? "Yes" : "No") << std::endl;
        std::cout << std::endl;

        // Initialize OpenArm with CAN interface
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm(can_interface,
                                              use_fd);  // Use specified interface and FD setting

        // Initialize single motor
        std::cout << "Initializing motor..." << std::endl;
        openarm.init_arm_motors({openarm::damiao_motor::MotorType::DM4310}, {send_can_id},
                                {recv_can_id});

        // Set callback mode to param for initial parameter reading
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

        // Query motor parameters (Master ID, Baudrate, and Control Mode)
        std::cout << "Reading motor parameters..." << std::endl;
        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::can_br));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Get motor and verify parameters
        const auto& motors = openarm.get_arm().get_motors();
        if (!motors.empty()) {
            const auto& motor = motors[0];
            double queried_mst_id =
                motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
            double queried_baudrate =
                motor.get_param(static_cast<int>(openarm::damiao_motor::RID::can_br));
            double queried_control_mode =
                motor.get_param(static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE));

            std::cout << "\n=== Motor Parameters ===" << std::endl;
            std::cout << "Send CAN ID: " << motor.get_send_can_id() << std::endl;
            std::cout << "Queried Master ID: " << queried_mst_id << std::endl;
            std::cout << "Queried Baudrate (1-9): " << queried_baudrate << std::endl;
            std::cout << "Queried Control Mode (1: MIT, 2: POS_VEL, 3: VEL, 4: TORQUE_POS): "
                      << queried_control_mode << std::endl;
            if (queried_control_mode != static_cast<int>(openarm::damiao_motor::ControlMode::MIT)) {
                std::cout << "Warning: Queried Control Mode (" << queried_control_mode
                          << ") is not MIT. Currently not supported." << std::endl;
            }

            // Verify recv_can_id matches queried master ID
            if (static_cast<uint32_t>(queried_mst_id) != recv_can_id) {
                std::cerr << "Error: Queried Master ID (" << queried_mst_id
                          << ") does not match provided recv_can_id (" << recv_can_id << ")"
                          << std::endl;
                return 1;
            }
            std::cout << "✓ Master ID verification passed" << std::endl;
        }

        // Switch to state callback mode for motor status updates
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Enable the motor
        std::cout << "\n=== Enabling Motor ===" << std::endl;
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();

        // Refresh 10 times at 10Hz (100ms intervals)
        std::cout << "\n=== Refreshing Motor Status (10Hz for 1 second) ===" << std::endl;
        for (int i = 1; i <= 10; i++) {
            openarm.refresh_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            openarm.recv_all();

            for (const auto& motor : openarm.get_arm().get_motors()) {
                std::cout << "\n--- Refresh " << i << "/10 ---" << std::endl;
                print_motor_status(motor);
            }
        }

        // Disable the motor
        std::cout << "\n=== Disabling Motor ===" << std::endl;
        openarm.disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();

        std::cout << "\n=== Script Completed Successfully ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
