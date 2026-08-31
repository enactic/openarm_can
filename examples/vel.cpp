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

// C++ counterpart of python/examples/test_vel.py.

#include <chrono>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <thread>
#include <vector>

using namespace openarm::damiao_motor;

int main() {
    // Create OpenArm instance
    openarm::can::socket::OpenArm arm("can0", true);

    // Initialize arm motors
    std::vector<MotorType> motor_types = {MotorType::DM4310};
    std::vector<uint32_t> send_ids = {0x24};
    std::vector<uint32_t> recv_ids = {0x34};
    std::vector<ControlMode> control_modes = {ControlMode::VEL};
    arm.init_arm_motors(motor_types, send_ids, recv_ids, control_modes);

    // Enable motors
    arm.enable_all();
    arm.recv_all();

    // VelParam{dq}
    //   dq : target velocity (rad/s)
    arm.set_callback_mode_all(CallbackMode::STATE);
    arm.get_arm().vel_control_all({VelParam{1.0}});
    arm.recv_all();

    // Read motor state every 0.1s for 30 iterations
    for (int i = 0; i < 30; i++) {
        arm.refresh_all();
        arm.recv_all();
        for (const auto& motor : arm.get_arm().get_motors()) {
            std::cout << motor.get_velocity() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Command zero velocity before disabling
    arm.get_arm().vel_control_all({VelParam{0.0}});
    arm.recv_all();

    arm.disable_all();
    arm.recv_all();
    return 0;
}
