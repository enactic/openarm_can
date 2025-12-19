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
#include <cstdlib>
#include <iostream>
#include <thread>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>

int main() {
    using namespace openarm::damiao_motor;

    const char* run_hw = std::getenv("OPENARM_CAN_RUN_HW_TESTS");
    if (!run_hw) {
        std::cout
            << "Skipping hardware posforce test (set OPENARM_CAN_RUN_HW_TESTS=1 to run against CAN)"
            << std::endl;
        return 0;
    }

    const char* iface_env = std::getenv("OPENARM_CAN_IFACE");
    std::string can_iface = iface_env ? iface_env : "can0";

    try {
        openarm::can::socket::OpenArm openarm(can_iface, /*enable_fd=*/true);
        openarm.init_gripper_motor(MotorType::DM4310, /*send_can_id=*/0x08, /*recv_can_id=*/0x18);

        // Switch gripper to torque-position control mode before sending posforce commands.
        openarm.set_callback_mode_all(CallbackMode::PARAM);
        openarm.get_gripper().set_control_mode_all(ControlMode::TORQUE_POS);

        openarm.enable_all();
        openarm.recv_all(500);

        PosForceParam param{-1, 5.0, 0.5};
        openarm.set_callback_mode_all(CallbackMode::STATE);
        openarm.get_gripper().posforce_control_one(0, param);
        openarm.recv_all(500);

        // Poll a few cycles to verify communication and state updates.
        for (int i = 0; i < 10; ++i) {
            openarm.refresh_all();
            openarm.recv_all(500);

            for (const auto& motor : openarm.get_gripper().get_motors()) {
                std::cout << "gripper motor (" << motor.get_send_can_id()
                          << ") position: " << motor.get_position() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Switch gripper back to MIT control mode before sending posforce commands.
        openarm.set_callback_mode_all(CallbackMode::PARAM);
        openarm.get_gripper().set_control_mode_all(ControlMode::MIT);

        openarm.disable_all();
        openarm.recv_all(500);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Hardware posforce test failed: " << e.what() << std::endl;
        return 1;
    }
}
