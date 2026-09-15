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

// C++ counterpart of python/examples/test_gripper_posforce.py.

#include <chrono>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <thread>
#include <vector>

using namespace openarm::damiao_motor;

int main() {
    openarm::can::socket::OpenArm arm("can1", true);

    arm.init_arm_motors({MotorType::DM4310}, {0x4}, {0x14}, {ControlMode::MIT});
    // init_gripper_motor writes the control mode to the motor. That write is
    // RAM only and is not read back, so a lost write leaves the motor in
    // whatever mode its flash holds while every command below is silently
    // discarded. Query RID 10 if the gripper does not respond to set_position.
    arm.init_gripper_motor(MotorType::DM4310, 0x8, 0x18, ControlMode::POS_FORCE);

    arm.set_callback_mode_all(CallbackMode::PARAM);
    arm.enable_all();
    arm.recv_all();

    arm.set_callback_mode_all(CallbackMode::STATE);
    auto& gripper = arm.get_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // position (rad), speed limit (rad/s), torque limit (per-unit, 0.0-1.0)
    const std::vector<std::tuple<double, double, double>> sequence = {
        {3.14 / 2.0, 25.0, 0.15},
        {0.0, 25.0, 0.15},
        {3.14 / 2.0, 25.0, 0.15},
        {0.0, 25.0, 0.15},
    };

    for (const auto& [position, speed, torque] : sequence) {
        std::cout << "set_position(" << position << ") speed=" << speed << " torque=" << torque
                  << std::endl;
        gripper.set_position(position, speed, torque);
        arm.get_arm().mit_control_all({MITParam{0, 0, 0, 0, 0}});

        for (int i = 0; i < 6; i++) {
            arm.refresh_all();
            arm.recv_all(500);
            for (const auto& motor : gripper.get_motors()) {
                std::cout << "  gripper position: " << motor.get_position() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    arm.disable_all();
    arm.recv_all();
    return 0;
}
