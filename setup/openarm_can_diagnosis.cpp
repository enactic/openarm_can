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
#include <cmath>
#include <string.h>
#include <time.h>
#include <vector>

static const char* br_label(int br_code) {
    // simple mapping example
    switch (br_code) {
        case 9: return "5 Mbps";
        case 4: return "1 Mbps";
        default: return "(unknown)";
    }
}

int main(int argc, char* argv[]) {
    std::cout << "OpenArm CAN diagnostics\n";

    // Args: <can_interface> [-fd]
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <can_interface> [-fd]\n";
        return 1;
    }
    std::string can_if = argv[1];
    bool use_fd = false;
    if (argc >= 3) {
        std::string arg2 = argv[2];
        if (arg2 == "-fd") use_fd = true;
        else {
            std::cerr << "Error: Unknown argument '" << arg2 << "'. Use -fd to enable CAN-FD.\n";
            return 1;
        }
    }

    std::cout << "CAN interface: " << can_if << std::endl;
    std::cout << "CAN-FD mode: " << (use_fd ? "enabled" : "disabled") << std::endl;

    std::cout << "Initializing OpenArm CAN..." << std::endl;
    // use the actual args
    openarm::can::socket::OpenArm openarm(can_if, use_fd);

    // Initialize arm motors
    std::vector<openarm::damiao_motor::MotorType> motor_types = {
        openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
        openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
        openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
        openarm::damiao_motor::MotorType::DM4310};
    std::vector<uint32_t> send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    std::vector<uint32_t> recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

    // Initialize gripper
    std::cout << "Initializing gripper..." << std::endl;
    openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);

    openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

    std::cout << "Reading motor parameters ..." << std::endl;
    openarm.query_param_all((int)openarm::damiao_motor::RID::MST_ID);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm.recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    openarm.query_param_all((int)openarm::damiao_motor::RID::can_br);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm.recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // read params
    const auto& arm_motors = openarm.get_arm().get_motors();
    const auto& gripper_motors = openarm.get_gripper().get_motors();

    std::vector<uint32_t> missing_ids;

    for (size_t i = 0; i < arm_motors.size(); ++i) {
        const auto& motor = arm_motors[i];
        double mst = motor.get_param((int)openarm::damiao_motor::RID::MST_ID);
        double br  = motor.get_param((int)openarm::damiao_motor::RID::can_br);

        if (mst < 0 || br < 0 || !std::isfinite(mst) || !std::isfinite(br)) {
            std::cout << "[arm#" << i << "] id=0x" << std::hex << recv_can_ids[i]
                      << std::dec << " -> NG (no response)\n";
            missing_ids.push_back(recv_can_ids[i]);
        } else {
            std::cout << "[arm#" << i << "] queried_mst_id: " << (uint32_t)mst
                      << "  queried_br: " << (int)br << " (" << br_label((int)br) << ")\n";
        }
    }

    for (size_t i = 0; i < gripper_motors.size(); ++i) {
        const auto& gr = gripper_motors[i];
        double mst = gr.get_param((int)openarm::damiao_motor::RID::MST_ID);
        double br  = gr.get_param((int)openarm::damiao_motor::RID::can_br);

        if (mst < 0 || br < 0 || !std::isfinite(mst) || !std::isfinite(br)) {
            std::cout << "[gripper] id=0x18 -> NG (no response)\n";
            missing_ids.push_back(0x18);
        } else {
            std::cout << "[gripper] queried_mst_id: " << (uint32_t)mst
                      << "  queried_br: " << (int)br << " (" << br_label((int)br) << ")\n";
        }
    }

    if (!missing_ids.empty()) {
        std::cout << "NG: failed IDs:";
        for (auto id : missing_ids) std::cout << " 0x" << std::hex << id;
        std::cout << std::dec << "\n";

        // show troubleshooting hints
        std::cout << "Hints:\n";
        std::cout << "  • Motor internal CAN bitrate may be different from host setting\n";
        std::cout << "  • USB2CAN adapter mode/config may be wrong (FD vs Classical, bitrate profile)\n";
        std::cout << "  • Wiring/power/termination/ID conflict may exist\n";
        // return non-zero if you want to signal failure
        return 2;
    } else {
        std::cout << "OK: all motors responded\n";
        return 0;
    }
}
