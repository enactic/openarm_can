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

#pragma once

#include <memory>
#include <string>

#include "../../canbus/can_device_collection.hpp"
#include "../../canbus/can_socket.hpp"
#include "arm_component.hpp"
#include "gripper_component.hpp"

namespace openarm::can::socket {
class OpenArm {
public:
    OpenArm(const std::string& can_interface, bool enable_fd = false);
    ~OpenArm() = default;

    std::string can_interface() const noexcept { return can_interface_; }
    bool can_fd_enabled() const noexcept { return enable_fd_; }

    // Component initialization
    void init_arm_motors(const std::vector<damiao_motor::MotorType>& motor_types,
                         const std::vector<uint32_t>& send_can_ids,
                         const std::vector<uint32_t>& recv_can_ids);

    void init_gripper_motor(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                            uint32_t recv_can_id);

    // Component access
    ArmComponent& get_arm() { return *arm_; }
    GripperComponent& get_gripper() { return *gripper_; }
    canbus::CANDeviceCollection& get_master_can_device_collection() {
        return *master_can_device_collection_;
    }

    // Damiao Motor operations (works only on sub_dm_device_collections_)
    void enable_all();
    void disable_all();
    void set_zero_all();
    void refresh_all();

    void refresh_one(int i);
    // The timeout for reading from socket, set to timeout_us.
    // Tuning this value may improve the performance but should be done with caution.
    void recv_all(int timeout_us = 500);
    void set_callback_mode_all(damiao_motor::CallbackMode callback_mode);
    void query_param_all(int RID);

private:
    std::string can_interface_;
    bool enable_fd_;
    std::unique_ptr<canbus::CANSocket> can_socket_;
    std::unique_ptr<ArmComponent> arm_;
    std::unique_ptr<GripperComponent> gripper_;
    std::unique_ptr<canbus::CANDeviceCollection> master_can_device_collection_;
    std::vector<damiao_motor::DMDeviceCollection*> sub_dm_device_collections_;
    void register_dm_device_collection(damiao_motor::DMDeviceCollection& device_collection);
};

}  // namespace openarm::can::socket
