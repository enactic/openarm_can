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

#include <vector>

#include "../../canbus/can_socket.hpp"
#include "../../damiao_motor/dm_motor.hpp"
#include "../../damiao_motor/dm_motor_device_collection.hpp"

namespace openarm::can::socket {

class ArmComponent : public damiao_motor::DMDeviceCollection {
public:
    ArmComponent(canbus::CANSocket& can_socket);
    ~ArmComponent() = default;

    void init_motor_devices(const std::vector<damiao_motor::MotorType>& motor_types,
                            const std::vector<uint32_t>& send_can_ids,
                            const std::vector<uint32_t>& recv_can_ids, bool use_fd);

private:
    std::vector<damiao_motor::Motor> motors_;
};

}  // namespace openarm::can::socket
