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

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "can_device.hpp"
#include "can_socket.hpp"

namespace damiao_can::canbus {
class CANDeviceCollection {
public:
    CANDeviceCollection(canbus::CANSocket& can_socket);
    ~CANDeviceCollection();

    void add_device(const std::shared_ptr<CANDevice>& device);
    void remove_device(const std::shared_ptr<CANDevice>& device);
    void dispatch_frame_callback(can_frame& frame);
    void dispatch_frame_callback(canfd_frame& frame);
    const std::map<canid_t, std::shared_ptr<CANDevice>>& get_devices() const { return devices_; }
    // Devices in the order they were added; motor indices follow this order.
    const std::vector<std::shared_ptr<CANDevice>>& get_ordered_devices() const {
        return ordered_devices_;
    }
    canbus::CANSocket& get_can_socket() const { return can_socket_; }
    int get_socket_fd() const { return can_socket_.get_socket_fd(); }

private:
    canbus::CANSocket& can_socket_;
    std::map<canid_t, std::shared_ptr<CANDevice>> devices_;
    std::vector<std::shared_ptr<CANDevice>> ordered_devices_;
};
}  // namespace damiao_can::canbus
