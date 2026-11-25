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

#include <algorithm>
#include <damiao_can/canbus/can_device_collection.hpp>
#include <damiao_can/canbus/can_socket.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

namespace damiao_can::canbus {

CANDeviceCollection::CANDeviceCollection(CANSocket& can_socket) : can_socket_(can_socket) {}

CANDeviceCollection::~CANDeviceCollection() {}

void CANDeviceCollection::add_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) {
        throw std::invalid_argument("CAN device must not be null");
    }

    const canid_t device_id = device->get_recv_can_id();
    if (devices_.find(device_id) != devices_.end()) {
        throw std::invalid_argument("duplicate receive CAN ID: " +
                                    std::to_string(static_cast<uint32_t>(device_id)));
    }
    devices_.emplace(device_id, device);
    ordered_devices_.push_back(device);
}

void CANDeviceCollection::remove_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) {
        throw std::invalid_argument("CAN device must not be null");
    }

    canid_t device_id = device->get_recv_can_id();
    auto it = devices_.find(device_id);
    if (it != devices_.end()) {
        const auto removed = it->second;
        // Remove from our collection
        devices_.erase(it);
        ordered_devices_.erase(
            std::remove(ordered_devices_.begin(), ordered_devices_.end(), removed),
            ordered_devices_.end());
    }
}

void CANDeviceCollection::dispatch_frame_callback(can_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->callback(frame);
    }
    // Note: Silently ignore frames for unknown devices (this is normal in CAN
    // networks)
}

void CANDeviceCollection::dispatch_frame_callback(canfd_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->callback(frame);
    }
    // Note: Silently ignore frames for unknown devices (this is normal in CAN
    // networks)
}

}  // namespace damiao_can::canbus
