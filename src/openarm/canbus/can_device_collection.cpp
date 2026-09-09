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

#include <iostream>
#include <openarm/canbus/can_device_collection.hpp>
#include <openarm/canbus/can_socket.hpp>

namespace openarm::canbus {

CANDeviceCollection::CANDeviceCollection(CANSocket& can_socket) : can_socket_(can_socket) {}

CANDeviceCollection::~CANDeviceCollection() {}

void CANDeviceCollection::add_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;

    // Add device to our collection
    canid_t device_id = device->get_recv_can_id();
    devices_[device_id] = device;
}

void CANDeviceCollection::remove_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;

    canid_t device_id = device->get_recv_can_id();
    auto it = devices_.find(device_id);
    if (it != devices_.end()) {
        // Remove from our collection
        devices_.erase(it);
    }
}

void CANDeviceCollection::dispatch_frame_callback(can_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->callback(frame);
    } else {
        // Counted rather than dropped: on a bus where every id is accounted for
        // this stays at zero, so anything here is a misconfiguration worth
        // seeing rather than background traffic.
        unmatched_frames_[frame.can_id]++;
    }
}

void CANDeviceCollection::dispatch_frame_callback(canfd_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->callback(frame);
    } else {
        // Counted rather than dropped: on a bus where every id is accounted for
        // this stays at zero, so anything here is a misconfiguration worth
        // seeing rather than background traffic.
        unmatched_frames_[frame.can_id]++;
    }
}

}  // namespace openarm::canbus
