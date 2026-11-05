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

namespace openarm::canbus {
class CANDeviceCollection {
public:
    CANDeviceCollection(canbus::CANSocket& can_socket);
    ~CANDeviceCollection();

    // Each device held here keeps a receive filter for its recv id on the
    // socket, so a socket shared by several collections receives the ids any of
    // them holds and nothing else.
    void add_device(const std::shared_ptr<CANDevice>& device);
    void remove_device(const std::shared_ptr<CANDevice>& device);
    void dispatch_frame_callback(can_frame& frame);
    void dispatch_frame_callback(canfd_frame& frame);
    const std::map<canid_t, std::shared_ptr<CANDevice>>& get_devices() const { return devices_; }
    canbus::CANSocket& get_can_socket() const { return can_socket_; }
    int get_socket_fd() const { return can_socket_.get_socket_fd(); }

    // Frames that arrived for an id no device here is registered for, counted
    // per id.
    //
    // Once a device is registered the socket only receives ids with a filter, so
    // these are frames for a filter added on the socket directly, or for a device
    // that only another collection on the socket holds. A motor whose master id
    // (RID 7) was left at its default of 0 replies on 0x00, so a tool that wants
    // to report one adds a filter for 0x00. On a correctly configured bus nothing
    // arrives there, which is what makes a non-zero count worth reading.
    const std::map<canid_t, uint64_t>& get_unmatched_frames() const { return unmatched_frames_; }
    void clear_unmatched_frames() { unmatched_frames_.clear(); }

private:
    canbus::CANSocket& can_socket_;
    std::map<canid_t, std::shared_ptr<CANDevice>> devices_;
    std::map<canid_t, uint64_t> unmatched_frames_;
};
}  // namespace openarm::canbus
