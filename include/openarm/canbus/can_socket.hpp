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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdexcept>
#include <string>

namespace openarm::canbus {

// Exception classes for socket operations
class CANSocketException : public std::runtime_error {
public:
    explicit CANSocketException(const std::string& message)
        : std::runtime_error("Socket error: " + message) {}
};

// Base socket management class
class CANSocket {
public:
    explicit CANSocket(const std::string& interface, bool enable_fd = false);
    ~CANSocket();

    // Disable copy, enable move
    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;
    CANSocket(CANSocket&&) = default;
    CANSocket& operator=(CANSocket&&) = default;

    // File descriptor access for Python bindings
    int get_socket_fd() const { return socket_fd_; }
    const std::string& get_interface() const { return interface_; }
    bool is_canfd_enabled() const { return fd_enabled_; }
    bool is_initialized() const { return socket_fd_ >= 0; }

    // Direct frame operations for Python bindings
    ssize_t read_raw_frame(void* buffer, size_t buffer_size);
    ssize_t write_raw_frame(const void* buffer, size_t frame_size);

    // write can_frame or canfd_frame
    bool write_can_frame(const can_frame& frame);
    bool write_canfd_frame(const canfd_frame& frame);

    // read can_frame or canfd_frame
    bool read_can_frame(can_frame& frame);
    bool read_canfd_frame(canfd_frame& frame);

    // check if data is available for reading (non-blocking)
    bool is_data_available(int timeout_us = 100);

protected:
    bool initialize_socket(const std::string& interface);
    void cleanup();

    int socket_fd_;
    std::string interface_;
    bool fd_enabled_;
};

}  // namespace openarm::canbus
