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
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace openarm::canbus {

// Exception classes for socket operations
class CANSocketException : public std::runtime_error {
public:
    explicit CANSocketException(const std::string& message)
        : std::runtime_error("Socket error: " + message) {}
};

// How often one kind of fault happened, and when it last did. Errors are
// counted rather than logged so that recording one costs a memory write and
// never any I/O, which matters because the only code that records them is the
// control loop.
struct ErrorCounter {
    uint64_t count = 0;
    std::chrono::steady_clock::time_point last{};

    void hit() {
        count++;
        last = std::chrono::steady_clock::now();
    }
    explicit operator bool() const { return count > 0; }
};

// State of the CAN interface, as opposed to the state of any one motor. A
// bus-off is a property of the whole bus and cannot be attributed to an axis,
// which is why it lives here and not on Motor.
//
// Counters latch: they are never cleared by recovery, only by clear(). A bus-off
// that the driver auto-restarts from can be over within milliseconds, so polling
// for the current state would miss it entirely.
struct BusStatus {
    // Controller state, reported by the kernel as error frames.
    ErrorCounter bus_off;         // CAN_ERR_BUSOFF
    ErrorCounter error_passive;   // CAN_ERR_CRTL: RX/TX_PASSIVE
    ErrorCounter error_warning;   // CAN_ERR_CRTL: RX/TX_WARNING
    ErrorCounter tx_overflow;     // CAN_ERR_CRTL: TX_OVERFLOW
    ErrorCounter rx_overflow;     // CAN_ERR_CRTL: RX_OVERFLOW
    ErrorCounter ack_error;       // CAN_ERR_ACK: nobody acknowledged
    ErrorCounter tx_timeout;      // CAN_ERR_TX_TIMEOUT
    ErrorCounter restarted;       // CAN_ERR_RESTARTED: recovered from bus-off

    // Transmit error counter / receive error counter, when the driver reports
    // them. TEC high with REC at zero means this node's frames are going
    // unacknowledged, which is what a termination or reflection fault looks like.
    uint8_t tec = 0;
    uint8_t rec = 0;

    // Send-side failures, taken from errno. Without these a cut cable looks
    // exactly like a quiet bus.
    ErrorCounter write_net_down;   // ENETDOWN / ENODEV: interface is not up
    ErrorCounter write_no_buffer;  // ENOBUFS / EAGAIN: transmit queue full
    ErrorCounter write_other;
    // errno of the most recent failed write. Bucketing alone loses too much:
    // an interface that goes down reports ENETDOWN once and then EINVAL for
    // every write after it, so "other" is where most of a real fault lands.
    int last_write_errno = 0;
    uint64_t writes_ok = 0;

    uint64_t error_frames = 0;

    bool healthy() const {
        return !bus_off && !error_passive && !tx_overflow && !rx_overflow && !ack_error &&
               !tx_timeout && !write_net_down && !write_no_buffer && !write_other;
    }
    void clear() { *this = BusStatus{}; }

    // Takes the raw fields rather than a frame type, because an error frame is
    // always CAN_MTU and so arrives as a can_frame on a classic socket and as a
    // canfd_frame on an FD one.
    void record_error_frame(canid_t can_id, const uint8_t* data, uint8_t len);
    void record_write_failure(int err);
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

    // Whether the interface currently has carrier. A CAN interface that goes
    // bus-off keeps IFF_UP set and only drops IFF_RUNNING, so write() still
    // succeeds and no error frame is produced for a fault that happened before
    // this socket existed. This is the only way to see that state on open.
    // Costs one ioctl, so poll it deliberately rather than every cycle.
    bool is_link_running() const;

    const BusStatus& get_bus_status() const { return bus_status_; }
    BusStatus& get_bus_status() { return bus_status_; }
    bool is_bus_healthy() const { return bus_status_.healthy(); }
    void clear_bus_status() { bus_status_.clear(); }

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
    BusStatus bus_status_;
};

}  // namespace openarm::canbus
