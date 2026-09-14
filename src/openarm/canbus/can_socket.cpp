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

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <openarm/canbus/can_socket.hpp>

namespace openarm::canbus {

// Added to the kernel UAPI in Linux 6.0. linux-libc-dev often lags behind the
// running kernel, and guarding the counter read with #ifdef would silently
// compile it out on a kernel that does report the counters. The value is fixed
// UAPI, so define it when the installed header is older.
#ifndef CAN_ERR_CNT
#define CAN_ERR_CNT 0x00000200U
#endif

namespace {
// Error classes subscribed to through CAN_RAW_ERR_FILTER.
//
// CAN_ERR_BUSERROR and CAN_ERR_PROT are deliberately excluded: drivers only emit
// them when berr-reporting is enabled, and they then arrive once per bit error,
// which on a degraded bus reaches tens of thousands per second. CAN_ERR_LOSTARB
// is ordinary traffic on a multi-master bus. What is left are state transitions,
// which arrive a handful of times per second at worst.
constexpr can_err_mask_t DEFAULT_ERR_MASK =
    CAN_ERR_TX_TIMEOUT | CAN_ERR_CRTL | CAN_ERR_ACK | CAN_ERR_BUSOFF | CAN_ERR_RESTARTED;
}  // namespace

void BusStatus::record_error_frame(canid_t can_id, const uint8_t* data, uint8_t len) {
    error_frames++;

    const canid_t cls = can_id & CAN_ERR_MASK;
    if (cls & CAN_ERR_BUSOFF) bus_off.hit();
    if (cls & CAN_ERR_ACK) ack_error.hit();
    if (cls & CAN_ERR_TX_TIMEOUT) tx_timeout.hit();
    if (cls & CAN_ERR_RESTARTED) restarted.hit();

    if ((cls & CAN_ERR_CRTL) && len >= 2) {
        const uint8_t c = data[1];
        if (c & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE)) error_passive.hit();
        if (c & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING)) error_warning.hit();
        if (c & CAN_ERR_CRTL_TX_OVERFLOW) tx_overflow.hit();
        if (c & CAN_ERR_CRTL_RX_OVERFLOW) rx_overflow.hit();
    }

    // Only set by drivers that report the controller's error counters;
    // otherwise tec/rec stay at their last known values.
    if ((cls & CAN_ERR_CNT) && len >= 8) {
        tec = data[6];
        rec = data[7];
    }
}

void BusStatus::record_write_failure(int err) {
    last_write_errno = err;
    switch (err) {
        case ENETDOWN:
        case ENODEV:
            // The interface is not up. This is what a bus-off with no
            // auto-restart, or an unplugged adapter, looks like from the
            // sending side.
            write_net_down.hit();
            break;
        case ENOBUFS:
        case EAGAIN:
            // Transmit queue full: frames are being produced faster than the
            // bus drains them, or the controller stopped transmitting.
            write_no_buffer.hit();
            break;
        default:
            write_other.hit();
            break;
    }
}

CANSocket::CANSocket(const std::string& interface, bool enable_fd)
    : socket_fd_(-1), interface_(interface), fd_enabled_(enable_fd) {
    if (!initialize_socket(interface)) {
        throw CANSocketException("Failed to initialize socket for interface: " + interface);
    }
}

CANSocket::~CANSocket() { cleanup(); }

bool CANSocket::initialize_socket(const std::string& interface) {
    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        return false;
    }

    struct ifreq ifr;
    struct sockaddr_can addr;

    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        cleanup();
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (fd_enabled_) {
        int enable_canfd = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                       sizeof(enable_canfd)) < 0) {
            cleanup();
            return false;
        }
    }

    // Subscribe to error frames. A raw CAN socket starts with an error mask of
    // zero, so the kernel registers no error filter for it and delivers no error
    // frames at all. Without this, bus-off and error-passive transitions stay
    // invisible to the application even though the driver detects and counts
    // them (visible via `ip -details -statistics link show`).
    can_err_mask_t err_mask = DEFAULT_ERR_MASK;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        cleanup();
        return false;
    }

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        cleanup();
        return false;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        cleanup();
        return false;
    }

    return true;
}

void CANSocket::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CANSocket::is_link_running() const {
    if (!is_initialized()) return false;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFFLAGS, &ifr) < 0) return false;

    return (ifr.ifr_flags & IFF_UP) && (ifr.ifr_flags & IFF_RUNNING);
}

ssize_t CANSocket::read_raw_frame(void* buffer, size_t buffer_size) {
    if (!is_initialized()) return -1;
    return read(socket_fd_, buffer, buffer_size);
}

ssize_t CANSocket::write_raw_frame(const void* buffer, size_t frame_size) {
    if (!is_initialized()) return -1;
    return write(socket_fd_, buffer, frame_size);
}

bool CANSocket::write_can_frame(const can_frame& frame) {
    if (write(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
        bus_status_.writes_ok++;
        return true;
    }
    bus_status_.record_write_failure(errno);
    return false;
}

bool CANSocket::write_canfd_frame(const canfd_frame& frame) {
    if (write(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
        bus_status_.writes_ok++;
        return true;
    }
    bus_status_.record_write_failure(errno);
    return false;
}

bool CANSocket::read_can_frame(can_frame& frame) {
    if (!is_initialized()) return false;
    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
    return bytes_read == sizeof(frame);
}

bool CANSocket::read_canfd_frame(canfd_frame& frame) {
    if (!is_initialized()) return false;

    // A CAN_MTU read only fills the first 16 bytes, leaving flags and
    // data[8..63] holding the previous frame's bytes. Clear them first.
    memset(&frame, 0, sizeof(frame));

    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));

    // With CAN_RAW_FD_FRAMES enabled the socket delivers both CANFD_MTU (72)
    // and CAN_MTU (16) frames, and the read size is what tells them apart.
    // Classic CAN frames and error frames always arrive as CAN_MTU. Since
    // can_frame and canfd_frame place can_id, len and data at the same
    // offsets, a CAN_MTU read is usable as a canfd_frame as is.
    //
    // Rejecting CAN_MTU here would not just drop the frame: read() has
    // already dequeued it, and the caller's receive loop stops on a false
    // return, leaving the rest of the cycle's frames unread.
    return bytes_read == static_cast<ssize_t>(CANFD_MTU) ||
           bytes_read == static_cast<ssize_t>(CAN_MTU);
}

bool CANSocket::is_data_available(int timeout_us) {
    if (!is_initialized()) return false;

    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    timeout.tv_sec = timeout_us / 1000000;
    timeout.tv_usec = (timeout_us % 1000000);

    int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);

    return (result > 0 && FD_ISSET(socket_fd_, &read_fds));
}

}  // namespace openarm::canbus
