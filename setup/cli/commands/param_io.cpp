// Copyright 2026 Enactic, Inc.
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

#include <linux/can.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <openarm/canbus/can_socket.hpp>
#include <thread>

#include "cli.hpp"

namespace openarm::cli {

namespace {

constexpr uint32_t CONFIG_CAN_ID = 0x7FF;
constexpr uint8_t CMD_READ = 0x33;
constexpr uint8_t CMD_WRITE = 0x55;
constexpr uint8_t CMD_SAVE = 0xAA;

// A motor answers a read or write within about a millisecond; 100 ms leaves
// room for a busy bus without making a dead motor slow to report.
constexpr int REPLY_TIMEOUT_MS = 100;
constexpr int ATTEMPTS = 3;

can_frame config_frame(uint16_t slave_id, uint8_t cmd, uint8_t rid, uint32_t value) {
    can_frame frame{};
    frame.can_id = CONFIG_CAN_ID;
    frame.can_dlc = 8;
    frame.data[0] = slave_id & 0xFF;
    frame.data[1] = (slave_id >> 8) & 0xFF;
    frame.data[2] = cmd;
    frame.data[3] = rid;
    frame.data[4] = value & 0xFF;
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = (value >> 16) & 0xFF;
    frame.data[7] = (value >> 24) & 0xFF;
    return frame;
}

// The reply comes from the motor's master ID, which change_id may be in the
// middle of changing, so it is matched by its payload instead: the slave ID,
// command and RID are echoed back. Status replies and other motors' frames are
// skipped.
std::optional<uint32_t> wait_reply(canbus::CANSocket& socket, uint16_t slave_id, uint8_t cmd,
                                   uint8_t rid) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(REPLY_TIMEOUT_MS);
    while (true) {
        const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
                                   deadline - std::chrono::steady_clock::now())
                                   .count();
        if (remaining <= 0 || !socket.is_data_available(static_cast<int>(remaining))) {
            return std::nullopt;
        }
        canfd_frame frame;
        if (!socket.read_canfd_frame(frame)) continue;
        if (frame.can_id & CAN_ERR_FLAG) continue;
        if (frame.len < 8) continue;
        const uint16_t from = frame.data[0] | (frame.data[1] << 8);
        if (from != slave_id || frame.data[2] != cmd || frame.data[3] != rid) continue;
        uint32_t value = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) |
                         (static_cast<uint32_t>(frame.data[7]) << 24);
        return value;
    }
}

// Reads and writes are idempotent, so a lost frame or reply is simply resent.
std::optional<uint32_t> transact(canbus::CANSocket& socket, uint16_t slave_id, uint8_t cmd,
                                 uint8_t rid, uint32_t value) {
    const can_frame frame = config_frame(slave_id, cmd, rid, value);
    for (int attempt = 0; attempt < ATTEMPTS; ++attempt) {
        if (!socket.write_can_frame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        if (auto reply = wait_reply(socket, slave_id, cmd, rid)) return reply;
    }
    return std::nullopt;
}

}  // namespace

std::optional<uint32_t> param_read(canbus::CANSocket& socket, uint16_t slave_id, uint8_t rid) {
    return transact(socket, slave_id, CMD_READ, rid, 0);
}

bool param_write_acked(canbus::CANSocket& socket, uint16_t slave_id, uint8_t rid, uint32_t value) {
    return transact(socket, slave_id, CMD_WRITE, rid, value).has_value();
}

bool param_write(canbus::CANSocket& socket, uint16_t slave_id, uint8_t rid, uint32_t value) {
    if (!param_write_acked(socket, slave_id, rid, value)) {
        std::cerr << "✗ Motor " << format_hex_id(slave_id)
                  << " did not acknowledge the write to RID " << static_cast<int>(rid) << "."
                  << std::endl;
        return false;
    }
    // The write reply alone does not prove the value was taken: read it back.
    auto readback = param_read(socket, slave_id, rid);
    if (!readback) {
        std::cerr << "✗ Motor " << format_hex_id(slave_id)
                  << " did not answer the read-back of RID " << static_cast<int>(rid) << "."
                  << std::endl;
        return false;
    }
    if (*readback != value) {
        std::cerr << "✗ Motor " << format_hex_id(slave_id) << " RID " << static_cast<int>(rid)
                  << " reads back 0x" << std::hex << *readback << " instead of 0x" << value
                  << std::dec << "." << std::endl;
        return false;
    }
    return true;
}

bool motor_disable(canbus::CANSocket& socket, uint16_t slave_id) {
    can_frame frame{};
    frame.can_id = slave_id;
    frame.can_dlc = 8;
    for (int i = 0; i < 7; i++) frame.data[i] = 0xFF;
    frame.data[7] = 0xFD;
    if (!socket.write_can_frame(frame)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

bool param_save(canbus::CANSocket& socket, uint16_t slave_id) {
    // Saving is refused while the motor is enabled.
    if (!motor_disable(socket, slave_id)) return false;
    // data[3] = 0x01 as in Damiao's reference implementation.
    if (!socket.write_can_frame(config_frame(slave_id, CMD_SAVE, 0x01, 0))) return false;
    // The motor is unresponsive while it writes its flash.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

}  // namespace openarm::cli
