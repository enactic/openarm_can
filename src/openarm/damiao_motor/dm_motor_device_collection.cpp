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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <openarm/damiao_motor/dm_motor_device_collection.hpp>

namespace openarm::damiao_motor {

DMDeviceCollection::DMDeviceCollection(canbus::CANSocket& can_socket)
    : can_socket_(can_socket),
      can_packet_encoder_(std::make_unique<CanPacketEncoder>()),
      can_packet_decoder_(std::make_unique<CanPacketDecoder>()),
      device_collection_(std::make_unique<canbus::CANDeviceCollection>(can_socket_)) {}

void DMDeviceCollection::enable_all() {
    for (auto dm_device : get_dm_devices()) {
        auto& motor = dm_device->get_motor();
        CANPacket enable_packet = CanPacketEncoder::create_enable_command(motor);
        send_command_to_device(dm_device, enable_packet);
    }
}

void DMDeviceCollection::disable_all() {
    for (auto dm_device : get_dm_devices()) {
        CANPacket disable_packet = CanPacketEncoder::create_disable_command(dm_device->get_motor());
        send_command_to_device(dm_device, disable_packet);
    }
}

void DMDeviceCollection::set_zero(int i) {
    auto dm_device = get_dm_devices().at(i);
    auto zero_packet = CanPacketEncoder::create_set_zero_command(dm_device->get_motor());
    send_command_to_device(dm_device, zero_packet);
}

void DMDeviceCollection::set_zero_all() {
    for (auto dm_device : get_dm_devices()) {
        CANPacket zero_packet = CanPacketEncoder::create_set_zero_command(dm_device->get_motor());
        send_command_to_device(dm_device, zero_packet);
    }
}

void DMDeviceCollection::refresh_one(int i) {
    auto dm_device = get_dm_devices().at(i);
    auto& motor = dm_device->get_motor();
    CANPacket refresh_packet = CanPacketEncoder::create_refresh_command(motor);
    send_command_to_device(dm_device, refresh_packet);
}

void DMDeviceCollection::refresh_all() {
    for (auto dm_device : get_dm_devices()) {
        auto& motor = dm_device->get_motor();
        CANPacket refresh_packet = CanPacketEncoder::create_refresh_command(motor);
        send_command_to_device(dm_device, refresh_packet);
    }
}

void DMDeviceCollection::set_callback_mode_all(CallbackMode callback_mode) {
    for (auto dm_device : get_dm_devices()) {
        dm_device->set_callback_mode(callback_mode);
    }
}

void DMDeviceCollection::query_param_one(int i, int RID) {
    CANPacket param_query =
        CanPacketEncoder::create_query_param_command(get_dm_devices()[i]->get_motor(), RID);
    send_command_to_device(get_dm_devices()[i], param_query);
}

void DMDeviceCollection::query_param_all(int RID) {
    for (auto dm_device : get_dm_devices()) {
        CANPacket param_query =
            CanPacketEncoder::create_query_param_command(dm_device->get_motor(), RID);
        send_command_to_device(dm_device, param_query);
    }
}

void DMDeviceCollection::send_command_to_device(std::shared_ptr<DMCANDevice> dm_device,
                                                const CANPacket& packet) {
    if (can_socket_.is_canfd_enabled()) {
        canfd_frame frame = dm_device->create_canfd_frame(packet.send_can_id, packet.data);
        can_socket_.write_canfd_frame(frame);
    } else {
        can_frame frame = dm_device->create_can_frame(packet.send_can_id, packet.data);
        can_socket_.write_can_frame(frame);
    }
}

void DMDeviceCollection::mit_control_one(int i, const MITParam& mit_param) {
    CANPacket mit_cmd =
        CanPacketEncoder::create_mit_control_command(get_dm_devices()[i]->get_motor(), mit_param);
    send_command_to_device(get_dm_devices()[i], mit_cmd);
}

void DMDeviceCollection::mit_control_all(const std::vector<MITParam>& mit_params) {
    for (size_t i = 0; i < mit_params.size(); i++) {
        mit_control_one(i, mit_params[i]);
    }
}

void DMDeviceCollection::posvel_control_one(int i, const PosVelParam& posvel_param) {
    CANPacket posvel_cmd = CanPacketEncoder::create_posvel_control_command(
        get_dm_devices()[i]->get_motor(), posvel_param);
    send_command_to_device(get_dm_devices()[i], posvel_cmd);
}

void DMDeviceCollection::posvel_control_all(const std::vector<PosVelParam>& posvel_params) {
    for (size_t i = 0; i < posvel_params.size(); i++) {
        posvel_control_one(i, posvel_params[i]);
    }
}

std::vector<Motor> DMDeviceCollection::get_motors() const {
    std::vector<Motor> motors;
    for (auto dm_device : get_dm_devices()) {
        motors.push_back(dm_device->get_motor());
    }
    return motors;
}

Motor DMDeviceCollection::get_motor(int i) const { return get_dm_devices().at(i)->get_motor(); }

std::vector<std::shared_ptr<DMCANDevice>> DMDeviceCollection::get_dm_devices() const {
    std::vector<std::shared_ptr<DMCANDevice>> dm_devices;
    for (const auto& [id, device] : device_collection_->get_devices()) {
        auto dm_device = std::dynamic_pointer_cast<DMCANDevice>(device);
        if (dm_device) {
            dm_devices.push_back(dm_device);
        }
    }
    return dm_devices;
}

}  // namespace openarm::damiao_motor
