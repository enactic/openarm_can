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

#include <algorithm>
#include <openarm/can/socket/gripper_component.hpp>

namespace openarm::can::socket {

GripperComponent::GripperComponent(canbus::CANSocket& can_socket)
    : DMDeviceCollection(can_socket) {}

void GripperComponent::init_motor_device(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                                         uint32_t recv_can_id, bool use_fd,
                                         damiao_motor::ControlMode control_mode) {
    // Create the motor
    motor_ = std::make_unique<damiao_motor::Motor>(motor_type, send_can_id, recv_can_id);
    // Create the device with a reference to the motor
    motor_device_ = std::make_shared<damiao_motor::DMCANDevice>(*motor_, CAN_SFF_MASK, use_fd);
    get_device_collection().add_device(motor_device_);

    set_callback_mode_all(damiao_motor::CallbackMode::PARAM);
    set_control_mode_one(0, control_mode);
}

void GripperComponent::open() { set_position(gripper_open_position_); }

void GripperComponent::open(double kp, double kd) {
    set_position_mit(gripper_open_position_, kp, kd);
}

void GripperComponent::close() { set_position(gripper_closed_position_); }

void GripperComponent::close(double kp, double kd) {
    set_position_mit(gripper_closed_position_, kp, kd);
}

void GripperComponent::set_limit(double speed_rad_s, double torque_pu) {
    limit_speed_rad_s_ = std::clamp(speed_rad_s, 0.0, 100.0);
    limit_torque_pu_ = std::clamp(torque_pu, 0.0, 1.0);
}

void GripperComponent::grasp(double torque_pu, double speed_rad_s) {
    set_position(gripper_grasp_position_, speed_rad_s, torque_pu);
}

void GripperComponent::set_position(double gripper_position, std::optional<double> speed_rad_s,
                                    std::optional<double> torque_pu) {
    if (!motor_device_) return;

    double speed_limit = speed_rad_s.value_or(limit_speed_rad_s_);
    double torque_limit = torque_pu.value_or(limit_torque_pu_);
    speed_limit = std::clamp(speed_limit, 0.0, 100.0);
    torque_limit = std::clamp(torque_limit, 0.0, 1.0);
    posforce_control_one(0, damiao_motor::PosForceParam{gripper_to_motor_position(gripper_position),
                                                        speed_limit, torque_limit});
}

void GripperComponent::set_position_mit(double gripper_position, double kp, double kd) {
    if (!motor_device_) return;

    mit_control_one(
        0, damiao_motor::MITParam{kp, kd, gripper_to_motor_position(gripper_position), 0.0, 0.0});
}
}  // namespace openarm::can::socket
