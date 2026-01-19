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

#include <cmath>
#include <iostream>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <openarm/damiao_motor/dm_motor_device.hpp>

namespace openarm::damiao_motor {

DMCANDevice::DMCANDevice(Motor& motor, canid_t recv_can_mask, bool use_fd)
    : canbus::CANDevice(motor.get_send_can_id(), motor.get_recv_can_id(), recv_can_mask, use_fd),
      motor_(motor),
      callback_mode_(CallbackMode::STATE),
      use_fd_(use_fd) {}

std::vector<uint8_t> DMCANDevice::get_data_from_frame(const can_frame& frame) {
    return std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
}

std::vector<uint8_t> DMCANDevice::get_data_from_frame(const canfd_frame& frame) {
    return std::vector<uint8_t>(frame.data, frame.data + frame.len);
}
void DMCANDevice::callback(const can_frame& frame) {
    if (use_fd_) {
        std::cerr << "WARNING: WRONG CALLBACK FUNCTION" << std::endl;
        return;
    }

    std::vector<uint8_t> data = get_data_from_frame(frame);

    switch (callback_mode_) {
        case STATE:
            if (frame.can_dlc >= 8) {
                // Convert frame data to vector and let Motor handle parsing
                StateResult result = CanPacketDecoder::parse_motor_state_data(motor_, data);
                if (frame.can_id == motor_.get_recv_can_id() && result.valid) {
                    motor_.update_state(result.id, result.err, result.raw_position,
                                        result.raw_velocity, result.raw_torque, result.position,
                                        result.velocity, result.torque, result.t_mos,
                                        result.t_rotor);
                }
            }
            break;
        case PARAM: {
            ParamResult result = CanPacketDecoder::parse_motor_param_data(data);
            if (result.valid) {
                motor_.set_temp_param(result.rid, result.value);
            }
            break;
        }
        case IGNORE:
            return;
        default:
            break;
    }
}

void DMCANDevice::callback(const canfd_frame& frame) {
    if (not use_fd_) {
        std::cerr << "WARNING: CANFD MODE NOT ENABLED" << std::endl;
        return;
    }

    if (frame.can_id != motor_.get_recv_can_id()) {
        std::cerr << "WARNING: CANFD FRAME ID DOES NOT MATCH MOTOR ID" << std::endl;
        return;
    }

    std::vector<uint8_t> data = get_data_from_frame(frame);
    if (callback_mode_ == STATE) {
        StateResult result = CanPacketDecoder::parse_motor_state_data(motor_, data);
        if (result.valid) {
            motor_.update_state(result.id, result.err, result.raw_position, result.raw_velocity,
                                result.raw_torque, result.position, result.velocity, result.torque,
                                result.t_mos, result.t_rotor);
        }
    } else if (callback_mode_ == PARAM) {
        ParamResult result = CanPacketDecoder::parse_motor_param_data(data);
        if (result.valid) {
            motor_.set_temp_param(result.rid, result.value);
        }
    } else if (callback_mode_ == IGNORE) {
        return;
    }
}

can_frame DMCANDevice::create_can_frame(canid_t send_can_id, std::vector<uint8_t> data) {
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = send_can_id;
    frame.can_dlc = data.size();
    std::copy(data.begin(), data.end(), frame.data);
    return frame;
}

canfd_frame DMCANDevice::create_canfd_frame(canid_t send_can_id, std::vector<uint8_t> data) {
    canfd_frame frame;
    frame.can_id = send_can_id;
    frame.len = data.size();
    frame.flags = CANFD_BRS;
    std::copy(data.begin(), data.end(), frame.data);
    return frame;
}

}  // namespace openarm::damiao_motor
