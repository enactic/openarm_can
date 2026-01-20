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

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

// Include the C++ headers
#include <linux/can.h>
#include <linux/can/raw.h>

#include <openarm/can/socket/arm_component.hpp>
#include <openarm/can/socket/gripper_component.hpp>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/canbus/can_device.hpp>
#include <openarm/canbus/can_device_collection.hpp>
#include <openarm/canbus/can_socket.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <openarm/damiao_motor/dm_motor_device.hpp>
#include <openarm/damiao_motor/dm_motor_device_collection.hpp>

using namespace openarm::canbus;
using namespace openarm::damiao_motor;
using namespace openarm::can::socket;

namespace nb = nanobind;

NB_MODULE(openarm_can, m) {
    m.doc() = "OpenArm CAN Python bindings for motor control via SocketCAN";

    // ============================================================================
    // DAMIAO MOTOR NAMESPACE - ENUMS AND CONSTANTS
    // ============================================================================

    // Motor Type enum
    nb::enum_<MotorType>(m, "MotorType")
        .value("DM3507", MotorType::DM3507)
        .value("DM4310", MotorType::DM4310)
        .value("DM4310_48V", MotorType::DM4310_48V)
        .value("DM4340", MotorType::DM4340)
        .value("DM4340_48V", MotorType::DM4340_48V)
        .value("DM6006", MotorType::DM6006)
        .value("DM8006", MotorType::DM8006)
        .value("DM8009", MotorType::DM8009)
        .value("DM10010L", MotorType::DM10010L)
        .value("DM10010", MotorType::DM10010)
        .value("DMH3510", MotorType::DMH3510)
        .value("DMH6215", MotorType::DMH6215)
        .value("DMG6220", MotorType::DMG6220)
        .value("COUNT", MotorType::COUNT)
        .export_values();

    // Motor Variable enum
    nb::enum_<RID>(m, "MotorVariable")
        .value("UV_Value", RID::UV_Value)
        .value("KT_Value", RID::KT_Value)
        .value("OT_Value", RID::OT_Value)
        .value("OC_Value", RID::OC_Value)
        .value("ACC", RID::ACC)
        .value("DEC", RID::DEC)
        .value("MAX_SPD", RID::MAX_SPD)
        .value("MST_ID", RID::MST_ID)
        .value("ESC_ID", RID::ESC_ID)
        .value("TIMEOUT", RID::TIMEOUT)
        .value("CTRL_MODE", RID::CTRL_MODE)
        .value("Damp", RID::Damp)
        .value("Inertia", RID::Inertia)
        .value("hw_ver", RID::hw_ver)
        .value("sw_ver", RID::sw_ver)
        .value("SN", RID::SN)
        .value("NPP", RID::NPP)
        .value("Rs", RID::Rs)
        .value("LS", RID::LS)
        .value("Flux", RID::Flux)
        .value("Gr", RID::Gr)
        .value("PMAX", RID::PMAX)
        .value("VMAX", RID::VMAX)
        .value("TMAX", RID::TMAX)
        .value("I_BW", RID::I_BW)
        .value("KP_ASR", RID::KP_ASR)
        .value("KI_ASR", RID::KI_ASR)
        .value("KP_APR", RID::KP_APR)
        .value("KI_APR", RID::KI_APR)
        .value("OV_Value", RID::OV_Value)
        .value("GREF", RID::GREF)
        .value("Deta", RID::Deta)
        .value("V_BW", RID::V_BW)
        .value("IQ_c1", RID::IQ_c1)
        .value("VL_c1", RID::VL_c1)
        .value("can_br", RID::can_br)
        .value("sub_ver", RID::sub_ver)
        .value("u_off", RID::u_off)
        .value("v_off", RID::v_off)
        .value("k1", RID::k1)
        .value("k2", RID::k2)
        .value("m_off", RID::m_off)
        .value("dir", RID::dir)
        .value("p_m", RID::p_m)
        .value("xout", RID::xout)
        .value("COUNT", RID::COUNT)
        .export_values();

    // Callback Mode enum
    nb::enum_<CallbackMode>(m, "CallbackMode")
        .value("STATE", CallbackMode::STATE)
        .value("PARAM", CallbackMode::PARAM)
        .value("IGNORE", CallbackMode::IGNORE)
        .export_values();

    nb::enum_<ControlMode>(m, "ControlMode")
        .value("MIT", ControlMode::MIT)
        .value("POS_VEL", ControlMode::POS_VEL)
        .value("VEL", ControlMode::VEL)
        .value("POS_FORCE", ControlMode::POS_FORCE)
        .export_values();

    // ============================================================================
    // DAMIAO MOTOR NAMESPACE - STRUCTS
    // ============================================================================

    // LimitParam struct
    nb::class_<LimitParam>(m, "LimitParam")
        .def(nb::init<>())
        .def_rw("pMax", &LimitParam::pMax)
        .def_rw("vMax", &LimitParam::vMax)
        .def_rw("tMax", &LimitParam::tMax);

    // ParamResult struct
    nb::class_<ParamResult>(m, "ParamResult")
        .def(nb::init<>())
        .def_rw("rid", &ParamResult::rid)
        .def_rw("value", &ParamResult::value)
        .def_rw("valid", &ParamResult::valid);

    // MotorStateResult struct
    nb::class_<StateResult>(m, "MotorStateResult")
        .def(nb::init<>())
        .def_rw("position", &StateResult::position)
        .def_rw("velocity", &StateResult::velocity)
        .def_rw("torque", &StateResult::torque)
        .def_rw("t_mos", &StateResult::t_mos)
        .def_rw("t_rotor", &StateResult::t_rotor)
        .def_rw("valid", &StateResult::valid);

    // CANPacket struct
    nb::class_<CANPacket>(m, "CANPacket")
        .def(nb::init<>())
        .def_rw("send_can_id", &CANPacket::send_can_id)
        .def_rw("data", &CANPacket::data);

    // MITParam struct
    nb::class_<MITParam>(m, "MITParam")
        .def(nb::init<>())
        .def(
            "__init__",
            [](MITParam* param, double kp, double kd, double q, double dq, double tau) {
                new (param) MITParam(MITParam{kp, kd, q, dq, tau});
            },
            nb::arg("kp"), nb::arg("kd"), nb::arg("q"), nb::arg("dq"), nb::arg("tau"))
        .def_rw("kp", &MITParam::kp)
        .def_rw("kd", &MITParam::kd)
        .def_rw("q", &MITParam::q)
        .def_rw("dq", &MITParam::dq)
        .def_rw("tau", &MITParam::tau);

    // PosVelParam struct
    nb::class_<PosVelParam>(m, "PosVelParam")
        .def(nb::init<>())
        .def(
            "__init__",
            [](PosVelParam* param, double q, double dq) {
                new (param) PosVelParam(PosVelParam{q, dq});
            },
            nb::arg("q"), nb::arg("dq"))
        .def_rw("q", &PosVelParam::q)
        .def_rw("dq", &PosVelParam::dq);

    // PosForceParam struct
    nb::class_<PosForceParam>(m, "PosForceParam")
        .def(nb::init<>())
        .def(
            "__init__",
            [](PosForceParam* param, double q, double dq, double i) {
                new (param) PosForceParam(PosForceParam{q, dq, i});
            },
            nb::arg("q"), nb::arg("dq"), nb::arg("i"))
        .def_rw("q", &PosForceParam::q)
        .def_rw("dq", &PosForceParam::dq)
        .def_rw("i", &PosForceParam::i);

    // ============================================================================
    // DAMIAO MOTOR NAMESPACE - MAIN CLASSES
    // ============================================================================

    // Motor class
    nb::class_<Motor>(m, "Motor")
        .def(nb::init<MotorType, uint32_t, uint32_t>(), nb::arg("motor_type"),
             nb::arg("send_can_id"), nb::arg("recv_can_id"))
        .def("get_position", &Motor::get_position)
        .def("get_velocity", &Motor::get_velocity)
        .def("get_torque", &Motor::get_torque)
        .def("get_state_tmos", &Motor::get_state_tmos)
        .def("get_state_trotor", &Motor::get_state_trotor)
        .def("get_send_can_id", &Motor::get_send_can_id)
        .def("get_recv_can_id", &Motor::get_recv_can_id)
        .def("get_motor_type", &Motor::get_motor_type)
        .def("is_enabled", &Motor::is_enabled)
        .def("get_param", &Motor::get_param, nb::arg("rid"))
        .def_static("get_limit_param", &Motor::get_limit_param, nb::arg("motor_type"));

    // MotorControl class
    nb::class_<CanPacketEncoder>(m, "CanPacketEncoder")
        .def_static("create_refresh_command", &CanPacketEncoder::create_refresh_command,
                    nb::arg("motor"))
        .def_static("create_enable_command", &CanPacketEncoder::create_enable_command,
                    nb::arg("motor"))
        .def_static("create_disable_command", &CanPacketEncoder::create_disable_command,
                    nb::arg("motor"))
        .def_static("create_set_zero_command", &CanPacketEncoder::create_set_zero_command,
                    nb::arg("motor"))
        .def_static("create_mit_control_command", &CanPacketEncoder::create_mit_control_command,
                    nb::arg("motor"), nb::arg("mit_param"))
        .def_static("create_posvel_control_command",
                    &CanPacketEncoder::create_posvel_control_command, nb::arg("motor"),
                    nb::arg("posvel_param"))
        .def_static("create_posforce_control_command",
                    &CanPacketEncoder::create_posforce_control_command, nb::arg("motor"),
                    nb::arg("posforce_param"))
        .def_static("create_query_param_command", &CanPacketEncoder::create_query_param_command,
                    nb::arg("motor"), nb::arg("rid"));

    nb::class_<CanPacketDecoder>(m, "CanPacketDecoder")
        .def_static("parse_motor_state_data", &CanPacketDecoder::parse_motor_state_data,
                    nb::arg("motor"), nb::arg("data"))
        .def_static("parse_motor_param_data", &CanPacketDecoder::parse_motor_param_data,
                    nb::arg("data"));

    // ============================================================================
    // CANBUS NAMESPACE - EXCEPTIONS AND BASE CLASSES
    // ============================================================================

    // CAN Socket Exception
    nb::exception<CANSocketException>(m, "CANSocketException");

    // CANDevice base class (MUST be defined before derived classes)
    nb::class_<CANDevice>(m, "CANDevice")
        .def("get_send_can_id", &CANDevice::get_send_can_id)
        .def("get_recv_can_id", &CANDevice::get_recv_can_id)
        .def("get_recv_can_mask", &CANDevice::get_recv_can_mask)
        .def("is_fd_enabled", &CANDevice::is_fd_enabled);

    // MotorDeviceCan class (NOW can inherit from CANDevice)
    nb::class_<DMCANDevice, CANDevice>(m, "MotorDeviceCan")
        .def(nb::init<Motor&, canid_t, bool>(), nb::arg("motor"), nb::arg("recv_can_mask"),
             nb::arg("use_fd"))
        .def("get_motor", &DMCANDevice::get_motor, nb::rv_policy::reference)
        .def("callback",
             static_cast<void (DMCANDevice::*)(const can_frame&)>(&DMCANDevice::callback),
             nb::arg("frame"))
        .def("callback",
             static_cast<void (DMCANDevice::*)(const canfd_frame&)>(&DMCANDevice::callback),
             nb::arg("frame"))
        .def("create_can_frame", &DMCANDevice::create_can_frame, nb::arg("send_can_id"),
             nb::arg("data"))
        .def("create_canfd_frame", &DMCANDevice::create_canfd_frame, nb::arg("send_can_id"),
             nb::arg("data"))
        .def("set_callback_mode", &DMCANDevice::set_callback_mode, nb::arg("callback_mode"));

    // CANDeviceCollection class
    nb::class_<CANDeviceCollection>(m, "CANDeviceCollection")
        .def(nb::init<CANSocket&>(), nb::arg("can_socket"))
        .def(
            "add_device",
            [](CANDeviceCollection& self, std::shared_ptr<CANDevice> device) {
                self.add_device(device);
            },
            nb::arg("device"))
        .def(
            "remove_device",
            [](CANDeviceCollection& self, std::shared_ptr<CANDevice> device) {
                self.remove_device(device);
            },
            nb::arg("device"))
        .def("dispatch_frame_callback",
             static_cast<void (CANDeviceCollection::*)(can_frame&)>(
                 &CANDeviceCollection::dispatch_frame_callback),
             nb::arg("frame"))
        .def("dispatch_frame_callback",
             static_cast<void (CANDeviceCollection::*)(canfd_frame&)>(
                 &CANDeviceCollection::dispatch_frame_callback),
             nb::arg("frame"))
        .def("get_devices", &CANDeviceCollection::get_devices);

    // CAN Socket class
    nb::class_<CANSocket>(m, "CANSocket")
        .def(nb::init<const std::string&, bool>(), nb::arg("interface"),
             nb::arg("enable_fd") = false)
        .def("get_socket_fd", &CANSocket::get_socket_fd)
        .def("get_interface", &CANSocket::get_interface)
        .def("is_canfd_enabled", &CANSocket::is_canfd_enabled)
        .def("is_initialized", &CANSocket::is_initialized)
        .def(
            "read_raw_frame",
            [](CANSocket& self, size_t buffer_size) {
                std::vector<uint8_t> buffer(buffer_size);
                ssize_t bytes_read = self.read_raw_frame(buffer.data(), buffer_size);
                if (bytes_read > 0) {
                    buffer.resize(bytes_read);
                    return nb::bytes(reinterpret_cast<const char*>(buffer.data()), bytes_read);
                }
                return nb::bytes();
            },
            nb::arg("buffer_size"))
        .def(
            "write_raw_frame",
            [](CANSocket& self, nb::bytes data) {
                // nb::bytes::data() is available since nanobind 2.0.0.
                return self.write_raw_frame(data.c_str(), data.size());
            },
            nb::arg("data"))
        .def("write_can_frame", &CANSocket::write_can_frame, nb::arg("frame"))
        .def("read_can_frame", &CANSocket::read_can_frame, nb::arg("frame"))
        .def("write_canfd_frame", &CANSocket::write_canfd_frame, nb::arg("frame"))
        .def("read_canfd_frame", &CANSocket::read_canfd_frame, nb::arg("frame"));

    // ============================================================================
    // LINUX CAN FRAME STRUCTURES
    // ============================================================================

    // CAN frame structures
    nb::class_<can_frame>(m, "CanFrame")
        .def(nb::init<>())
        .def_rw("can_id", &can_frame::can_id)
        .def_rw("can_dlc", &can_frame::can_dlc)
        .def_prop_rw(
            "data",
            [](const can_frame& frame) {
                return nb::bytes(reinterpret_cast<const char*>(frame.data), frame.can_dlc);
            },
            [](can_frame& frame, nb::bytes data) {
                size_t len = std::min(data.size(), sizeof(frame.data));
                frame.can_dlc = len;
                // nb::bytes::data() is available since nanobind 2.0.0.
                std::memcpy(frame.data, data.c_str(), len);
            });

    nb::class_<canfd_frame>(m, "CanFdFrame")
        .def(nb::init<>())
        .def_rw("can_id", &canfd_frame::can_id)
        .def_rw("len", &canfd_frame::len)
        .def_rw("flags", &canfd_frame::flags)
        .def_prop_rw(
            "data",
            [](const canfd_frame& frame) {
                return nb::bytes(reinterpret_cast<const char*>(frame.data), frame.len);
            },
            [](canfd_frame& frame, nb::bytes data) {
                size_t len = std::min(data.size(), sizeof(frame.data));
                frame.len = len;
                // nb::bytes::data() is available since nanobind 2.0.0.
                std::memcpy(frame.data, data.c_str(), len);
            });

    // ============================================================================
    // TOP-LEVEL COMPONENT CLASSES
    // ============================================================================

    // DMDeviceCollection class (base class for ArmComponent and
    // GripperComponent)
    nb::class_<DMDeviceCollection>(m, "DMDeviceCollection")
        .def(nb::init<CANSocket&>(), nb::arg("can_socket"))
        .def("enable_all", &DMDeviceCollection::enable_all)
        .def("disable_all", &DMDeviceCollection::disable_all)
        .def("set_zero_all", &DMDeviceCollection::set_zero_all)
        .def("refresh_all", &DMDeviceCollection::refresh_all)
        .def("set_callback_mode_all", &DMDeviceCollection::set_callback_mode_all,
             nb::arg("callback_mode"))
        .def("query_param_all", &DMDeviceCollection::query_param_all, nb::arg("rid"))
        .def("set_control_mode_one", &DMDeviceCollection::set_control_mode_one, nb::arg("index"),
             nb::arg("mode"))
        .def("set_control_mode_all", &DMDeviceCollection::set_control_mode_all, nb::arg("mode"))
        .def("mit_control_one", &DMDeviceCollection::mit_control_one, nb::arg("index"),
             nb::arg("mit_param"))
        .def("mit_control_all", &DMDeviceCollection::mit_control_all, nb::arg("mit_params"))
        .def("posvel_control_one", &DMDeviceCollection::posvel_control_one, nb::arg("index"),
             nb::arg("posvel_param"))
        .def("posvel_control_all", &DMDeviceCollection::posvel_control_all,
             nb::arg("posvel_params"))
        .def("posforce_control_one", &DMDeviceCollection::posforce_control_one, nb::arg("index"),
             nb::arg("posforce_param"))
        .def("posforce_control_all", &DMDeviceCollection::posforce_control_all,
             nb::arg("posforce_params"))
        .def("get_motors", &DMDeviceCollection::get_motors)
        .def("get_device_collection", &DMDeviceCollection::get_device_collection,
             nb::rv_policy::reference);

    // ArmComponent class
    nb::class_<ArmComponent, DMDeviceCollection>(m, "ArmComponent")
        .def(nb::init<CANSocket&>(), nb::arg("can_socket"))
        .def("init_motor_devices", &ArmComponent::init_motor_devices, nb::arg("motor_types"),
             nb::arg("send_can_ids"), nb::arg("recv_can_ids"), nb::arg("use_fd"),
             nb::arg("control_modes") = std::vector<ControlMode>{});

    // GripperComponent class
    nb::class_<GripperComponent, DMDeviceCollection>(m, "GripperComponent")
        .def(nb::init<CANSocket&>(), nb::arg("can_socket"))
        .def("init_motor_device", &GripperComponent::init_motor_device, nb::arg("motor_type"),
             nb::arg("send_can_id"), nb::arg("recv_can_id"), nb::arg("use_fd"),
             nb::arg("control_mode") = ControlMode::MIT)
        .def("open", nb::overload_cast<>(&GripperComponent::open))
        .def("open", nb::overload_cast<double, double>(&GripperComponent::open), nb::arg("kp"),
             nb::arg("kd"))
        .def("close", nb::overload_cast<>(&GripperComponent::close))
        .def("close", nb::overload_cast<double, double>(&GripperComponent::close), nb::arg("kp"),
             nb::arg("kd"))
        .def("set_limit", &GripperComponent::set_limit, nb::arg("speed_rad_s"),
             nb::arg("torque_pu"),
             "Set default gripper limits for pos-force control.\n"
             "speed_rad_s: max closing speed in rad/s.\n"
             "torque_pu: per-unit current limit [0, 1].")
        .def("grasp", &GripperComponent::grasp, nb::arg("torque_pu"), nb::arg("speed_rad_s") = 5.0,
             "Close to a negative target to keep force applied.\n"
             "torque_pu: per-unit current limit [0, 1].\n"
             "speed_rad_s: max closing speed in rad/s.")
        .def("set_position", &GripperComponent::set_position, nb::arg("position"),
             nb::arg("speed_rad_s") = nb::none(), nb::arg("torque_pu") = nb::none(),
             nb::arg("raw_position") = false,
             "Command gripper position with optional per-call limit overrides.\n"
             "position: gripper target (0=closed, 1=open).\n"
             "speed_rad_s: max closing speed in rad/s.\n"
             "torque_pu: per-unit current limit [0, 1].\n"
             "raw_position: treat position as raw motor radians if true.")
        .def("set_zero", &GripperComponent::set_zero, "Set current position as zero.")
        .def("set_position_mit", &GripperComponent::set_position_mit, nb::arg("position"),
             nb::arg("kp") = 50.0, nb::arg("kd") = 1.0)
        .def("get_motor", &GripperComponent::get_motor, nb::rv_policy::reference_internal);

    // OpenArm class (main high-level interface)
    nb::class_<OpenArm>(m, "OpenArm")
        .def(nb::init<const std::string&, bool>(), nb::arg("can_interface"),
             nb::arg("enable_fd") = false)
        .def("init_arm_motors", &OpenArm::init_arm_motors, nb::arg("motor_types"),
             nb::arg("send_can_ids"), nb::arg("recv_can_ids"),
             nb::arg("control_modes") = std::vector<ControlMode>{})
        .def("init_gripper_motor", &OpenArm::init_gripper_motor, nb::arg("motor_type"),
             nb::arg("send_can_id"), nb::arg("recv_can_id"),
             nb::arg("control_mode") = ControlMode::MIT)
        .def("get_arm", &OpenArm::get_arm, nb::rv_policy::reference)
        .def("get_gripper", &OpenArm::get_gripper, nb::rv_policy::reference)
        .def("get_master_can_device_collection", &OpenArm::get_master_can_device_collection,
             nb::rv_policy::reference)
        .def("enable_all", &OpenArm::enable_all)
        .def("disable_all", &OpenArm::disable_all)
        .def("set_zero_all", &OpenArm::set_zero_all)
        .def("refresh_all", &OpenArm::refresh_all)
        .def("recv_all", &OpenArm::recv_all, nb::arg("first_timeout_us") = 500)
        .def("set_callback_mode_all", &OpenArm::set_callback_mode_all, nb::arg("callback_mode"))
        .def("query_param_all", &OpenArm::query_param_all, nb::arg("rid"));
}
