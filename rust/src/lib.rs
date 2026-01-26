//! OpenArm CAN - Pure Rust Implementation
//!
//! This crate provides Python bindings for controlling Damiao motors via CAN bus.

#![allow(dead_code)]

use pyo3::prelude::*;

pub mod canbus;
pub mod components;
pub mod damiao_motor;

use canbus::{CANDevice, CANDeviceCollection, CANSocket, CanFdFrame, CanFrame, MotorDeviceCan, CANSocketException};
use components::{ArmComponent, GripperComponent, OpenArm};
use damiao_motor::{
    CANPacket, CallbackMode, CanPacketDecoder, CanPacketEncoder, ControlMode, DMDeviceCollection,
    LimitParam, MITParam, Motor, MotorStateResult, MotorType, MotorVariable, ParamResult,
    PosForceParam, PosVelParam,
};

/// OpenArm CAN Python module.
#[pymodule]
fn openarm_can(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Enums
    m.add_class::<MotorType>()?;
    m.add_class::<MotorVariable>()?;
    m.add_class::<CallbackMode>()?;
    m.add_class::<ControlMode>()?;

    // Data structures
    m.add_class::<LimitParam>()?;
    m.add_class::<ParamResult>()?;
    m.add_class::<MotorStateResult>()?;
    m.add_class::<CanFrame>()?;
    m.add_class::<CanFdFrame>()?;
    m.add_class::<MITParam>()?;
    m.add_class::<PosVelParam>()?;
    m.add_class::<PosForceParam>()?;
    m.add_class::<CANPacket>()?;

    // Classes
    m.add_class::<Motor>()?;
    m.add_class::<CANSocket>()?;
    m.add_class::<CANDevice>()?;
    m.add_class::<MotorDeviceCan>()?;
    m.add_class::<CANDeviceCollection>()?;
    m.add_class::<DMDeviceCollection>()?;
    m.add_class::<ArmComponent>()?;
    m.add_class::<GripperComponent>()?;
    m.add_class::<OpenArm>()?;
    m.add_class::<CanPacketEncoder>()?;
    m.add_class::<CanPacketDecoder>()?;

    // Exception
    m.add("CANSocketException", m.py().get_type::<CANSocketException>())?;

    Ok(())
}
