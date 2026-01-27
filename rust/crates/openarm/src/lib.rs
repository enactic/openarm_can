//! OpenArm - Pure Rust Implementation
//!
//! This crate provides core functionality for controlling Damiao motors via CAN bus.
//!
//! # Modules
//!
//! - `canbus` - Low-level CAN bus interface (socket, devices, collections)
//! - `damiao_motor` - Damiao motor protocol (encoding, decoding, motor state)
//! - `components` - High-level components (OpenArm, Arm, Gripper)
//! - `error` - Error types

#![allow(dead_code)]

pub mod canbus;
pub mod components;
pub mod damiao_motor;
pub mod error;

// Re-export main types for convenience
pub use canbus::{CANDevice, CANDeviceCollection, CANDeviceTrait, CANSocket, CanFdFrame, CanFrame, MotorDeviceCan};
pub use components::{ArmComponent, GripperComponent, OpenArm};
pub use damiao_motor::{
    CANPacket, CallbackMode, CanPacketDecoder, CanPacketEncoder, ControlMode, DMDeviceCollection,
    LimitParam, MITParam, Motor, MotorState, MotorStateResult, MotorType, MotorVariable, ParamResult,
    PosForceParam, PosVelParam,
};
pub use error::{OpenArmError, Result};
