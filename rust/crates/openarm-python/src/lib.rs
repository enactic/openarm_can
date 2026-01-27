//! Python bindings for OpenArm CAN motor control.
//!
//! This crate provides PyO3 wrappers around the core openarm crate.

use pyo3::prelude::*;

mod canbus;
mod components;
mod damiao_motor;
mod error;

use canbus::{PyCANDevice, PyCANSocket, PyCanFdFrame, PyCanFrame};
use components::{PyArmComponent, PyGripperComponent, PyOpenArm};
use damiao_motor::{
    PyCANPacket, PyCallbackMode, PyCanPacketDecoder, PyCanPacketEncoder, PyControlMode,
    PyLimitParam, PyMITParam, PyMotor, PyMotorStateResult, PyMotorType, PyMotorVariable,
    PyParamResult, PyPosForceParam, PyPosVelParam,
};
use error::CANSocketException;

/// OpenArm CAN Python module.
#[pymodule]
fn openarm_can(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Enums
    m.add_class::<PyMotorType>()?;
    m.add_class::<PyMotorVariable>()?;
    m.add_class::<PyCallbackMode>()?;
    m.add_class::<PyControlMode>()?;

    // Data structures
    m.add_class::<PyLimitParam>()?;
    m.add_class::<PyParamResult>()?;
    m.add_class::<PyMotorStateResult>()?;
    m.add_class::<PyCanFrame>()?;
    m.add_class::<PyCanFdFrame>()?;
    m.add_class::<PyMITParam>()?;
    m.add_class::<PyPosVelParam>()?;
    m.add_class::<PyPosForceParam>()?;
    m.add_class::<PyCANPacket>()?;

    // Classes
    m.add_class::<PyMotor>()?;
    m.add_class::<PyCANSocket>()?;
    m.add_class::<PyCANDevice>()?;
    m.add_class::<PyOpenArm>()?;
    m.add_class::<PyArmComponent>()?;
    m.add_class::<PyGripperComponent>()?;
    m.add_class::<PyCanPacketEncoder>()?;
    m.add_class::<PyCanPacketDecoder>()?;

    // Exception
    m.add("CANSocketException", m.py().get_type::<CANSocketException>())?;

    Ok(())
}
