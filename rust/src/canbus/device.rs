//! CAN device trait and base implementations.

use pyo3::prelude::*;
use std::sync::{Arc, Mutex};

use crate::damiao_motor::{
    CallbackMode, CanPacketDecoder, Motor,
};

/// Trait for CAN-communicating devices.
pub trait CANDeviceTrait: Send + Sync {
    /// Get the send CAN ID.
    fn send_can_id(&self) -> u32;

    /// Get the receive CAN ID.
    fn recv_can_id(&self) -> u32;

    /// Handle incoming CAN frame.
    fn callback(&self, can_id: u32, data: &[u8]);

    /// Get the callback mode.
    fn get_callback_mode(&self) -> CallbackMode;

    /// Set the callback mode.
    fn set_callback_mode(&self, mode: CallbackMode);
}

/// Abstract base CAN device for Python.
#[pyclass(subclass)]
#[derive(Clone)]
pub struct CANDevice {
    send_can_id: u32,
    recv_can_id: u32,
}

#[pymethods]
impl CANDevice {
    #[new]
    #[pyo3(signature = (send_can_id, recv_can_id))]
    pub fn new(send_can_id: u32, recv_can_id: u32) -> Self {
        Self {
            send_can_id,
            recv_can_id,
        }
    }

    /// Get the send CAN ID.
    #[getter]
    pub fn get_send_can_id(&self) -> u32 {
        self.send_can_id
    }

    /// Get the receive CAN ID.
    #[getter]
    pub fn get_recv_can_id(&self) -> u32 {
        self.recv_can_id
    }

    fn __repr__(&self) -> String {
        format!(
            "CANDevice(send_id=0x{:X}, recv_id=0x{:X})",
            self.send_can_id, self.recv_can_id
        )
    }
}

/// Motor device state (internal).
struct MotorDeviceState {
    callback_mode: CallbackMode,
}

/// Damiao motor CAN device implementation.
#[pyclass(extends=CANDevice)]
pub struct MotorDeviceCan {
    motor: Motor,
    state: Arc<Mutex<MotorDeviceState>>,
}

#[pymethods]
impl MotorDeviceCan {
    #[new]
    #[pyo3(signature = (motor))]
    pub fn new(motor: Motor) -> (Self, CANDevice) {
        let send_can_id = motor.send_can_id();
        let recv_can_id = motor.recv_can_id();

        let device = Self {
            motor,
            state: Arc::new(Mutex::new(MotorDeviceState {
                callback_mode: CallbackMode::STATE,
            })),
        };

        let base = CANDevice::new(send_can_id, recv_can_id);
        (device, base)
    }

    /// Get the motor.
    #[getter]
    pub fn get_motor(&self) -> Motor {
        self.motor.clone()
    }

    /// Get the callback mode.
    pub fn get_callback_mode(&self) -> CallbackMode {
        self.state.lock().unwrap().callback_mode
    }

    /// Set the callback mode.
    pub fn set_callback_mode(&mut self, mode: CallbackMode) {
        self.state.lock().unwrap().callback_mode = mode;
    }

    /// Process incoming CAN data.
    pub fn callback(&self, _can_id: u32, data: Vec<u8>) {
        self.process_callback(&data);
    }

    fn __repr__(&self) -> String {
        format!(
            "MotorDeviceCan(motor={:?}, mode={:?})",
            self.motor,
            self.get_callback_mode()
        )
    }
}

impl MotorDeviceCan {
    /// Process callback (internal).
    pub(crate) fn process_callback(&self, data: &[u8]) {
        let mode = self.state.lock().unwrap().callback_mode;
        match mode {
            CallbackMode::STATE => {
                CanPacketDecoder::parse_and_update_motor_state(&self.motor, data);
            }
            CallbackMode::PARAM => {
                CanPacketDecoder::parse_and_store_param(&self.motor, data);
            }
            CallbackMode::IGNORE => {}
        }
    }

    /// Get motor reference (internal).
    pub(crate) fn motor(&self) -> &Motor {
        &self.motor
    }

    /// Clone for internal use.
    pub(crate) fn clone_inner(&self) -> Self {
        Self {
            motor: self.motor.clone(),
            state: Arc::clone(&self.state),
        }
    }
}

impl CANDeviceTrait for MotorDeviceCan {
    fn send_can_id(&self) -> u32 {
        self.motor.send_can_id()
    }

    fn recv_can_id(&self) -> u32 {
        self.motor.recv_can_id()
    }

    fn callback(&self, _can_id: u32, data: &[u8]) {
        self.process_callback(data);
    }

    fn get_callback_mode(&self) -> CallbackMode {
        self.state.lock().unwrap().callback_mode
    }

    fn set_callback_mode(&self, mode: CallbackMode) {
        self.state.lock().unwrap().callback_mode = mode;
    }
}
