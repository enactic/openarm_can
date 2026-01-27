//! CAN device trait and base implementations.

use std::sync::{Arc, Mutex};

use crate::damiao_motor::{CallbackMode, CanPacketDecoder, Motor};

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

/// Abstract base CAN device.
#[derive(Clone, Debug)]
pub struct CANDevice {
    send_can_id: u32,
    recv_can_id: u32,
}

impl CANDevice {
    /// Create a new CAN device.
    pub fn new(send_can_id: u32, recv_can_id: u32) -> Self {
        Self {
            send_can_id,
            recv_can_id,
        }
    }

    /// Get the send CAN ID.
    pub fn send_can_id(&self) -> u32 {
        self.send_can_id
    }

    /// Get the receive CAN ID.
    pub fn recv_can_id(&self) -> u32 {
        self.recv_can_id
    }
}

/// Motor device state (internal).
struct MotorDeviceState {
    callback_mode: CallbackMode,
}

/// Damiao motor CAN device implementation.
pub struct MotorDeviceCan {
    motor: Motor,
    state: Arc<Mutex<MotorDeviceState>>,
}

impl MotorDeviceCan {
    /// Create a new motor device.
    pub fn new(motor: Motor) -> Self {
        Self {
            motor,
            state: Arc::new(Mutex::new(MotorDeviceState {
                callback_mode: CallbackMode::STATE,
            })),
        }
    }

    /// Get the motor.
    pub fn motor(&self) -> &Motor {
        &self.motor
    }

    /// Get a clone of the motor.
    pub fn motor_clone(&self) -> Motor {
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
    pub fn process_callback(&self, data: &[u8]) {
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

    /// Clone for internal use.
    pub fn clone_inner(&self) -> Self {
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
