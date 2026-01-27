//! Motor state container.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use super::constants::{ControlMode, MotorType};

/// Internal motor state (protected by mutex).
#[derive(Debug, Clone)]
pub struct MotorState {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub t_mos: i32,
    pub t_rotor: i32,
    pub enabled: bool,
    pub temp_param_dict: HashMap<i32, f64>,
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            t_mos: 0,
            t_rotor: 0,
            enabled: false,
            temp_param_dict: HashMap::new(),
        }
    }
}

/// Motor state container for a single Damiao motor.
#[derive(Clone, Debug)]
pub struct Motor {
    motor_type: MotorType,
    send_can_id: u32,
    recv_can_id: u32,
    control_mode: ControlMode,
    pub(crate) state: Arc<Mutex<MotorState>>,
}

impl Motor {
    /// Create a new motor.
    pub fn new(
        motor_type: MotorType,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: ControlMode,
    ) -> Self {
        Self {
            motor_type,
            send_can_id,
            recv_can_id,
            control_mode,
            state: Arc::new(Mutex::new(MotorState::default())),
        }
    }

    /// Get the motor type.
    pub fn motor_type(&self) -> MotorType {
        self.motor_type
    }

    /// Get the send CAN ID.
    pub fn send_can_id(&self) -> u32 {
        self.send_can_id
    }

    /// Get the receive CAN ID.
    pub fn recv_can_id(&self) -> u32 {
        self.recv_can_id
    }

    /// Get the control mode.
    pub fn control_mode(&self) -> ControlMode {
        self.control_mode
    }

    /// Set the control mode.
    pub fn set_control_mode(&mut self, mode: ControlMode) {
        self.control_mode = mode;
    }

    /// Get the current position (rad).
    pub fn get_position(&self) -> f64 {
        self.state.lock().unwrap().position
    }

    /// Get the current velocity (rad/s).
    pub fn get_velocity(&self) -> f64 {
        self.state.lock().unwrap().velocity
    }

    /// Get the current torque (Nm).
    pub fn get_torque(&self) -> f64 {
        self.state.lock().unwrap().torque
    }

    /// Get the MOS temperature.
    pub fn get_state_tmos(&self) -> i32 {
        self.state.lock().unwrap().t_mos
    }

    /// Get the rotor temperature.
    pub fn get_state_trotor(&self) -> i32 {
        self.state.lock().unwrap().t_rotor
    }

    /// Check if motor is enabled.
    pub fn is_enabled(&self) -> bool {
        self.state.lock().unwrap().enabled
    }

    /// Get a temporary parameter value.
    pub fn get_temp_param(&self, rid: i32) -> Option<f64> {
        self.state.lock().unwrap().temp_param_dict.get(&rid).copied()
    }

    /// Update motor state from decoded CAN response.
    pub fn update_state(
        &self,
        position: f64,
        velocity: f64,
        torque: f64,
        t_mos: i32,
        t_rotor: i32,
    ) {
        let mut state = self.state.lock().unwrap();
        state.position = position;
        state.velocity = velocity;
        state.torque = torque;
        state.t_mos = t_mos;
        state.t_rotor = t_rotor;
    }

    /// Set enabled state.
    pub fn set_enabled(&self, enabled: bool) {
        self.state.lock().unwrap().enabled = enabled;
    }

    /// Store a temporary parameter value.
    pub fn set_temp_param(&self, rid: i32, value: f64) {
        self.state.lock().unwrap().temp_param_dict.insert(rid, value);
    }
}
