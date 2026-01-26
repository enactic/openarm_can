//! Arm component for controlling multiple arm motors.

use pyo3::prelude::*;
use std::sync::{Arc, Mutex};

use crate::canbus::{CANDeviceCollection, MotorDeviceCan};
use crate::damiao_motor::{
    CallbackMode, ControlMode, DMDeviceCollection, MITParam, Motor, MotorType, MotorVariable,
    PosForceParam, PosVelParam,
};

/// Arm component wrapper for multiple arm motors.
#[pyclass]
pub struct ArmComponent {
    inner: DMDeviceCollection,
}

#[pymethods]
impl ArmComponent {
    /// Initialize arm motors.
    #[pyo3(signature = (motor_types, send_can_ids, recv_can_ids, control_modes=None))]
    pub fn init_motor_devices(
        &mut self,
        motor_types: Vec<MotorType>,
        send_can_ids: Vec<u32>,
        recv_can_ids: Vec<u32>,
        control_modes: Option<Vec<ControlMode>>,
    ) -> PyResult<()> {
        if motor_types.len() != send_can_ids.len() || motor_types.len() != recv_can_ids.len() {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "motor_types, send_can_ids, and recv_can_ids must have the same length",
            ));
        }

        let modes = control_modes.unwrap_or_else(|| vec![ControlMode::MIT; motor_types.len()]);
        if modes.len() != motor_types.len() {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "control_modes must have the same length as motor_types",
            ));
        }

        for ((motor_type, send_id), (recv_id, mode)) in motor_types
            .iter()
            .zip(send_can_ids.iter())
            .zip(recv_can_ids.iter().zip(modes.iter()))
        {
            let motor = Motor::new(*motor_type, *send_id, *recv_id, *mode);
            let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone()).0));
            self.inner.add_motor_device(motor, device);
        }

        Ok(())
    }

    /// Get the motors.
    pub fn get_motors(&self) -> Vec<Motor> {
        self.inner.get_motors()
    }

    /// Get a specific motor by index.
    pub fn get_motor(&self, index: usize) -> PyResult<Motor> {
        self.inner.get_motor(index)
    }

    /// Get the number of motors.
    pub fn motor_count(&self) -> usize {
        self.inner.motor_count()
    }

    /// Enable all motors.
    pub fn enable_all(&self) -> PyResult<()> {
        self.inner.enable_all()
    }

    /// Disable all motors.
    pub fn disable_all(&self) -> PyResult<()> {
        self.inner.disable_all()
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> PyResult<()> {
        self.inner.set_zero_all()
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> PyResult<()> {
        self.inner.refresh_all()
    }

    /// Refresh state for one motor.
    pub fn refresh_one(&self, index: usize) -> PyResult<()> {
        self.inner.refresh_one(index)
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> PyResult<()> {
        self.inner.query_param_all(rid)
    }

    /// Query parameter for one motor.
    pub fn query_param_one(&self, index: usize, rid: MotorVariable) -> PyResult<()> {
        self.inner.query_param_one(index, rid)
    }

    /// MIT control for one motor.
    pub fn mit_control_one(&self, index: usize, param: &MITParam) -> PyResult<()> {
        self.inner.mit_control_one(index, param)
    }

    /// MIT control for all motors.
    pub fn mit_control_all(&self, params: Vec<MITParam>) -> PyResult<()> {
        self.inner.mit_control_all(params)
    }

    /// Position-velocity control for one motor.
    pub fn posvel_control_one(&self, index: usize, param: &PosVelParam) -> PyResult<()> {
        self.inner.posvel_control_one(index, param)
    }

    /// Position-velocity control for all motors.
    pub fn posvel_control_all(&self, params: Vec<PosVelParam>) -> PyResult<()> {
        self.inner.posvel_control_all(params)
    }

    /// Position-force control for one motor.
    pub fn posforce_control_one(&self, index: usize, param: &PosForceParam) -> PyResult<()> {
        self.inner.posforce_control_one(index, param)
    }

    /// Position-force control for all motors.
    pub fn posforce_control_all(&self, params: Vec<PosForceParam>) -> PyResult<()> {
        self.inner.posforce_control_all(params)
    }

    /// Set control mode for one motor.
    pub fn set_control_mode_one(&self, index: usize, mode: ControlMode) -> PyResult<()> {
        self.inner.set_control_mode_one(index, mode)
    }

    /// Set control mode for all motors.
    pub fn set_control_mode_all(&self, mode: ControlMode) -> PyResult<()> {
        self.inner.set_control_mode_all(mode)
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        self.inner.set_callback_mode_all(mode)
    }

    /// Receive all available frames.
    #[pyo3(signature = (first_timeout_us=500))]
    pub fn recv_all(&self, first_timeout_us: u64) -> PyResult<usize> {
        self.inner.recv_all(first_timeout_us)
    }

    fn __repr__(&self) -> String {
        format!("ArmComponent(motors={})", self.inner.motor_count())
    }
}

impl ArmComponent {
    /// Create from shared collection (internal).
    pub(crate) fn from_collection(collection: Arc<CANDeviceCollection>) -> Self {
        Self {
            inner: DMDeviceCollection::from_collection(collection),
        }
    }

    /// Add a motor device (internal).
    pub(crate) fn add_motor_device(&mut self, motor: Motor, device: Arc<Mutex<MotorDeviceCan>>) {
        self.inner.add_motor_device(motor, device);
    }

    /// Get inner collection (internal).
    pub(crate) fn collection(&self) -> &Arc<CANDeviceCollection> {
        self.inner.collection()
    }
}
