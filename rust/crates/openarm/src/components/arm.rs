//! Arm component for controlling multiple arm motors.

use std::sync::{Arc, Mutex};

use crate::canbus::{CANDeviceCollection, MotorDeviceCan};
use crate::damiao_motor::{
    CallbackMode, ControlMode, DMDeviceCollection, MITParam, Motor, MotorType, MotorVariable,
    PosForceParam, PosVelParam,
};
use crate::error::{OpenArmError, Result};

/// Arm component wrapper for multiple arm motors.
pub struct ArmComponent {
    inner: DMDeviceCollection,
}

impl ArmComponent {
    /// Create from shared collection.
    pub fn from_collection(collection: Arc<CANDeviceCollection>) -> Self {
        Self {
            inner: DMDeviceCollection::from_collection(collection),
        }
    }

    /// Add a motor device.
    pub fn add_motor_device(&mut self, motor: Motor, device: Arc<Mutex<MotorDeviceCan>>) {
        self.inner.add_motor_device(motor, device);
    }

    /// Get inner collection.
    pub fn collection(&self) -> &Arc<CANDeviceCollection> {
        self.inner.collection()
    }

    /// Initialize arm motors.
    pub fn init_motor_devices(
        &mut self,
        motor_types: &[MotorType],
        send_can_ids: &[u32],
        recv_can_ids: &[u32],
        control_modes: Option<&[ControlMode]>,
    ) -> Result<()> {
        if motor_types.len() != send_can_ids.len() || motor_types.len() != recv_can_ids.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: motor_types.len(),
                actual: send_can_ids.len(),
            });
        }

        let default_modes = vec![ControlMode::MIT; motor_types.len()];
        let modes = control_modes.unwrap_or(&default_modes);
        if modes.len() != motor_types.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: motor_types.len(),
                actual: modes.len(),
            });
        }

        for ((motor_type, send_id), (recv_id, mode)) in motor_types
            .iter()
            .zip(send_can_ids.iter())
            .zip(recv_can_ids.iter().zip(modes.iter()))
        {
            let motor = Motor::new(*motor_type, *send_id, *recv_id, *mode);
            let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone())));
            self.inner.add_motor_device(motor, device);
        }

        Ok(())
    }

    /// Get the motors.
    pub fn get_motors(&self) -> Vec<Motor> {
        self.inner.get_motors()
    }

    /// Get a specific motor by index.
    pub fn get_motor(&self, index: usize) -> Result<Motor> {
        self.inner.get_motor(index)
    }

    /// Get the number of motors.
    pub fn motor_count(&self) -> usize {
        self.inner.motor_count()
    }

    /// Enable all motors.
    pub fn enable_all(&self) -> Result<()> {
        self.inner.enable_all()
    }

    /// Disable all motors.
    pub fn disable_all(&self) -> Result<()> {
        self.inner.disable_all()
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> Result<()> {
        self.inner.set_zero_all()
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> Result<()> {
        self.inner.refresh_all()
    }

    /// Refresh state for one motor.
    pub fn refresh_one(&self, index: usize) -> Result<()> {
        self.inner.refresh_one(index)
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> Result<()> {
        self.inner.query_param_all(rid)
    }

    /// Query parameter for one motor.
    pub fn query_param_one(&self, index: usize, rid: MotorVariable) -> Result<()> {
        self.inner.query_param_one(index, rid)
    }

    /// MIT control for one motor.
    pub fn mit_control_one(&self, index: usize, param: &MITParam) -> Result<()> {
        self.inner.mit_control_one(index, param)
    }

    /// MIT control for all motors.
    pub fn mit_control_all(&self, params: &[MITParam]) -> Result<()> {
        self.inner.mit_control_all(params)
    }

    /// Position-velocity control for one motor.
    pub fn posvel_control_one(&self, index: usize, param: &PosVelParam) -> Result<()> {
        self.inner.posvel_control_one(index, param)
    }

    /// Position-velocity control for all motors.
    pub fn posvel_control_all(&self, params: &[PosVelParam]) -> Result<()> {
        self.inner.posvel_control_all(params)
    }

    /// Position-force control for one motor.
    pub fn posforce_control_one(&self, index: usize, param: &PosForceParam) -> Result<()> {
        self.inner.posforce_control_one(index, param)
    }

    /// Position-force control for all motors.
    pub fn posforce_control_all(&self, params: &[PosForceParam]) -> Result<()> {
        self.inner.posforce_control_all(params)
    }

    /// Set control mode for one motor.
    pub fn set_control_mode_one(&self, index: usize, mode: ControlMode) -> Result<()> {
        self.inner.set_control_mode_one(index, mode)
    }

    /// Set control mode for all motors.
    pub fn set_control_mode_all(&self, mode: ControlMode) -> Result<()> {
        self.inner.set_control_mode_all(mode)
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        self.inner.set_callback_mode_all(mode)
    }

    /// Receive all available frames.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        self.inner.recv_all(first_timeout_us)
    }
}
