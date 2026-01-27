//! Gripper component for controlling a gripper motor.

use std::sync::{Arc, Mutex};

use crate::canbus::{CANDeviceCollection, MotorDeviceCan};
use crate::damiao_motor::{
    CallbackMode, ControlMode, DMDeviceCollection, MITParam, Motor, MotorType,
    MotorVariable, PosForceParam, PosVelParam,
};
use crate::error::Result;

/// Gripper component for single-motor gripper control.
pub struct GripperComponent {
    inner: DMDeviceCollection,
    // Position mapping constants
    gripper_open_position: f64,
    gripper_closed_position: f64,
    gripper_grasp_position: f64,
    motor_open_position: f64,
    motor_closed_position: f64,
    // Default limits
    default_speed_rad_s: f64,
    default_torque_pu: f64,
}

impl GripperComponent {
    /// Create from shared collection.
    pub fn from_collection(collection: Arc<CANDeviceCollection>) -> Self {
        Self {
            inner: DMDeviceCollection::from_collection(collection),
            gripper_open_position: 1.0,
            gripper_closed_position: 0.0,
            gripper_grasp_position: -0.1,
            motor_open_position: -1.0472,
            motor_closed_position: 0.0,
            default_speed_rad_s: 5.0,
            default_torque_pu: 0.3,
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

    /// Convert gripper position [0,1] to motor position in radians.
    fn gripper_to_motor_position(&self, gripper_position: f64) -> f64 {
        // Linear interpolation: gripper [0,1] -> motor [closed, open]
        let gripper_range = self.gripper_open_position - self.gripper_closed_position;
        let motor_range = self.motor_open_position - self.motor_closed_position;
        let normalized = (gripper_position - self.gripper_closed_position) / gripper_range;
        self.motor_closed_position + normalized * motor_range
    }

    /// Initialize gripper motor.
    pub fn init_motor_device(
        &mut self,
        motor_type: MotorType,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: ControlMode,
    ) -> Result<()> {
        let motor = Motor::new(motor_type, send_can_id, recv_can_id, control_mode);
        let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone())));
        self.inner.add_motor_device(motor, device);
        Ok(())
    }

    /// Get the gripper motor.
    pub fn get_motor(&self) -> Result<Motor> {
        self.inner.get_motor(0)
    }

    /// Get the motors (for API compatibility).
    pub fn get_motors(&self) -> Vec<Motor> {
        self.inner.get_motors()
    }

    /// Enable the gripper motor.
    pub fn enable_all(&self) -> Result<()> {
        self.inner.enable_all()
    }

    /// Disable the gripper motor.
    pub fn disable_all(&self) -> Result<()> {
        self.inner.disable_all()
    }

    /// Set zero position.
    pub fn set_zero(&self) -> Result<()> {
        self.inner.set_zero_all()
    }

    /// Set zero for all (alias for set_zero).
    pub fn set_zero_all(&self) -> Result<()> {
        self.inner.set_zero_all()
    }

    /// Refresh state.
    pub fn refresh_all(&self) -> Result<()> {
        self.inner.refresh_all()
    }

    /// Query parameter.
    pub fn query_param_all(&self, rid: MotorVariable) -> Result<()> {
        self.inner.query_param_all(rid)
    }

    /// Set callback mode.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        self.inner.set_callback_mode_all(mode)
    }

    /// Receive all available frames.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        self.inner.recv_all(first_timeout_us)
    }

    /// Set default speed and torque limits.
    pub fn set_limit(&mut self, speed_rad_s: f64, torque_pu: f64) {
        self.default_speed_rad_s = speed_rad_s;
        self.default_torque_pu = torque_pu;
    }

    /// Open the gripper using MIT control.
    pub fn open(&self, kp: f64, kd: f64) -> Result<()> {
        let motor_pos = self.gripper_to_motor_position(self.gripper_open_position);
        let param = MITParam {
            kp,
            kd,
            q: motor_pos,
            dq: 0.0,
            tau: 0.0,
        };
        self.inner.mit_control_one(0, &param)
    }

    /// Close the gripper using MIT control.
    pub fn close(&self, kp: f64, kd: f64) -> Result<()> {
        let motor_pos = self.gripper_to_motor_position(self.gripper_closed_position);
        let param = MITParam {
            kp,
            kd,
            q: motor_pos,
            dq: 0.0,
            tau: 0.0,
        };
        self.inner.mit_control_one(0, &param)
    }

    /// Set gripper position using position-force control.
    ///
    /// Args:
    ///     position: Gripper position (0.0 = closed, 1.0 = open)
    ///     speed: Speed limit in rad/s (optional, uses default)
    ///     torque: Torque limit as per-unit [0,1] (optional, uses default)
    ///     raw_position: If true, position is raw motor position in radians
    pub fn set_position(
        &self,
        position: f64,
        speed: Option<f64>,
        torque: Option<f64>,
        raw_position: bool,
    ) -> Result<()> {
        let motor_pos = if raw_position {
            position
        } else {
            self.gripper_to_motor_position(position)
        };

        let speed_val = speed.unwrap_or(self.default_speed_rad_s);
        let torque_val = torque.unwrap_or(self.default_torque_pu);

        let param = PosForceParam {
            q: motor_pos,
            dq: speed_val,
            i: torque_val,
        };
        self.inner.posforce_control_one(0, &param)
    }

    /// Set gripper position using MIT control (legacy).
    pub fn set_position_mit(&self, position: f64, kp: f64, kd: f64) -> Result<()> {
        let motor_pos = self.gripper_to_motor_position(position);
        let param = MITParam {
            kp,
            kd,
            q: motor_pos,
            dq: 0.0,
            tau: 0.0,
        };
        self.inner.mit_control_one(0, &param)
    }

    /// Grasp with force control.
    ///
    /// Moves to grasp position with specified force.
    pub fn grasp(&self, torque_pu: Option<f64>, speed_rad_s: Option<f64>) -> Result<()> {
        let motor_pos = self.gripper_to_motor_position(self.gripper_grasp_position);
        let torque_val = torque_pu.unwrap_or(self.default_torque_pu);
        let speed_val = speed_rad_s.unwrap_or(self.default_speed_rad_s);

        let param = PosForceParam {
            q: motor_pos,
            dq: speed_val,
            i: torque_val,
        };
        self.inner.posforce_control_one(0, &param)
    }

    /// MIT control for the gripper motor.
    pub fn mit_control_one(&self, index: usize, param: &MITParam) -> Result<()> {
        self.inner.mit_control_one(index, param)
    }

    /// MIT control for all (single motor).
    pub fn mit_control_all(&self, params: &[MITParam]) -> Result<()> {
        self.inner.mit_control_all(params)
    }

    /// Position-velocity control.
    pub fn posvel_control_one(&self, index: usize, param: &PosVelParam) -> Result<()> {
        self.inner.posvel_control_one(index, param)
    }

    /// Position-velocity control for all.
    pub fn posvel_control_all(&self, params: &[PosVelParam]) -> Result<()> {
        self.inner.posvel_control_all(params)
    }

    /// Position-force control.
    pub fn posforce_control_one(&self, index: usize, param: &PosForceParam) -> Result<()> {
        self.inner.posforce_control_one(index, param)
    }

    /// Position-force control for all.
    pub fn posforce_control_all(&self, params: &[PosForceParam]) -> Result<()> {
        self.inner.posforce_control_all(params)
    }

    /// Set control mode.
    pub fn set_control_mode_one(&self, index: usize, mode: ControlMode) -> Result<()> {
        self.inner.set_control_mode_one(index, mode)
    }

    /// Set control mode for all.
    pub fn set_control_mode_all(&self, mode: ControlMode) -> Result<()> {
        self.inner.set_control_mode_all(mode)
    }

    /// Get motor count.
    pub fn motor_count(&self) -> usize {
        self.inner.motor_count()
    }
}
