//! Device collection for managing multiple motors.

use std::sync::{Arc, Mutex};

use crate::canbus::{CANDeviceCollection, MotorDeviceCan};
use crate::error::{OpenArmError, Result};

use super::constants::*;
use super::control::CanPacketEncoder;
use super::motor::Motor;

/// Collection of Damiao motor devices.
pub struct DMDeviceCollection {
    motors: Vec<Motor>,
    devices: Vec<Arc<Mutex<MotorDeviceCan>>>,
    pub(crate) collection: Arc<CANDeviceCollection>,
}

impl DMDeviceCollection {
    /// Create from shared collection.
    pub fn from_collection(collection: Arc<CANDeviceCollection>) -> Self {
        Self {
            motors: Vec::new(),
            devices: Vec::new(),
            collection,
        }
    }

    /// Add a motor and its device.
    pub fn add_motor_device(&mut self, motor: Motor, device: Arc<Mutex<MotorDeviceCan>>) {
        self.motors.push(motor);
        self.collection.register_device_internal(Arc::clone(&device));
        self.devices.push(device);
    }

    /// Get collection reference.
    pub fn collection(&self) -> &Arc<CANDeviceCollection> {
        &self.collection
    }

    /// Get the motors.
    pub fn get_motors(&self) -> Vec<Motor> {
        self.motors.clone()
    }

    /// Get a specific motor by index.
    pub fn get_motor(&self, index: usize) -> Result<Motor> {
        self.motors
            .get(index)
            .cloned()
            .ok_or(OpenArmError::IndexOutOfRange(index))
    }

    /// Get the number of motors.
    pub fn motor_count(&self) -> usize {
        self.motors.len()
    }

    /// Enable all motors.
    pub fn enable_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_enable_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Disable all motors.
    pub fn disable_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_disable_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_set_zero_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_refresh_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Refresh state for one motor.
    pub fn refresh_one(&self, index: usize) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_refresh_command(motor);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_query_param_command(motor, rid);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Query parameter for one motor.
    pub fn query_param_one(&self, index: usize, rid: MotorVariable) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_query_param_command(motor, rid);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// MIT control for one motor.
    pub fn mit_control_one(&self, index: usize, param: &MITParam) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_mit_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// MIT control for all motors.
    pub fn mit_control_all(&self, params: &[MITParam]) -> Result<()> {
        if params.len() != self.motors.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: self.motors.len(),
                actual: params.len(),
            });
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_mit_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Position-velocity control for one motor.
    pub fn posvel_control_one(&self, index: usize, param: &PosVelParam) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_posvel_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Position-velocity control for all motors.
    pub fn posvel_control_all(&self, params: &[PosVelParam]) -> Result<()> {
        if params.len() != self.motors.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: self.motors.len(),
                actual: params.len(),
            });
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_posvel_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Position-force control for one motor.
    pub fn posforce_control_one(&self, index: usize, param: &PosForceParam) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_posforce_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Position-force control for all motors.
    pub fn posforce_control_all(&self, params: &[PosForceParam]) -> Result<()> {
        if params.len() != self.motors.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: self.motors.len(),
                actual: params.len(),
            });
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_posforce_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Set control mode for one motor.
    pub fn set_control_mode_one(&self, index: usize, mode: ControlMode) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_set_control_mode_command(motor, mode);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Set control mode for all motors.
    pub fn set_control_mode_all(&self, mode: ControlMode) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_set_control_mode_command(motor, mode);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        for device in &self.devices {
            device.lock().unwrap().set_callback_mode(mode);
        }
    }

    /// Receive all available frames.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        self.collection.recv_all(first_timeout_us)
    }
}

/// Collection of Damiao motor devices using AnyCANSocket (local or remote).
#[cfg(feature = "remote")]
pub struct AnyDMDeviceCollection {
    motors: Vec<Motor>,
    devices: Vec<Arc<Mutex<MotorDeviceCan>>>,
    pub(crate) collection: Arc<crate::canbus::AnyCANDeviceCollection>,
}

#[cfg(feature = "remote")]
impl AnyDMDeviceCollection {
    /// Create from shared collection.
    pub fn from_collection(collection: Arc<crate::canbus::AnyCANDeviceCollection>) -> Self {
        Self {
            motors: Vec::new(),
            devices: Vec::new(),
            collection,
        }
    }

    /// Add a motor and its device.
    pub fn add_motor_device(&mut self, motor: Motor, device: Arc<Mutex<MotorDeviceCan>>) {
        self.motors.push(motor);
        self.collection.register_device_internal(Arc::clone(&device));
        self.devices.push(device);
    }

    /// Get collection reference.
    pub fn collection(&self) -> &Arc<crate::canbus::AnyCANDeviceCollection> {
        &self.collection
    }

    /// Get the motors.
    pub fn get_motors(&self) -> Vec<Motor> {
        self.motors.clone()
    }

    /// Get a specific motor by index.
    pub fn get_motor(&self, index: usize) -> Result<Motor> {
        self.motors
            .get(index)
            .cloned()
            .ok_or(OpenArmError::IndexOutOfRange(index))
    }

    /// Get the number of motors.
    pub fn motor_count(&self) -> usize {
        self.motors.len()
    }

    /// Enable all motors.
    pub fn enable_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_enable_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Disable all motors.
    pub fn disable_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_disable_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_set_zero_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_refresh_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Refresh state for one motor.
    pub fn refresh_one(&self, index: usize) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_refresh_command(motor);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_query_param_command(motor, rid);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Query parameter for one motor.
    pub fn query_param_one(&self, index: usize, rid: MotorVariable) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_query_param_command(motor, rid);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// MIT control for one motor.
    pub fn mit_control_one(&self, index: usize, param: &MITParam) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_mit_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// MIT control for all motors.
    pub fn mit_control_all(&self, params: &[MITParam]) -> Result<()> {
        if params.len() != self.motors.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: self.motors.len(),
                actual: params.len(),
            });
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_mit_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Position-velocity control for one motor.
    pub fn posvel_control_one(&self, index: usize, param: &PosVelParam) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_posvel_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Position-velocity control for all motors.
    pub fn posvel_control_all(&self, params: &[PosVelParam]) -> Result<()> {
        if params.len() != self.motors.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: self.motors.len(),
                actual: params.len(),
            });
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_posvel_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Position-force control for one motor.
    pub fn posforce_control_one(&self, index: usize, param: &PosForceParam) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_posforce_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Position-force control for all motors.
    pub fn posforce_control_all(&self, params: &[PosForceParam]) -> Result<()> {
        if params.len() != self.motors.len() {
            return Err(OpenArmError::ParamCountMismatch {
                expected: self.motors.len(),
                actual: params.len(),
            });
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_posforce_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Set control mode for one motor.
    pub fn set_control_mode_one(&self, index: usize, mode: ControlMode) -> Result<()> {
        let motor = self
            .motors
            .get(index)
            .ok_or(OpenArmError::IndexOutOfRange(index))?;
        let packet = CanPacketEncoder::create_set_control_mode_command(motor, mode);
        self.collection.send_packet(packet.send_can_id, &packet.data)?;
        Ok(())
    }

    /// Set control mode for all motors.
    pub fn set_control_mode_all(&self, mode: ControlMode) -> Result<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_set_control_mode_command(motor, mode);
            self.collection.send_packet(packet.send_can_id, &packet.data)?;
        }
        Ok(())
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        for device in &self.devices {
            device.lock().unwrap().set_callback_mode(mode);
        }
    }

    /// Receive all available frames.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        self.collection.recv_all(first_timeout_us)
    }
}
