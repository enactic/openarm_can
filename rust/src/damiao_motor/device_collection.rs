//! Device collection for managing multiple motors.

use pyo3::prelude::*;
use std::sync::{Arc, Mutex};

use crate::canbus::{CANDeviceCollection, MotorDeviceCan};

use super::constants::*;
use super::control::CanPacketEncoder;
use super::motor::Motor;

/// Collection of Damiao motor devices.
#[pyclass(subclass)]
pub struct DMDeviceCollection {
    motors: Vec<Motor>,
    devices: Vec<Arc<Mutex<MotorDeviceCan>>>,
    pub(crate) collection: Arc<CANDeviceCollection>,
}

#[pymethods]
impl DMDeviceCollection {
    /// Get the motors.
    pub fn get_motors(&self) -> Vec<Motor> {
        self.motors.clone()
    }

    /// Get a specific motor by index.
    pub fn get_motor(&self, index: usize) -> PyResult<Motor> {
        self.motors.get(index).cloned().ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!("Motor index {} out of range", index))
        })
    }

    /// Get the number of motors.
    pub fn motor_count(&self) -> usize {
        self.motors.len()
    }

    /// Enable all motors.
    pub fn enable_all(&self) -> PyResult<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_enable_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Disable all motors.
    pub fn disable_all(&self) -> PyResult<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_disable_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> PyResult<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_set_zero_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> PyResult<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_refresh_command(motor);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Refresh state for one motor.
    pub fn refresh_one(&self, index: usize) -> PyResult<()> {
        let motor = self.motors.get(index).ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!("Motor index {} out of range", index))
        })?;
        let packet = CanPacketEncoder::create_refresh_command(motor);
        self.collection.send_packet(packet.send_can_id, &packet.data)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        Ok(())
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> PyResult<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_query_param_command(motor, rid);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Query parameter for one motor.
    pub fn query_param_one(&self, index: usize, rid: MotorVariable) -> PyResult<()> {
        let motor = self.motors.get(index).ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!("Motor index {} out of range", index))
        })?;
        let packet = CanPacketEncoder::create_query_param_command(motor, rid);
        self.collection.send_packet(packet.send_can_id, &packet.data)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        Ok(())
    }

    /// MIT control for one motor.
    pub fn mit_control_one(&self, index: usize, param: &MITParam) -> PyResult<()> {
        let motor = self.motors.get(index).ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!("Motor index {} out of range", index))
        })?;
        let packet = CanPacketEncoder::create_mit_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        Ok(())
    }

    /// MIT control for all motors.
    pub fn mit_control_all(&self, params: Vec<MITParam>) -> PyResult<()> {
        if params.len() != self.motors.len() {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Expected {} params, got {}",
                self.motors.len(),
                params.len()
            )));
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_mit_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Position-velocity control for one motor.
    pub fn posvel_control_one(&self, index: usize, param: &PosVelParam) -> PyResult<()> {
        let motor = self.motors.get(index).ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!("Motor index {} out of range", index))
        })?;
        let packet = CanPacketEncoder::create_posvel_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        Ok(())
    }

    /// Position-velocity control for all motors.
    pub fn posvel_control_all(&self, params: Vec<PosVelParam>) -> PyResult<()> {
        if params.len() != self.motors.len() {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Expected {} params, got {}",
                self.motors.len(),
                params.len()
            )));
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_posvel_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Position-force control for one motor.
    pub fn posforce_control_one(&self, index: usize, param: &PosForceParam) -> PyResult<()> {
        let motor = self.motors.get(index).ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!("Motor index {} out of range", index))
        })?;
        let packet = CanPacketEncoder::create_posforce_control_command(motor, param);
        self.collection.send_packet(packet.send_can_id, &packet.data)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        Ok(())
    }

    /// Position-force control for all motors.
    pub fn posforce_control_all(&self, params: Vec<PosForceParam>) -> PyResult<()> {
        if params.len() != self.motors.len() {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Expected {} params, got {}",
                self.motors.len(),
                params.len()
            )));
        }
        for (motor, param) in self.motors.iter().zip(params.iter()) {
            let packet = CanPacketEncoder::create_posforce_control_command(motor, param);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        }
        Ok(())
    }

    /// Set control mode for one motor.
    pub fn set_control_mode_one(&self, index: usize, mode: ControlMode) -> PyResult<()> {
        let motor = self.motors.get(index).ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!("Motor index {} out of range", index))
        })?;
        let packet = CanPacketEncoder::create_set_control_mode_command(motor, mode);
        self.collection.send_packet(packet.send_can_id, &packet.data)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
        Ok(())
    }

    /// Set control mode for all motors.
    pub fn set_control_mode_all(&self, mode: ControlMode) -> PyResult<()> {
        for motor in &self.motors {
            let packet = CanPacketEncoder::create_set_control_mode_command(motor, mode);
            self.collection.send_packet(packet.send_can_id, &packet.data)
                .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("Send error: {}", e)))?;
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
    #[pyo3(signature = (first_timeout_us=500))]
    pub fn recv_all(&self, first_timeout_us: u64) -> PyResult<usize> {
        self.collection.recv_all(first_timeout_us)
    }

    fn __repr__(&self) -> String {
        format!("DMDeviceCollection(motors={})", self.motors.len())
    }
}

impl DMDeviceCollection {
    /// Add a motor and its device (internal).
    pub(crate) fn add_motor_device(&mut self, motor: Motor, device: Arc<Mutex<MotorDeviceCan>>) {
        self.motors.push(motor);
        self.collection.register_device_internal(Arc::clone(&device));
        self.devices.push(device);
    }

    /// Create from shared socket (internal).
    pub(crate) fn from_collection(collection: Arc<CANDeviceCollection>) -> Self {
        Self {
            motors: Vec::new(),
            devices: Vec::new(),
            collection,
        }
    }

    /// Get collection reference (internal).
    pub(crate) fn collection(&self) -> &Arc<CANDeviceCollection> {
        &self.collection
    }
}
