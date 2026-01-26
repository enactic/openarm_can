//! Main OpenArm orchestrator class.

use pyo3::prelude::*;
use std::sync::{Arc, Mutex};

use crate::canbus::{CANDeviceCollection, CANSocket, MotorDeviceCan};
use crate::damiao_motor::{CallbackMode, ControlMode, Motor, MotorType, MotorVariable};

use super::arm::ArmComponent;
use super::gripper::GripperComponent;

/// Main OpenArm orchestrator class.
///
/// Aggregates CANSocket, ArmComponent, GripperComponent, and manages
/// the master CANDeviceCollection for frame dispatch.
#[pyclass]
pub struct OpenArm {
    socket: Arc<Mutex<CANSocket>>,
    master_collection: Arc<CANDeviceCollection>,
    arm: ArmComponent,
    gripper: GripperComponent,
    enable_fd: bool,
}

#[pymethods]
impl OpenArm {
    #[new]
    #[pyo3(signature = (can_interface, enable_fd=false))]
    pub fn new(can_interface: String, enable_fd: bool) -> PyResult<Self> {
        let socket = CANSocket::new(can_interface, enable_fd, 100)?;
        let socket_arc = Arc::new(Mutex::new(socket));

        // Create master collection that will be shared
        let master_collection = Arc::new(CANDeviceCollection::from_socket_arc(Arc::clone(&socket_arc)));

        // Create components that share the master collection
        let arm = ArmComponent::from_collection(Arc::clone(&master_collection));
        let gripper = GripperComponent::from_collection(Arc::clone(&master_collection));

        Ok(Self {
            socket: socket_arc,
            master_collection,
            arm,
            gripper,
            enable_fd,
        })
    }

    /// Initialize arm motors.
    #[pyo3(signature = (motor_types, send_can_ids, recv_can_ids, control_modes=None))]
    pub fn init_arm_motors(
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

            // Register with master collection
            self.master_collection.register_device_internal(Arc::clone(&device));

            // Add to arm component
            self.arm.add_motor_device(motor, device);
        }

        Ok(())
    }

    /// Initialize gripper motor.
    #[pyo3(signature = (motor_type, send_can_id, recv_can_id, control_mode=ControlMode::MIT))]
    pub fn init_gripper_motor(
        &mut self,
        motor_type: MotorType,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: ControlMode,
    ) -> PyResult<()> {
        let motor = Motor::new(motor_type, send_can_id, recv_can_id, control_mode);
        let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone()).0));

        // Register with master collection
        self.master_collection.register_device_internal(Arc::clone(&device));

        // Add to gripper component
        self.gripper.add_motor_device(motor, device);

        Ok(())
    }

    /// Get the arm component.
    pub fn get_arm(&self) -> ArmComponent {
        // Return a new ArmComponent that shares the same collection
        // This is needed because PyO3 doesn't support returning references
        let mut arm = ArmComponent::from_collection(Arc::clone(&self.master_collection));
        // Copy motors from the original arm
        for motor in self.arm.get_motors() {
            let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone()).0));
            arm.add_motor_device(motor, device);
        }
        arm
    }

    /// Get the gripper component.
    pub fn get_gripper(&self) -> GripperComponent {
        // Return a new GripperComponent that shares the same collection
        let mut gripper = GripperComponent::from_collection(Arc::clone(&self.master_collection));
        for motor in self.gripper.get_motors() {
            let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone()).0));
            gripper.add_motor_device(motor, device);
        }
        gripper
    }

    /// Get the master CAN device collection.
    pub fn get_master_can_device_collection(&self) -> CANDeviceCollection {
        CANDeviceCollection::from_socket_arc(Arc::clone(&self.socket))
    }

    /// Enable all motors (arm and gripper).
    pub fn enable_all(&self) -> PyResult<()> {
        self.arm.enable_all()?;
        self.gripper.enable_all()?;
        Ok(())
    }

    /// Disable all motors (arm and gripper).
    pub fn disable_all(&self) -> PyResult<()> {
        self.arm.disable_all()?;
        self.gripper.disable_all()?;
        Ok(())
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> PyResult<()> {
        self.arm.set_zero_all()?;
        self.gripper.set_zero_all()?;
        Ok(())
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> PyResult<()> {
        self.arm.refresh_all()?;
        self.gripper.refresh_all()?;
        Ok(())
    }

    /// Refresh state for one motor (arm only).
    pub fn refresh_one(&self, index: usize) -> PyResult<()> {
        self.arm.refresh_one(index)
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> PyResult<()> {
        self.arm.query_param_all(rid)?;
        self.gripper.query_param_all(rid)?;
        Ok(())
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        self.master_collection.set_callback_mode_all(mode);
    }

    /// Receive all available frames with timeout for first frame.
    #[pyo3(signature = (first_timeout_us=500))]
    pub fn recv_all(&self, first_timeout_us: u64) -> PyResult<usize> {
        self.master_collection.recv_all(first_timeout_us)
    }

    /// Check if CAN-FD is enabled.
    #[getter]
    pub fn get_enable_fd(&self) -> bool {
        self.enable_fd
    }

    fn __repr__(&self) -> String {
        format!(
            "OpenArm(arm_motors={}, gripper_motors={}, enable_fd={})",
            self.arm.motor_count(),
            self.gripper.get_motors().len(),
            self.enable_fd
        )
    }
}
