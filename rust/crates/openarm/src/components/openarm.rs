//! Main OpenArm orchestrator class.

use std::sync::{Arc, Mutex};

use crate::canbus::{CANDeviceCollection, CANSocket, MotorDeviceCan};
use crate::damiao_motor::{CallbackMode, ControlMode, Motor, MotorType, MotorVariable};
use crate::error::{OpenArmError, Result};

use super::arm::ArmComponent;
use super::gripper::GripperComponent;

/// Main OpenArm orchestrator class.
///
/// Aggregates CANSocket, ArmComponent, GripperComponent, and manages
/// the master CANDeviceCollection for frame dispatch.
pub struct OpenArm {
    socket: Arc<Mutex<CANSocket>>,
    master_collection: Arc<CANDeviceCollection>,
    arm: ArmComponent,
    gripper: GripperComponent,
    enable_fd: bool,
}

impl OpenArm {
    /// Create a new OpenArm instance.
    pub fn new(can_interface: String, enable_fd: bool) -> Result<Self> {
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
    pub fn init_arm_motors(
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

            // Register with master collection
            self.master_collection.register_device_internal(Arc::clone(&device));

            // Add to arm component
            self.arm.add_motor_device(motor, device);
        }

        Ok(())
    }

    /// Initialize gripper motor.
    pub fn init_gripper_motor(
        &mut self,
        motor_type: MotorType,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: ControlMode,
    ) -> Result<()> {
        let motor = Motor::new(motor_type, send_can_id, recv_can_id, control_mode);
        let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone())));

        // Register with master collection
        self.master_collection.register_device_internal(Arc::clone(&device));

        // Add to gripper component
        self.gripper.add_motor_device(motor, device);

        Ok(())
    }

    /// Get the arm component reference.
    pub fn arm(&self) -> &ArmComponent {
        &self.arm
    }

    /// Get the arm component mutable reference.
    pub fn arm_mut(&mut self) -> &mut ArmComponent {
        &mut self.arm
    }

    /// Get the gripper component reference.
    pub fn gripper(&self) -> &GripperComponent {
        &self.gripper
    }

    /// Get the gripper component mutable reference.
    pub fn gripper_mut(&mut self) -> &mut GripperComponent {
        &mut self.gripper
    }

    /// Get the master CAN device collection.
    pub fn master_collection(&self) -> &Arc<CANDeviceCollection> {
        &self.master_collection
    }

    /// Get the socket.
    pub fn socket(&self) -> Arc<Mutex<CANSocket>> {
        Arc::clone(&self.socket)
    }

    /// Enable all motors (arm and gripper).
    pub fn enable_all(&self) -> Result<()> {
        self.arm.enable_all()?;
        self.gripper.enable_all()?;
        Ok(())
    }

    /// Disable all motors (arm and gripper).
    pub fn disable_all(&self) -> Result<()> {
        self.arm.disable_all()?;
        self.gripper.disable_all()?;
        Ok(())
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> Result<()> {
        self.arm.set_zero_all()?;
        self.gripper.set_zero_all()?;
        Ok(())
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> Result<()> {
        self.arm.refresh_all()?;
        self.gripper.refresh_all()?;
        Ok(())
    }

    /// Refresh state for one motor (arm only).
    pub fn refresh_one(&self, index: usize) -> Result<()> {
        self.arm.refresh_one(index)
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> Result<()> {
        self.arm.query_param_all(rid)?;
        self.gripper.query_param_all(rid)?;
        Ok(())
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        self.master_collection.set_callback_mode_all(mode);
    }

    /// Receive all available frames with timeout for first frame.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        self.master_collection.recv_all(first_timeout_us)
    }

    /// Check if CAN-FD is enabled.
    pub fn enable_fd(&self) -> bool {
        self.enable_fd
    }
}

/// Remote OpenArm orchestrator class for controlling motors over P2P.
///
/// This struct provides the same API as OpenArm but uses a remote CAN socket
/// via xoq P2P to communicate with motors on a remote machine.
#[cfg(feature = "remote")]
pub struct RemoteOpenArm {
    socket: Arc<Mutex<crate::canbus::AnyCANSocket>>,
    master_collection: Arc<crate::canbus::AnyCANDeviceCollection>,
    arm: super::arm::AnyArmComponent,
    gripper: super::gripper::AnyGripperComponent,
    enable_fd: bool,
}

#[cfg(feature = "remote")]
impl RemoteOpenArm {
    /// Create a new RemoteOpenArm instance connected to a remote CAN server.
    ///
    /// # Arguments
    /// * `server_id` - The xoq server endpoint ID to connect to
    /// * `enable_fd` - Whether to enable CAN FD support
    ///
    /// # Example
    /// ```no_run
    /// use openarm::RemoteOpenArm;
    ///
    /// let arm = RemoteOpenArm::new("abc123...", false)?;
    /// # Ok::<(), openarm::error::OpenArmError>(())
    /// ```
    pub fn new(server_id: &str, enable_fd: bool) -> Result<Self> {
        let remote_socket = xoq::socketcan::new(server_id)
            .enable_fd(enable_fd)
            .timeout(std::time::Duration::from_millis(100))
            .open()
            .map_err(|e| OpenArmError::SocketError(format!("Failed to connect to remote: {}", e)))?;

        let socket = crate::canbus::AnyCANSocket::Remote(remote_socket);
        let socket_arc = Arc::new(Mutex::new(socket));

        let master_collection = Arc::new(crate::canbus::AnyCANDeviceCollection::from_socket_arc(
            Arc::clone(&socket_arc),
        ));

        let arm = super::arm::AnyArmComponent::from_collection(Arc::clone(&master_collection));
        let gripper =
            super::gripper::AnyGripperComponent::from_collection(Arc::clone(&master_collection));

        Ok(Self {
            socket: socket_arc,
            master_collection,
            arm,
            gripper,
            enable_fd,
        })
    }

    /// Initialize arm motors.
    pub fn init_arm_motors(
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

            self.master_collection
                .register_device_internal(Arc::clone(&device));
            self.arm.add_motor_device(motor, device);
        }

        Ok(())
    }

    /// Initialize gripper motor.
    pub fn init_gripper_motor(
        &mut self,
        motor_type: MotorType,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: ControlMode,
    ) -> Result<()> {
        let motor = Motor::new(motor_type, send_can_id, recv_can_id, control_mode);
        let device = Arc::new(Mutex::new(MotorDeviceCan::new(motor.clone())));

        self.master_collection
            .register_device_internal(Arc::clone(&device));
        self.gripper.add_motor_device(motor, device);

        Ok(())
    }

    /// Get the arm component reference.
    pub fn arm(&self) -> &super::arm::AnyArmComponent {
        &self.arm
    }

    /// Get the arm component mutable reference.
    pub fn arm_mut(&mut self) -> &mut super::arm::AnyArmComponent {
        &mut self.arm
    }

    /// Get the gripper component reference.
    pub fn gripper(&self) -> &super::gripper::AnyGripperComponent {
        &self.gripper
    }

    /// Get the gripper component mutable reference.
    pub fn gripper_mut(&mut self) -> &mut super::gripper::AnyGripperComponent {
        &mut self.gripper
    }

    /// Get the master CAN device collection.
    pub fn master_collection(&self) -> &Arc<crate::canbus::AnyCANDeviceCollection> {
        &self.master_collection
    }

    /// Get the socket.
    pub fn socket(&self) -> Arc<Mutex<crate::canbus::AnyCANSocket>> {
        Arc::clone(&self.socket)
    }

    /// Enable all motors (arm and gripper).
    pub fn enable_all(&self) -> Result<()> {
        self.arm.enable_all()?;
        self.gripper.enable_all()?;
        Ok(())
    }

    /// Disable all motors (arm and gripper).
    pub fn disable_all(&self) -> Result<()> {
        self.arm.disable_all()?;
        self.gripper.disable_all()?;
        Ok(())
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> Result<()> {
        self.arm.set_zero_all()?;
        self.gripper.set_zero_all()?;
        Ok(())
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> Result<()> {
        self.arm.refresh_all()?;
        self.gripper.refresh_all()?;
        Ok(())
    }

    /// Refresh state for one motor (arm only).
    pub fn refresh_one(&self, index: usize) -> Result<()> {
        self.arm.refresh_one(index)
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: MotorVariable) -> Result<()> {
        self.arm.query_param_all(rid)?;
        self.gripper.query_param_all(rid)?;
        Ok(())
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: CallbackMode) {
        self.master_collection.set_callback_mode_all(mode);
    }

    /// Receive all available frames with timeout for first frame.
    pub fn recv_all(&self, first_timeout_us: u64) -> Result<usize> {
        self.master_collection.recv_all(first_timeout_us)
    }

    /// Check if CAN-FD is enabled.
    pub fn enable_fd(&self) -> bool {
        self.enable_fd
    }
}
