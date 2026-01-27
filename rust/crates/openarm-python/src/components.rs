//! Python wrappers for component types.

use pyo3::prelude::*;
use std::sync::{Arc, Mutex};

use crate::damiao_motor::{
    PyCallbackMode, PyControlMode, PyMITParam, PyMotor, PyMotorType, PyMotorVariable,
    PyPosForceParam, PyPosVelParam,
};
use crate::error::IntoPyResult;

/// OpenArm orchestrator wrapper.
#[pyclass(name = "OpenArm")]
pub struct PyOpenArm {
    inner: openarm::OpenArm,
}

#[pymethods]
impl PyOpenArm {
    #[new]
    #[pyo3(signature = (can_interface, enable_fd=false))]
    pub fn new(can_interface: String, enable_fd: bool) -> PyResult<Self> {
        let inner = openarm::OpenArm::new(can_interface, enable_fd).into_py_result()?;
        Ok(Self { inner })
    }

    /// Initialize arm motors.
    #[pyo3(signature = (motor_types, send_can_ids, recv_can_ids, control_modes=None))]
    pub fn init_arm_motors(
        &mut self,
        motor_types: Vec<PyMotorType>,
        send_can_ids: Vec<u32>,
        recv_can_ids: Vec<u32>,
        control_modes: Option<Vec<PyControlMode>>,
    ) -> PyResult<()> {
        let types: Vec<openarm::MotorType> = motor_types.into_iter().map(|t| t.into()).collect();
        let modes: Option<Vec<openarm::ControlMode>> =
            control_modes.map(|m| m.into_iter().map(|c| c.into()).collect());
        self.inner
            .init_arm_motors(&types, &send_can_ids, &recv_can_ids, modes.as_deref())
            .into_py_result()
    }

    /// Initialize gripper motor.
    #[pyo3(signature = (motor_type, send_can_id, recv_can_id, control_mode=PyControlMode::MIT))]
    pub fn init_gripper_motor(
        &mut self,
        motor_type: PyMotorType,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: PyControlMode,
    ) -> PyResult<()> {
        self.inner
            .init_gripper_motor(motor_type.into(), send_can_id, recv_can_id, control_mode.into())
            .into_py_result()
    }

    /// Get the arm component.
    pub fn get_arm(&self) -> PyArmComponent {
        PyArmComponent::from_ref(self.inner.arm())
    }

    /// Get the gripper component.
    pub fn get_gripper(&self) -> PyGripperComponent {
        PyGripperComponent::from_ref(self.inner.gripper())
    }

    /// Enable all motors (arm and gripper).
    pub fn enable_all(&self) -> PyResult<()> {
        self.inner.enable_all().into_py_result()
    }

    /// Disable all motors (arm and gripper).
    pub fn disable_all(&self) -> PyResult<()> {
        self.inner.disable_all().into_py_result()
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> PyResult<()> {
        self.inner.set_zero_all().into_py_result()
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> PyResult<()> {
        self.inner.refresh_all().into_py_result()
    }

    /// Refresh state for one motor (arm only).
    pub fn refresh_one(&self, index: usize) -> PyResult<()> {
        self.inner.refresh_one(index).into_py_result()
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: PyMotorVariable) -> PyResult<()> {
        self.inner.query_param_all(rid.into()).into_py_result()
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: PyCallbackMode) {
        self.inner.set_callback_mode_all(mode.into());
    }

    /// Receive all available frames with timeout for first frame.
    #[pyo3(signature = (first_timeout_us=500))]
    pub fn recv_all(&self, first_timeout_us: u64) -> PyResult<usize> {
        self.inner.recv_all(first_timeout_us).into_py_result()
    }

    /// Check if CAN-FD is enabled.
    #[getter]
    pub fn get_enable_fd(&self) -> bool {
        self.inner.enable_fd()
    }

    fn __repr__(&self) -> String {
        format!(
            "OpenArm(arm_motors={}, gripper_motors={}, enable_fd={})",
            self.inner.arm().motor_count(),
            self.inner.gripper().motor_count(),
            self.inner.enable_fd()
        )
    }
}

/// Arm component wrapper.
#[pyclass(name = "ArmComponent")]
pub struct PyArmComponent {
    // Store the collection and create a fresh ArmComponent for each operation
    // This is necessary because we can't hold a reference to the inner component
    collection: Arc<openarm::CANDeviceCollection>,
    motors: Vec<openarm::Motor>,
}

impl PyArmComponent {
    pub fn from_ref(arm: &openarm::ArmComponent) -> Self {
        Self {
            collection: Arc::clone(arm.collection()),
            motors: arm.get_motors(),
        }
    }

    fn make_inner(&self) -> openarm::ArmComponent {
        let mut arm = openarm::ArmComponent::from_collection(Arc::clone(&self.collection));
        for motor in &self.motors {
            let device = Arc::new(Mutex::new(openarm::MotorDeviceCan::new(motor.clone())));
            arm.add_motor_device(motor.clone(), device);
        }
        arm
    }
}

#[pymethods]
impl PyArmComponent {
    /// Get the motors.
    pub fn get_motors(&self) -> Vec<PyMotor> {
        self.motors.iter().cloned().map(PyMotor::from).collect()
    }

    /// Get a specific motor by index.
    pub fn get_motor(&self, index: usize) -> PyResult<PyMotor> {
        self.motors
            .get(index)
            .cloned()
            .map(PyMotor::from)
            .ok_or_else(|| pyo3::exceptions::PyIndexError::new_err(format!("Index {} out of range", index)))
    }

    /// Get the number of motors.
    pub fn motor_count(&self) -> usize {
        self.motors.len()
    }

    /// Enable all motors.
    pub fn enable_all(&self) -> PyResult<()> {
        self.make_inner().enable_all().into_py_result()
    }

    /// Disable all motors.
    pub fn disable_all(&self) -> PyResult<()> {
        self.make_inner().disable_all().into_py_result()
    }

    /// Set zero position for all motors.
    pub fn set_zero_all(&self) -> PyResult<()> {
        self.make_inner().set_zero_all().into_py_result()
    }

    /// Refresh state for all motors.
    pub fn refresh_all(&self) -> PyResult<()> {
        self.make_inner().refresh_all().into_py_result()
    }

    /// Refresh state for one motor.
    pub fn refresh_one(&self, index: usize) -> PyResult<()> {
        self.make_inner().refresh_one(index).into_py_result()
    }

    /// Query parameter for all motors.
    pub fn query_param_all(&self, rid: PyMotorVariable) -> PyResult<()> {
        self.make_inner().query_param_all(rid.into()).into_py_result()
    }

    /// Query parameter for one motor.
    pub fn query_param_one(&self, index: usize, rid: PyMotorVariable) -> PyResult<()> {
        self.make_inner().query_param_one(index, rid.into()).into_py_result()
    }

    /// MIT control for one motor.
    pub fn mit_control_one(&self, index: usize, param: &PyMITParam) -> PyResult<()> {
        self.make_inner().mit_control_one(index, &param.into()).into_py_result()
    }

    /// MIT control for all motors.
    pub fn mit_control_all(&self, params: Vec<PyMITParam>) -> PyResult<()> {
        let core_params: Vec<openarm::MITParam> = params.iter().map(|p| p.into()).collect();
        self.make_inner().mit_control_all(&core_params).into_py_result()
    }

    /// Position-velocity control for one motor.
    pub fn posvel_control_one(&self, index: usize, param: &PyPosVelParam) -> PyResult<()> {
        self.make_inner().posvel_control_one(index, &param.into()).into_py_result()
    }

    /// Position-velocity control for all motors.
    pub fn posvel_control_all(&self, params: Vec<PyPosVelParam>) -> PyResult<()> {
        let core_params: Vec<openarm::PosVelParam> = params.iter().map(|p| p.into()).collect();
        self.make_inner().posvel_control_all(&core_params).into_py_result()
    }

    /// Position-force control for one motor.
    pub fn posforce_control_one(&self, index: usize, param: &PyPosForceParam) -> PyResult<()> {
        self.make_inner().posforce_control_one(index, &param.into()).into_py_result()
    }

    /// Position-force control for all motors.
    pub fn posforce_control_all(&self, params: Vec<PyPosForceParam>) -> PyResult<()> {
        let core_params: Vec<openarm::PosForceParam> = params.iter().map(|p| p.into()).collect();
        self.make_inner().posforce_control_all(&core_params).into_py_result()
    }

    /// Set control mode for one motor.
    pub fn set_control_mode_one(&self, index: usize, mode: PyControlMode) -> PyResult<()> {
        self.make_inner().set_control_mode_one(index, mode.into()).into_py_result()
    }

    /// Set control mode for all motors.
    pub fn set_control_mode_all(&self, mode: PyControlMode) -> PyResult<()> {
        self.make_inner().set_control_mode_all(mode.into()).into_py_result()
    }

    /// Set callback mode for all devices.
    pub fn set_callback_mode_all(&self, mode: PyCallbackMode) {
        self.make_inner().set_callback_mode_all(mode.into());
    }

    /// Receive all available frames.
    #[pyo3(signature = (first_timeout_us=500))]
    pub fn recv_all(&self, first_timeout_us: u64) -> PyResult<usize> {
        self.make_inner().recv_all(first_timeout_us).into_py_result()
    }

    fn __repr__(&self) -> String {
        format!("ArmComponent(motors={})", self.motors.len())
    }
}

/// Gripper component wrapper.
#[pyclass(name = "GripperComponent")]
pub struct PyGripperComponent {
    collection: Arc<openarm::CANDeviceCollection>,
    motors: Vec<openarm::Motor>,
    default_speed_rad_s: f64,
    default_torque_pu: f64,
}

impl PyGripperComponent {
    pub fn from_ref(gripper: &openarm::GripperComponent) -> Self {
        Self {
            collection: Arc::clone(gripper.collection()),
            motors: gripper.get_motors(),
            default_speed_rad_s: 5.0,
            default_torque_pu: 0.3,
        }
    }

    fn make_inner(&self) -> openarm::GripperComponent {
        let mut gripper = openarm::GripperComponent::from_collection(Arc::clone(&self.collection));
        for motor in &self.motors {
            let device = Arc::new(Mutex::new(openarm::MotorDeviceCan::new(motor.clone())));
            gripper.add_motor_device(motor.clone(), device);
        }
        gripper
    }
}

#[pymethods]
impl PyGripperComponent {
    /// Get the gripper motor.
    pub fn get_motor(&self) -> PyResult<PyMotor> {
        self.motors
            .first()
            .cloned()
            .map(PyMotor::from)
            .ok_or_else(|| pyo3::exceptions::PyIndexError::new_err("No gripper motor configured"))
    }

    /// Get the motors (for API compatibility).
    pub fn get_motors(&self) -> Vec<PyMotor> {
        self.motors.iter().cloned().map(PyMotor::from).collect()
    }

    /// Enable the gripper motor.
    pub fn enable_all(&self) -> PyResult<()> {
        self.make_inner().enable_all().into_py_result()
    }

    /// Disable the gripper motor.
    pub fn disable_all(&self) -> PyResult<()> {
        self.make_inner().disable_all().into_py_result()
    }

    /// Set zero position.
    pub fn set_zero(&self) -> PyResult<()> {
        self.make_inner().set_zero().into_py_result()
    }

    /// Set zero for all (alias for set_zero).
    pub fn set_zero_all(&self) -> PyResult<()> {
        self.make_inner().set_zero_all().into_py_result()
    }

    /// Refresh state.
    pub fn refresh_all(&self) -> PyResult<()> {
        self.make_inner().refresh_all().into_py_result()
    }

    /// Query parameter.
    pub fn query_param_all(&self, rid: PyMotorVariable) -> PyResult<()> {
        self.make_inner().query_param_all(rid.into()).into_py_result()
    }

    /// Set callback mode.
    pub fn set_callback_mode_all(&self, mode: PyCallbackMode) {
        self.make_inner().set_callback_mode_all(mode.into());
    }

    /// Receive all available frames.
    #[pyo3(signature = (first_timeout_us=500))]
    pub fn recv_all(&self, first_timeout_us: u64) -> PyResult<usize> {
        self.make_inner().recv_all(first_timeout_us).into_py_result()
    }

    /// Set default speed and torque limits.
    pub fn set_limit(&mut self, speed_rad_s: f64, torque_pu: f64) {
        self.default_speed_rad_s = speed_rad_s;
        self.default_torque_pu = torque_pu;
    }

    /// Open the gripper using MIT control.
    #[pyo3(signature = (kp=2.0, kd=0.5))]
    pub fn open(&self, kp: f64, kd: f64) -> PyResult<()> {
        self.make_inner().open(kp, kd).into_py_result()
    }

    /// Close the gripper using MIT control.
    #[pyo3(signature = (kp=2.0, kd=0.5))]
    pub fn close(&self, kp: f64, kd: f64) -> PyResult<()> {
        self.make_inner().close(kp, kd).into_py_result()
    }

    /// Set gripper position using position-force control.
    #[pyo3(signature = (position, speed=None, torque=None, raw_position=false))]
    pub fn set_position(
        &self,
        position: f64,
        speed: Option<f64>,
        torque: Option<f64>,
        raw_position: bool,
    ) -> PyResult<()> {
        let speed = speed.or(Some(self.default_speed_rad_s));
        let torque = torque.or(Some(self.default_torque_pu));
        self.make_inner().set_position(position, speed, torque, raw_position).into_py_result()
    }

    /// Set gripper position using MIT control (legacy).
    #[pyo3(signature = (position, kp=2.0, kd=0.5))]
    pub fn set_position_mit(&self, position: f64, kp: f64, kd: f64) -> PyResult<()> {
        self.make_inner().set_position_mit(position, kp, kd).into_py_result()
    }

    /// Grasp with force control.
    #[pyo3(signature = (torque_pu=None, speed_rad_s=None))]
    pub fn grasp(&self, torque_pu: Option<f64>, speed_rad_s: Option<f64>) -> PyResult<()> {
        let torque = torque_pu.or(Some(self.default_torque_pu));
        let speed = speed_rad_s.or(Some(self.default_speed_rad_s));
        self.make_inner().grasp(torque, speed).into_py_result()
    }

    /// MIT control for the gripper motor.
    pub fn mit_control_one(&self, index: usize, param: &PyMITParam) -> PyResult<()> {
        self.make_inner().mit_control_one(index, &param.into()).into_py_result()
    }

    /// MIT control for all (single motor).
    pub fn mit_control_all(&self, params: Vec<PyMITParam>) -> PyResult<()> {
        let core_params: Vec<openarm::MITParam> = params.iter().map(|p| p.into()).collect();
        self.make_inner().mit_control_all(&core_params).into_py_result()
    }

    /// Position-velocity control.
    pub fn posvel_control_one(&self, index: usize, param: &PyPosVelParam) -> PyResult<()> {
        self.make_inner().posvel_control_one(index, &param.into()).into_py_result()
    }

    /// Position-velocity control for all.
    pub fn posvel_control_all(&self, params: Vec<PyPosVelParam>) -> PyResult<()> {
        let core_params: Vec<openarm::PosVelParam> = params.iter().map(|p| p.into()).collect();
        self.make_inner().posvel_control_all(&core_params).into_py_result()
    }

    /// Position-force control.
    pub fn posforce_control_one(&self, index: usize, param: &PyPosForceParam) -> PyResult<()> {
        self.make_inner().posforce_control_one(index, &param.into()).into_py_result()
    }

    /// Position-force control for all.
    pub fn posforce_control_all(&self, params: Vec<PyPosForceParam>) -> PyResult<()> {
        let core_params: Vec<openarm::PosForceParam> = params.iter().map(|p| p.into()).collect();
        self.make_inner().posforce_control_all(&core_params).into_py_result()
    }

    /// Set control mode.
    pub fn set_control_mode_one(&self, index: usize, mode: PyControlMode) -> PyResult<()> {
        self.make_inner().set_control_mode_one(index, mode.into()).into_py_result()
    }

    /// Set control mode for all.
    pub fn set_control_mode_all(&self, mode: PyControlMode) -> PyResult<()> {
        self.make_inner().set_control_mode_all(mode.into()).into_py_result()
    }

    fn __repr__(&self) -> String {
        format!("GripperComponent(motors={})", self.motors.len())
    }
}
