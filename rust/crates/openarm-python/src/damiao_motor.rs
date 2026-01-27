//! Python wrappers for damiao_motor types.

use pyo3::prelude::*;


/// Motor types supported by the Damiao motor family.
#[pyclass(name = "MotorType", eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyMotorType {
    DM3507 = 0,
    DM4310 = 1,
    DM4310_48V = 2,
    DM4340 = 3,
    DM4340_48V = 4,
    DM6006 = 5,
    DM8006 = 6,
    DM8009 = 7,
    DM10010L = 8,
    DM10010 = 9,
    DMH3510 = 10,
    DMH6215 = 11,
    DMG6220 = 12,
}

impl From<PyMotorType> for openarm::MotorType {
    fn from(t: PyMotorType) -> Self {
        match t {
            PyMotorType::DM3507 => openarm::MotorType::DM3507,
            PyMotorType::DM4310 => openarm::MotorType::DM4310,
            PyMotorType::DM4310_48V => openarm::MotorType::DM4310_48V,
            PyMotorType::DM4340 => openarm::MotorType::DM4340,
            PyMotorType::DM4340_48V => openarm::MotorType::DM4340_48V,
            PyMotorType::DM6006 => openarm::MotorType::DM6006,
            PyMotorType::DM8006 => openarm::MotorType::DM8006,
            PyMotorType::DM8009 => openarm::MotorType::DM8009,
            PyMotorType::DM10010L => openarm::MotorType::DM10010L,
            PyMotorType::DM10010 => openarm::MotorType::DM10010,
            PyMotorType::DMH3510 => openarm::MotorType::DMH3510,
            PyMotorType::DMH6215 => openarm::MotorType::DMH6215,
            PyMotorType::DMG6220 => openarm::MotorType::DMG6220,
        }
    }
}

impl From<openarm::MotorType> for PyMotorType {
    fn from(t: openarm::MotorType) -> Self {
        match t {
            openarm::MotorType::DM3507 => PyMotorType::DM3507,
            openarm::MotorType::DM4310 => PyMotorType::DM4310,
            openarm::MotorType::DM4310_48V => PyMotorType::DM4310_48V,
            openarm::MotorType::DM4340 => PyMotorType::DM4340,
            openarm::MotorType::DM4340_48V => PyMotorType::DM4340_48V,
            openarm::MotorType::DM6006 => PyMotorType::DM6006,
            openarm::MotorType::DM8006 => PyMotorType::DM8006,
            openarm::MotorType::DM8009 => PyMotorType::DM8009,
            openarm::MotorType::DM10010L => PyMotorType::DM10010L,
            openarm::MotorType::DM10010 => PyMotorType::DM10010,
            openarm::MotorType::DMH3510 => PyMotorType::DMH3510,
            openarm::MotorType::DMH6215 => PyMotorType::DMH6215,
            openarm::MotorType::DMG6220 => PyMotorType::DMG6220,
        }
    }
}

/// Control modes for motor operation.
#[pyclass(name = "ControlMode", eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[allow(non_camel_case_types)]
pub enum PyControlMode {
    #[default]
    MIT = 1,
    POS_VEL = 2,
    VEL = 3,
    POS_FORCE = 4,
}

impl From<PyControlMode> for openarm::ControlMode {
    fn from(m: PyControlMode) -> Self {
        match m {
            PyControlMode::MIT => openarm::ControlMode::MIT,
            PyControlMode::POS_VEL => openarm::ControlMode::POS_VEL,
            PyControlMode::VEL => openarm::ControlMode::VEL,
            PyControlMode::POS_FORCE => openarm::ControlMode::POS_FORCE,
        }
    }
}

impl From<openarm::ControlMode> for PyControlMode {
    fn from(m: openarm::ControlMode) -> Self {
        match m {
            openarm::ControlMode::MIT => PyControlMode::MIT,
            openarm::ControlMode::POS_VEL => PyControlMode::POS_VEL,
            openarm::ControlMode::VEL => PyControlMode::VEL,
            openarm::ControlMode::POS_FORCE => PyControlMode::POS_FORCE,
        }
    }
}

/// Callback modes for frame processing.
#[pyclass(name = "CallbackMode", eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PyCallbackMode {
    #[default]
    STATE = 0,
    PARAM = 1,
    IGNORE = 2,
}

impl From<PyCallbackMode> for openarm::CallbackMode {
    fn from(m: PyCallbackMode) -> Self {
        match m {
            PyCallbackMode::STATE => openarm::CallbackMode::STATE,
            PyCallbackMode::PARAM => openarm::CallbackMode::PARAM,
            PyCallbackMode::IGNORE => openarm::CallbackMode::IGNORE,
        }
    }
}

impl From<openarm::CallbackMode> for PyCallbackMode {
    fn from(m: openarm::CallbackMode) -> Self {
        match m {
            openarm::CallbackMode::STATE => PyCallbackMode::STATE,
            openarm::CallbackMode::PARAM => PyCallbackMode::PARAM,
            openarm::CallbackMode::IGNORE => PyCallbackMode::IGNORE,
        }
    }
}

/// Motor variable (register) IDs for parameter queries.
#[pyclass(name = "MotorVariable", eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types)]
pub enum PyMotorVariable {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    Ls = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GTEFP = 30,
    GTEFN = 31,
    Alias = 32,
    codeVersion = 33,
    motorType = 34,
    canRateLevel = 35,
    canIdLevel = 36,
    CBKP = 37,
    CBKD = 38,
    sub_ver = 39,
    u_off = 40,
    v_off = 41,
    k1 = 42,
    k2 = 43,
    m_off = 44,
    dir = 45,
    p_m = 46,
    xout = 47,
    enableBKP = 48,
    bkp_loc = 49,
    PMIN = 50,
    masterid = 51,
    isReduction = 52,
    run_state = 56,
    error_state = 80,
    CUR_angle = 81,
}

impl From<PyMotorVariable> for openarm::MotorVariable {
    fn from(v: PyMotorVariable) -> Self {
        match v {
            PyMotorVariable::UV_Value => openarm::MotorVariable::UV_Value,
            PyMotorVariable::KT_Value => openarm::MotorVariable::KT_Value,
            PyMotorVariable::OT_Value => openarm::MotorVariable::OT_Value,
            PyMotorVariable::OC_Value => openarm::MotorVariable::OC_Value,
            PyMotorVariable::ACC => openarm::MotorVariable::ACC,
            PyMotorVariable::DEC => openarm::MotorVariable::DEC,
            PyMotorVariable::MAX_SPD => openarm::MotorVariable::MAX_SPD,
            PyMotorVariable::MST_ID => openarm::MotorVariable::MST_ID,
            PyMotorVariable::ESC_ID => openarm::MotorVariable::ESC_ID,
            PyMotorVariable::TIMEOUT => openarm::MotorVariable::TIMEOUT,
            PyMotorVariable::CTRL_MODE => openarm::MotorVariable::CTRL_MODE,
            PyMotorVariable::Damp => openarm::MotorVariable::Damp,
            PyMotorVariable::Inertia => openarm::MotorVariable::Inertia,
            PyMotorVariable::hw_ver => openarm::MotorVariable::hw_ver,
            PyMotorVariable::sw_ver => openarm::MotorVariable::sw_ver,
            PyMotorVariable::SN => openarm::MotorVariable::SN,
            PyMotorVariable::NPP => openarm::MotorVariable::NPP,
            PyMotorVariable::Rs => openarm::MotorVariable::Rs,
            PyMotorVariable::Ls => openarm::MotorVariable::Ls,
            PyMotorVariable::Flux => openarm::MotorVariable::Flux,
            PyMotorVariable::Gr => openarm::MotorVariable::Gr,
            PyMotorVariable::PMAX => openarm::MotorVariable::PMAX,
            PyMotorVariable::VMAX => openarm::MotorVariable::VMAX,
            PyMotorVariable::TMAX => openarm::MotorVariable::TMAX,
            PyMotorVariable::I_BW => openarm::MotorVariable::I_BW,
            PyMotorVariable::KP_ASR => openarm::MotorVariable::KP_ASR,
            PyMotorVariable::KI_ASR => openarm::MotorVariable::KI_ASR,
            PyMotorVariable::KP_APR => openarm::MotorVariable::KP_APR,
            PyMotorVariable::KI_APR => openarm::MotorVariable::KI_APR,
            PyMotorVariable::OV_Value => openarm::MotorVariable::OV_Value,
            PyMotorVariable::GTEFP => openarm::MotorVariable::GTEFP,
            PyMotorVariable::GTEFN => openarm::MotorVariable::GTEFN,
            PyMotorVariable::Alias => openarm::MotorVariable::Alias,
            PyMotorVariable::codeVersion => openarm::MotorVariable::codeVersion,
            PyMotorVariable::motorType => openarm::MotorVariable::motorType,
            PyMotorVariable::canRateLevel => openarm::MotorVariable::canRateLevel,
            PyMotorVariable::canIdLevel => openarm::MotorVariable::canIdLevel,
            PyMotorVariable::CBKP => openarm::MotorVariable::CBKP,
            PyMotorVariable::CBKD => openarm::MotorVariable::CBKD,
            PyMotorVariable::sub_ver => openarm::MotorVariable::sub_ver,
            PyMotorVariable::u_off => openarm::MotorVariable::u_off,
            PyMotorVariable::v_off => openarm::MotorVariable::v_off,
            PyMotorVariable::k1 => openarm::MotorVariable::k1,
            PyMotorVariable::k2 => openarm::MotorVariable::k2,
            PyMotorVariable::m_off => openarm::MotorVariable::m_off,
            PyMotorVariable::dir => openarm::MotorVariable::dir,
            PyMotorVariable::p_m => openarm::MotorVariable::p_m,
            PyMotorVariable::xout => openarm::MotorVariable::xout,
            PyMotorVariable::enableBKP => openarm::MotorVariable::enableBKP,
            PyMotorVariable::bkp_loc => openarm::MotorVariable::bkp_loc,
            PyMotorVariable::PMIN => openarm::MotorVariable::PMIN,
            PyMotorVariable::masterid => openarm::MotorVariable::masterid,
            PyMotorVariable::isReduction => openarm::MotorVariable::isReduction,
            PyMotorVariable::run_state => openarm::MotorVariable::run_state,
            PyMotorVariable::error_state => openarm::MotorVariable::error_state,
            PyMotorVariable::CUR_angle => openarm::MotorVariable::CUR_angle,
        }
    }
}

/// Motor limit parameters.
#[pyclass(name = "LimitParam", get_all, set_all)]
#[derive(Debug, Clone, Copy)]
pub struct PyLimitParam {
    pub p_max: f64,
    pub v_max: f64,
    pub t_max: f64,
}

#[pymethods]
impl PyLimitParam {
    #[new]
    #[pyo3(signature = (p_max, v_max, t_max))]
    pub fn new(p_max: f64, v_max: f64, t_max: f64) -> Self {
        Self { p_max, v_max, t_max }
    }

    fn __repr__(&self) -> String {
        format!(
            "LimitParam(p_max={}, v_max={}, t_max={})",
            self.p_max, self.v_max, self.t_max
        )
    }
}

/// Result of a motor state query.
#[pyclass(name = "MotorStateResult", get_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PyMotorStateResult {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub t_mos: i32,
    pub t_rotor: i32,
    pub valid: bool,
}

#[pymethods]
impl PyMotorStateResult {
    #[new]
    #[pyo3(signature = (position=0.0, velocity=0.0, torque=0.0, t_mos=0, t_rotor=0, valid=false))]
    pub fn new(
        position: f64,
        velocity: f64,
        torque: f64,
        t_mos: i32,
        t_rotor: i32,
        valid: bool,
    ) -> Self {
        Self {
            position,
            velocity,
            torque,
            t_mos,
            t_rotor,
            valid,
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "MotorStateResult(position={}, velocity={}, torque={}, t_mos={}, t_rotor={}, valid={})",
            self.position, self.velocity, self.torque, self.t_mos, self.t_rotor, self.valid
        )
    }
}

impl From<openarm::MotorStateResult> for PyMotorStateResult {
    fn from(r: openarm::MotorStateResult) -> Self {
        Self {
            position: r.position,
            velocity: r.velocity,
            torque: r.torque,
            t_mos: r.t_mos,
            t_rotor: r.t_rotor,
            valid: r.valid,
        }
    }
}

/// Result of a parameter query.
#[pyclass(name = "ParamResult", get_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PyParamResult {
    pub rid: i32,
    pub value: f64,
    pub valid: bool,
}

#[pymethods]
impl PyParamResult {
    #[new]
    #[pyo3(signature = (rid=0, value=0.0, valid=false))]
    pub fn new(rid: i32, value: f64, valid: bool) -> Self {
        Self { rid, value, valid }
    }

    fn __repr__(&self) -> String {
        format!(
            "ParamResult(rid={}, value={}, valid={})",
            self.rid, self.value, self.valid
        )
    }
}

impl From<openarm::ParamResult> for PyParamResult {
    fn from(r: openarm::ParamResult) -> Self {
        Self {
            rid: r.rid,
            value: r.value,
            valid: r.valid,
        }
    }
}

/// MIT control parameters.
#[pyclass(name = "MITParam", get_all, set_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PyMITParam {
    pub kp: f64,
    pub kd: f64,
    pub q: f64,
    pub dq: f64,
    pub tau: f64,
}

#[pymethods]
impl PyMITParam {
    #[new]
    #[pyo3(signature = (kp=0.0, kd=0.0, q=0.0, dq=0.0, tau=0.0))]
    pub fn new(kp: f64, kd: f64, q: f64, dq: f64, tau: f64) -> Self {
        Self { kp, kd, q, dq, tau }
    }

    fn __repr__(&self) -> String {
        format!(
            "MITParam(kp={}, kd={}, q={}, dq={}, tau={})",
            self.kp, self.kd, self.q, self.dq, self.tau
        )
    }
}

impl From<&PyMITParam> for openarm::MITParam {
    fn from(p: &PyMITParam) -> Self {
        Self {
            kp: p.kp,
            kd: p.kd,
            q: p.q,
            dq: p.dq,
            tau: p.tau,
        }
    }
}

/// Position-velocity control parameters.
#[pyclass(name = "PosVelParam", get_all, set_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PyPosVelParam {
    pub q: f64,
    pub dq: f64,
}

#[pymethods]
impl PyPosVelParam {
    #[new]
    #[pyo3(signature = (q=0.0, dq=0.0))]
    pub fn new(q: f64, dq: f64) -> Self {
        Self { q, dq }
    }

    fn __repr__(&self) -> String {
        format!("PosVelParam(q={}, dq={})", self.q, self.dq)
    }
}

impl From<&PyPosVelParam> for openarm::PosVelParam {
    fn from(p: &PyPosVelParam) -> Self {
        Self { q: p.q, dq: p.dq }
    }
}

/// Position-force control parameters.
#[pyclass(name = "PosForceParam", get_all, set_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PyPosForceParam {
    pub q: f64,
    pub dq: f64,
    pub i: f64,
}

#[pymethods]
impl PyPosForceParam {
    #[new]
    #[pyo3(signature = (q=0.0, dq=0.0, i=0.0))]
    pub fn new(q: f64, dq: f64, i: f64) -> Self {
        Self { q, dq, i }
    }

    fn __repr__(&self) -> String {
        format!("PosForceParam(q={}, dq={}, i={})", self.q, self.dq, self.i)
    }
}

impl From<&PyPosForceParam> for openarm::PosForceParam {
    fn from(p: &PyPosForceParam) -> Self {
        Self {
            q: p.q,
            dq: p.dq,
            i: p.i,
        }
    }
}

/// CAN packet for transmission.
#[pyclass(name = "CANPacket", get_all)]
#[derive(Debug, Clone)]
pub struct PyCANPacket {
    pub send_can_id: u32,
    pub data: Vec<u8>,
}

#[pymethods]
impl PyCANPacket {
    #[new]
    pub fn new(send_can_id: u32, data: Vec<u8>) -> Self {
        Self { send_can_id, data }
    }

    fn __repr__(&self) -> String {
        format!(
            "CANPacket(send_can_id=0x{:X}, data={:?})",
            self.send_can_id, self.data
        )
    }
}

impl From<openarm::CANPacket> for PyCANPacket {
    fn from(p: openarm::CANPacket) -> Self {
        Self {
            send_can_id: p.send_can_id,
            data: p.data,
        }
    }
}

/// Motor state container wrapper.
#[pyclass(name = "Motor")]
#[derive(Clone)]
pub struct PyMotor {
    pub(crate) inner: openarm::Motor,
}

#[pymethods]
impl PyMotor {
    #[new]
    #[pyo3(signature = (motor_type, send_can_id, recv_can_id, control_mode=PyControlMode::MIT))]
    pub fn new(
        motor_type: PyMotorType,
        send_can_id: u32,
        recv_can_id: u32,
        control_mode: PyControlMode,
    ) -> Self {
        Self {
            inner: openarm::Motor::new(motor_type.into(), send_can_id, recv_can_id, control_mode.into()),
        }
    }

    /// Get the motor type.
    #[getter]
    pub fn get_motor_type(&self) -> PyMotorType {
        self.inner.motor_type().into()
    }

    /// Get the send CAN ID.
    #[getter]
    pub fn get_send_can_id(&self) -> u32 {
        self.inner.send_can_id()
    }

    /// Get the receive CAN ID.
    #[getter]
    pub fn get_recv_can_id(&self) -> u32 {
        self.inner.recv_can_id()
    }

    /// Get the control mode.
    #[getter]
    pub fn get_control_mode(&self) -> PyControlMode {
        self.inner.control_mode().into()
    }

    /// Get the current position (rad).
    pub fn get_position(&self) -> f64 {
        self.inner.get_position()
    }

    /// Get the current velocity (rad/s).
    pub fn get_velocity(&self) -> f64 {
        self.inner.get_velocity()
    }

    /// Get the current torque (Nm).
    pub fn get_torque(&self) -> f64 {
        self.inner.get_torque()
    }

    /// Get the MOS temperature.
    pub fn get_state_tmos(&self) -> i32 {
        self.inner.get_state_tmos()
    }

    /// Get the rotor temperature.
    pub fn get_state_trotor(&self) -> i32 {
        self.inner.get_state_trotor()
    }

    /// Check if motor is enabled.
    pub fn is_enabled(&self) -> bool {
        self.inner.is_enabled()
    }

    /// Get a temporary parameter value.
    pub fn get_temp_param(&self, rid: i32) -> Option<f64> {
        self.inner.get_temp_param(rid)
    }

    fn __repr__(&self) -> String {
        format!(
            "Motor(type={:?}, send_id=0x{:X}, recv_id=0x{:X}, pos={:.3}, vel={:.3}, tau={:.3})",
            self.inner.motor_type(),
            self.inner.send_can_id(),
            self.inner.recv_can_id(),
            self.inner.get_position(),
            self.inner.get_velocity(),
            self.inner.get_torque()
        )
    }
}

impl From<openarm::Motor> for PyMotor {
    fn from(m: openarm::Motor) -> Self {
        Self { inner: m }
    }
}

/// CAN packet encoder wrapper.
#[pyclass(name = "CanPacketEncoder")]
#[derive(Debug, Clone, Default)]
pub struct PyCanPacketEncoder;

#[pymethods]
impl PyCanPacketEncoder {
    #[new]
    pub fn new() -> Self {
        Self
    }

    /// Create enable command.
    #[staticmethod]
    pub fn create_enable_command(motor: &PyMotor) -> PyCANPacket {
        openarm::CanPacketEncoder::create_enable_command(&motor.inner).into()
    }

    /// Create disable command.
    #[staticmethod]
    pub fn create_disable_command(motor: &PyMotor) -> PyCANPacket {
        openarm::CanPacketEncoder::create_disable_command(&motor.inner).into()
    }

    /// Create set zero command.
    #[staticmethod]
    pub fn create_set_zero_command(motor: &PyMotor) -> PyCANPacket {
        openarm::CanPacketEncoder::create_set_zero_command(&motor.inner).into()
    }

    /// Create refresh command.
    #[staticmethod]
    pub fn create_refresh_command(motor: &PyMotor) -> PyCANPacket {
        openarm::CanPacketEncoder::create_refresh_command(&motor.inner).into()
    }

    /// Create MIT control command.
    #[staticmethod]
    pub fn create_mit_control_command(motor: &PyMotor, param: &PyMITParam) -> PyCANPacket {
        openarm::CanPacketEncoder::create_mit_control_command(&motor.inner, &param.into()).into()
    }

    /// Create position-velocity control command.
    #[staticmethod]
    pub fn create_posvel_control_command(motor: &PyMotor, param: &PyPosVelParam) -> PyCANPacket {
        openarm::CanPacketEncoder::create_posvel_control_command(&motor.inner, &param.into()).into()
    }

    /// Create position-force control command.
    #[staticmethod]
    pub fn create_posforce_control_command(motor: &PyMotor, param: &PyPosForceParam) -> PyCANPacket {
        openarm::CanPacketEncoder::create_posforce_control_command(&motor.inner, &param.into()).into()
    }

    /// Create set control mode command.
    #[staticmethod]
    pub fn create_set_control_mode_command(motor: &PyMotor, mode: PyControlMode) -> PyCANPacket {
        openarm::CanPacketEncoder::create_set_control_mode_command(&motor.inner, mode.into()).into()
    }

    /// Create query parameter command.
    #[staticmethod]
    pub fn create_query_param_command(motor: &PyMotor, rid: PyMotorVariable) -> PyCANPacket {
        openarm::CanPacketEncoder::create_query_param_command(&motor.inner, rid.into()).into()
    }
}

/// CAN packet decoder wrapper.
#[pyclass(name = "CanPacketDecoder")]
#[derive(Debug, Clone, Default)]
pub struct PyCanPacketDecoder;

#[pymethods]
impl PyCanPacketDecoder {
    #[new]
    pub fn new() -> Self {
        Self
    }

    /// Parse motor state data from CAN frame.
    #[staticmethod]
    pub fn parse_motor_state_data(motor: &PyMotor, data: Vec<u8>) -> PyMotorStateResult {
        openarm::CanPacketDecoder::parse_motor_state_data(&motor.inner, &data).into()
    }

    /// Parse parameter data from CAN frame.
    #[staticmethod]
    pub fn parse_motor_param_data(data: Vec<u8>) -> PyParamResult {
        openarm::CanPacketDecoder::parse_motor_param_data(&data).into()
    }
}
