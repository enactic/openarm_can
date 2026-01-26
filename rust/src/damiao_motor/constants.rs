//! Motor type definitions, control modes, and protocol constants for Damiao motors.

use pyo3::prelude::*;

/// Motor types supported by the Damiao motor family.
#[pyclass(eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotorType {
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

/// Control modes for motor operation.
#[pyclass(eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[allow(non_camel_case_types)]
pub enum ControlMode {
    #[default]
    MIT = 1,
    POS_VEL = 2,
    VEL = 3,
    POS_FORCE = 4,
}

/// Callback modes for frame processing.
#[pyclass(eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CallbackMode {
    #[default]
    STATE = 0,
    PARAM = 1,
    IGNORE = 2,
}

/// Motor variable (register) IDs for parameter queries.
#[pyclass(eq, eq_int)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types)]
pub enum MotorVariable {
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

/// Motor limit parameters (position, velocity, torque limits).
#[pyclass(get_all, set_all)]
#[derive(Debug, Clone, Copy)]
pub struct LimitParam {
    pub p_max: f64,
    pub v_max: f64,
    pub t_max: f64,
}

#[pymethods]
impl LimitParam {
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

/// Motor-specific limit parameters indexed by MotorType.
pub const MOTOR_LIMIT_PARAMS: [LimitParam; 13] = [
    LimitParam { p_max: 12.5, v_max: 50.0, t_max: 5.0 },    // DM3507
    LimitParam { p_max: 12.5, v_max: 30.0, t_max: 10.0 },   // DM4310
    LimitParam { p_max: 12.5, v_max: 50.0, t_max: 10.0 },   // DM4310_48V
    LimitParam { p_max: 12.5, v_max: 10.0, t_max: 28.0 },   // DM4340
    LimitParam { p_max: 12.5, v_max: 10.0, t_max: 28.0 },   // DM4340_48V
    LimitParam { p_max: 12.5, v_max: 45.0, t_max: 1.2 },    // DM6006
    LimitParam { p_max: 12.5, v_max: 45.0, t_max: 3.0 },    // DM8006
    LimitParam { p_max: 12.5, v_max: 25.0, t_max: 54.0 },   // DM8009
    LimitParam { p_max: 12.5, v_max: 20.0, t_max: 60.0 },   // DM10010L
    LimitParam { p_max: 12.5, v_max: 20.0, t_max: 100.0 },  // DM10010
    LimitParam { p_max: 12.5, v_max: 280.0, t_max: 0.75 },  // DMH3510
    LimitParam { p_max: 12.5, v_max: 100.0, t_max: 13.4 },  // DMH6215
    LimitParam { p_max: 12.5, v_max: 100.0, t_max: 20.0 },  // DMG6220
];

impl MotorType {
    /// Get the limit parameters for this motor type.
    pub fn get_limits(&self) -> &'static LimitParam {
        &MOTOR_LIMIT_PARAMS[*self as usize]
    }
}

/// Result of a motor state query.
#[pyclass(get_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorStateResult {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub t_mos: i32,
    pub t_rotor: i32,
    pub valid: bool,
}

#[pymethods]
impl MotorStateResult {
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

/// Result of a parameter query.
#[pyclass(get_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ParamResult {
    pub rid: i32,
    pub value: f64,
    pub valid: bool,
}

#[pymethods]
impl ParamResult {
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

/// MIT control parameters.
#[pyclass(get_all, set_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct MITParam {
    pub kp: f64,
    pub kd: f64,
    pub q: f64,
    pub dq: f64,
    pub tau: f64,
}

#[pymethods]
impl MITParam {
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

/// Position-velocity control parameters.
#[pyclass(get_all, set_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PosVelParam {
    pub q: f64,
    pub dq: f64,
}

#[pymethods]
impl PosVelParam {
    #[new]
    #[pyo3(signature = (q=0.0, dq=0.0))]
    pub fn new(q: f64, dq: f64) -> Self {
        Self { q, dq }
    }

    fn __repr__(&self) -> String {
        format!("PosVelParam(q={}, dq={})", self.q, self.dq)
    }
}

/// Position-force control parameters.
#[pyclass(get_all, set_all)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PosForceParam {
    pub q: f64,
    pub dq: f64,
    pub i: f64,
}

#[pymethods]
impl PosForceParam {
    #[new]
    #[pyo3(signature = (q=0.0, dq=0.0, i=0.0))]
    pub fn new(q: f64, dq: f64, i: f64) -> Self {
        Self { q, dq, i }
    }

    fn __repr__(&self) -> String {
        format!("PosForceParam(q={}, dq={}, i={})", self.q, self.dq, self.i)
    }
}

/// CAN packet for transmission.
#[pyclass(get_all)]
#[derive(Debug, Clone)]
pub struct CANPacket {
    pub send_can_id: u32,
    pub data: Vec<u8>,
}

#[pymethods]
impl CANPacket {
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
