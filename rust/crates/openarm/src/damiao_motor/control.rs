//! CAN packet encoding and decoding for Damiao motor protocol.

use super::constants::*;
use super::motor::Motor;

/// Clamp a value to a range.
fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

/// Scale a float to an unsigned integer.
fn float_to_uint(x: f64, x_min: f64, x_max: f64, bits: u32) -> u32 {
    let span = x_max - x_min;
    let offset = x - x_min;
    let max_val = ((1u64 << bits) - 1) as f64;
    ((offset / span) * max_val) as u32
}

/// Scale an unsigned integer to a float.
fn uint_to_float(x: u32, x_min: f64, x_max: f64, bits: u32) -> f64 {
    let span = x_max - x_min;
    let max_val = ((1u64 << bits) - 1) as f64;
    x_min + (x as f64 / max_val) * span
}

/// CAN packet encoder for Damiao motor commands.
#[derive(Debug, Clone, Default)]
pub struct CanPacketEncoder;

impl CanPacketEncoder {
    /// Create a new encoder.
    pub fn new() -> Self {
        Self
    }

    /// Create enable command.
    pub fn create_enable_command(motor: &Motor) -> CANPacket {
        CANPacket {
            send_can_id: motor.send_can_id(),
            data: vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        }
    }

    /// Create disable command.
    pub fn create_disable_command(motor: &Motor) -> CANPacket {
        CANPacket {
            send_can_id: motor.send_can_id(),
            data: vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
        }
    }

    /// Create set zero command (flash current position as zero).
    pub fn create_set_zero_command(motor: &Motor) -> CANPacket {
        CANPacket {
            send_can_id: motor.send_can_id(),
            data: vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
        }
    }

    /// Create refresh command (request state update).
    pub fn create_refresh_command(motor: &Motor) -> CANPacket {
        let can_id = motor.send_can_id();
        CANPacket {
            send_can_id: 0x7FF,
            data: vec![
                (can_id & 0xFF) as u8,
                ((can_id >> 8) & 0xFF) as u8,
                0xCC,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        }
    }

    /// Create MIT control command.
    pub fn create_mit_control_command(motor: &Motor, param: &MITParam) -> CANPacket {
        let limits = motor.motor_type().get_limits();

        // Clamp values to limits
        let q = clamp(param.q, -limits.p_max, limits.p_max);
        let dq = clamp(param.dq, -limits.v_max, limits.v_max);
        let tau = clamp(param.tau, -limits.t_max, limits.t_max);
        let kp = clamp(param.kp, 0.0, 500.0);
        let kd = clamp(param.kd, 0.0, 5.0);

        // Scale to integers
        let q_int = float_to_uint(q, -limits.p_max, limits.p_max, 16);
        let dq_int = float_to_uint(dq, -limits.v_max, limits.v_max, 12);
        let kp_int = float_to_uint(kp, 0.0, 500.0, 12);
        let kd_int = float_to_uint(kd, 0.0, 5.0, 12);
        let tau_int = float_to_uint(tau, -limits.t_max, limits.t_max, 12);

        // Pack into 8 bytes
        let mut data = vec![0u8; 8];
        data[0] = (q_int >> 8) as u8;
        data[1] = (q_int & 0xFF) as u8;
        data[2] = (dq_int >> 4) as u8;
        data[3] = ((dq_int & 0x0F) << 4) as u8 | ((kp_int >> 8) & 0x0F) as u8;
        data[4] = (kp_int & 0xFF) as u8;
        data[5] = (kd_int >> 4) as u8;
        data[6] = ((kd_int & 0x0F) << 4) as u8 | ((tau_int >> 8) & 0x0F) as u8;
        data[7] = (tau_int & 0xFF) as u8;

        CANPacket {
            send_can_id: motor.send_can_id(),
            data,
        }
    }

    /// Create position-velocity control command.
    pub fn create_posvel_control_command(motor: &Motor, param: &PosVelParam) -> CANPacket {
        let limits = motor.motor_type().get_limits();

        let q = clamp(param.q, -limits.p_max, limits.p_max);
        let dq = clamp(param.dq, -limits.v_max, limits.v_max);

        // Convert to fixed-point representation
        let q_bytes = (q * 10000.0) as i32;
        let dq_bytes = (dq * 10000.0) as i32;

        let mut data = vec![0u8; 8];
        data[0] = (q_bytes & 0xFF) as u8;
        data[1] = ((q_bytes >> 8) & 0xFF) as u8;
        data[2] = ((q_bytes >> 16) & 0xFF) as u8;
        data[3] = ((q_bytes >> 24) & 0xFF) as u8;
        data[4] = (dq_bytes & 0xFF) as u8;
        data[5] = ((dq_bytes >> 8) & 0xFF) as u8;
        data[6] = ((dq_bytes >> 16) & 0xFF) as u8;
        data[7] = ((dq_bytes >> 24) & 0xFF) as u8;

        CANPacket {
            send_can_id: motor.send_can_id() + 0x100,
            data,
        }
    }

    /// Create position-force control command.
    pub fn create_posforce_control_command(motor: &Motor, param: &PosForceParam) -> CANPacket {
        let limits = motor.motor_type().get_limits();

        let q = clamp(param.q, -limits.p_max, limits.p_max);
        let dq = clamp(param.dq, 0.0, limits.v_max);
        let i = clamp(param.i, 0.0, 1.0);

        // Convert to fixed-point representation
        let q_bytes = (q * 10000.0) as i32;
        let dq_scaled = (dq * 100.0) as u16;
        let i_scaled = (i * 10000.0) as u16;

        let mut data = vec![0u8; 8];
        data[0] = (q_bytes & 0xFF) as u8;
        data[1] = ((q_bytes >> 8) & 0xFF) as u8;
        data[2] = ((q_bytes >> 16) & 0xFF) as u8;
        data[3] = ((q_bytes >> 24) & 0xFF) as u8;
        data[4] = (dq_scaled & 0xFF) as u8;
        data[5] = ((dq_scaled >> 8) & 0xFF) as u8;
        data[6] = (i_scaled & 0xFF) as u8;
        data[7] = ((i_scaled >> 8) & 0xFF) as u8;

        CANPacket {
            send_can_id: motor.send_can_id() + 0x300,
            data,
        }
    }

    /// Create set control mode command.
    pub fn create_set_control_mode_command(motor: &Motor, mode: ControlMode) -> CANPacket {
        let can_id = motor.send_can_id();
        let mode_val = mode as u8;

        CANPacket {
            send_can_id: 0x7FF,
            data: vec![
                (can_id & 0xFF) as u8,
                ((can_id >> 8) & 0xFF) as u8,
                0x55,
                MotorVariable::CTRL_MODE as u8,
                mode_val,
                0x00,
                0x00,
                0x00,
            ],
        }
    }

    /// Create query parameter command.
    pub fn create_query_param_command(motor: &Motor, rid: MotorVariable) -> CANPacket {
        let can_id = motor.send_can_id();

        CANPacket {
            send_can_id: 0x7FF,
            data: vec![
                (can_id & 0xFF) as u8,
                ((can_id >> 8) & 0xFF) as u8,
                0x33,
                rid as u8,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        }
    }
}

/// CAN packet decoder for Damiao motor responses.
#[derive(Debug, Clone, Default)]
pub struct CanPacketDecoder;

impl CanPacketDecoder {
    /// Create a new decoder.
    pub fn new() -> Self {
        Self
    }

    /// Parse motor state data from CAN frame.
    pub fn parse_motor_state_data(motor: &Motor, data: &[u8]) -> MotorStateResult {
        if data.len() < 8 {
            return MotorStateResult {
                valid: false,
                ..Default::default()
            };
        }

        let limits = motor.motor_type().get_limits();

        // Extract raw values from packed data
        let q_raw = ((data[1] as u32) << 8) | (data[2] as u32);
        let dq_raw = ((data[3] as u32) << 4) | ((data[4] >> 4) as u32);
        let tau_raw = (((data[4] & 0x0F) as u32) << 8) | (data[5] as u32);
        let t_mos = data[6] as i32;
        let t_rotor = data[7] as i32;

        // Convert to physical values
        let position = uint_to_float(q_raw, -limits.p_max, limits.p_max, 16);
        let velocity = uint_to_float(dq_raw, -limits.v_max, limits.v_max, 12);
        let torque = uint_to_float(tau_raw, -limits.t_max, limits.t_max, 12);

        MotorStateResult {
            position,
            velocity,
            torque,
            t_mos,
            t_rotor,
            valid: true,
        }
    }

    /// Parse parameter data from CAN frame.
    pub fn parse_motor_param_data(data: &[u8]) -> ParamResult {
        if data.len() < 8 {
            return ParamResult {
                valid: false,
                ..Default::default()
            };
        }

        let rid = data[3] as i32;

        // Extract value bytes (little-endian)
        let value_bytes = [data[4], data[5], data[6], data[7]];

        // Determine if it's a float or int based on RID
        // Most parameters are floats, some are ints
        let value = if rid == MotorVariable::CTRL_MODE as i32
            || rid == MotorVariable::MST_ID as i32
            || rid == MotorVariable::ESC_ID as i32
        {
            i32::from_le_bytes(value_bytes) as f64
        } else {
            f32::from_le_bytes(value_bytes) as f64
        };

        ParamResult {
            rid,
            value,
            valid: true,
        }
    }

    /// Parse and update motor state.
    pub fn parse_and_update_motor_state(motor: &Motor, data: &[u8]) -> bool {
        if data.len() < 8 {
            return false;
        }

        let limits = motor.motor_type().get_limits();

        // Extract raw values from packed data
        let q_raw = ((data[1] as u32) << 8) | (data[2] as u32);
        let dq_raw = ((data[3] as u32) << 4) | ((data[4] >> 4) as u32);
        let tau_raw = (((data[4] & 0x0F) as u32) << 8) | (data[5] as u32);
        let t_mos = data[6] as i32;
        let t_rotor = data[7] as i32;

        // Convert to physical values
        let position = uint_to_float(q_raw, -limits.p_max, limits.p_max, 16);
        let velocity = uint_to_float(dq_raw, -limits.v_max, limits.v_max, 12);
        let torque = uint_to_float(tau_raw, -limits.t_max, limits.t_max, 12);

        motor.update_state(position, velocity, torque, t_mos, t_rotor);
        true
    }

    /// Parse and store parameter result.
    pub fn parse_and_store_param(motor: &Motor, data: &[u8]) -> bool {
        if data.len() < 8 {
            return false;
        }

        let rid = data[3] as i32;
        let value_bytes = [data[4], data[5], data[6], data[7]];

        let value = if rid == MotorVariable::CTRL_MODE as i32
            || rid == MotorVariable::MST_ID as i32
            || rid == MotorVariable::ESC_ID as i32
        {
            i32::from_le_bytes(value_bytes) as f64
        } else {
            f32::from_le_bytes(value_bytes) as f64
        };

        motor.set_temp_param(rid, value);
        true
    }
}
