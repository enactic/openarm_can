"""
OpenArm CAN Python bindings for motor control via SocketCAN
"""
from __future__ import annotations
import collections.abc
import enum
import typing
__all__: list[str] = ['ACC', 'ArmComponent', 'CANDevice', 'CANDeviceCollection', 'CANPacket', 'CANSocket', 'CANSocketException', 'COUNT', 'CTRL_MODE', 'CallbackMode', 'CanFdFrame', 'CanFrame', 'CanPacketDecoder', 'CanPacketEncoder', 'ControlMode', 'DEC', 'DM10010', 'DM10010L', 'DM3507', 'DM4310', 'DM4310_48V', 'DM4340', 'DM4340_48V', 'DM6006', 'DM8006', 'DM8009', 'DMDeviceCollection', 'DMG6220', 'DMH3510', 'DMH6215', 'Damp', 'Deta', 'ESC_ID', 'Flux', 'GREF', 'Gr', 'GripperComponent', 'IGNORE', 'IQ_c1', 'I_BW', 'Inertia', 'KI_APR', 'KI_ASR', 'KP_APR', 'KP_ASR', 'KT_Value', 'LS', 'LimitParam', 'MAX_SPD', 'MIT', 'MITParam', 'MST_ID', 'Motor', 'MotorDeviceCan', 'MotorStateResult', 'MotorType', 'MotorVariable', 'NPP', 'OC_Value', 'OT_Value', 'OV_Value', 'OpenArm', 'PARAM', 'PMAX', 'POS_FORCE', 'POS_VEL', 'ParamResult', 'PosForceParam', 'PosVelParam', 'Rs', 'SN', 'STATE', 'TIMEOUT', 'TMAX', 'UV_Value', 'VEL', 'VL_c1', 'VMAX', 'V_BW', 'can_br', 'dir', 'hw_ver', 'k1', 'k2', 'm_off', 'p_m', 'sub_ver', 'sw_ver', 'u_off', 'v_off', 'xout']
class ArmComponent(DMDeviceCollection):
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self, can_socket: CANSocket) -> None:
        ...
    def init_motor_devices(self, motor_types: collections.abc.Sequence[MotorType], send_can_ids: collections.abc.Sequence[int], recv_can_ids: collections.abc.Sequence[int], use_fd: bool, control_modes: collections.abc.Sequence[ControlMode] = []) -> None:
        ...
class CANDevice:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def get_recv_can_id(self) -> int:
        ...
    def get_recv_can_mask(self) -> int:
        ...
    def get_send_can_id(self) -> int:
        ...
    def is_fd_enabled(self) -> bool:
        ...
class CANDeviceCollection:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self, can_socket: CANSocket) -> None:
        ...
    def add_device(self, device: ...) -> None:
        ...
    def dispatch_frame_callback(self, frame: CanFrame) -> None:
        """
        dispatch_frame_callback(self, frame: openarm_can.openarm_can.CanFdFrame) -> None
        """
    def get_devices(self) -> ...:
        ...
    def remove_device(self, device: ...) -> None:
        ...
class CANPacket:
    send_can_id: int
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self) -> None:
        ...
    @property
    def data(self) -> list[int]:
        ...
    @data.setter
    def data(self, arg: collections.abc.Sequence[int]) -> None:
        ...
class CANSocket:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self, interface: str, enable_fd: bool = False) -> None:
        ...
    def get_interface(self) -> str:
        ...
    def get_socket_fd(self) -> int:
        ...
    def is_canfd_enabled(self) -> bool:
        ...
    def is_initialized(self) -> bool:
        ...
    def read_can_frame(self, frame: CanFrame) -> bool:
        ...
    def read_canfd_frame(self, frame: CanFdFrame) -> bool:
        ...
    def read_raw_frame(self, buffer_size: int) -> bytes:
        ...
    def write_can_frame(self, frame: CanFrame) -> bool:
        ...
    def write_canfd_frame(self, frame: CanFdFrame) -> bool:
        ...
    def write_raw_frame(self, data: bytes) -> int:
        ...
class CANSocketException(Exception):
    pass
class CallbackMode(enum.Enum):
    IGNORE: typing.ClassVar[CallbackMode]  # value = CallbackMode.IGNORE
    PARAM: typing.ClassVar[CallbackMode]  # value = CallbackMode.PARAM
    STATE: typing.ClassVar[CallbackMode]  # value = CallbackMode.STATE
class CanFdFrame:
    can_id: int
    data: bytes
    flags: int
    len: int
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self) -> None:
        ...
class CanFrame:
    can_dlc: int
    can_id: int
    data: bytes
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self) -> None:
        ...
class CanPacketDecoder:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @staticmethod
    def parse_motor_state_data(
        motor: Motor,
        data: collections.abc.Sequence[int],
    ) -> MotorStateResult:
        ...

    @staticmethod
    def parse_motor_param_data(
        data: collections.abc.Sequence[int],
    ) -> ParamResult:
        ...
class CanPacketEncoder:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @staticmethod
    def create_refresh_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_enable_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_disable_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_set_zero_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_mit_control_command(motor: Motor, mit_param: MITParam) -> CANPacket:
        ...

    @staticmethod
    def create_posvel_control_command(motor: Motor, posvel_param: PosVelParam) -> CANPacket:
        ...

    @staticmethod
    def create_posforce_control_command(motor: Motor, posforce_param: PosForceParam) -> CANPacket:
        ...

    @staticmethod
    def create_query_param_command(motor: Motor, rid: int) -> CANPacket:
        ...
class ControlMode(enum.Enum):
    MIT: typing.ClassVar[ControlMode]  # value = ControlMode.MIT
    POS_FORCE: typing.ClassVar[ControlMode]  # value = ControlMode.POS_FORCE
    POS_VEL: typing.ClassVar[ControlMode]  # value = ControlMode.POS_VEL
    VEL: typing.ClassVar[ControlMode]  # value = ControlMode.VEL
class DMDeviceCollection:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self, can_socket: CANSocket) -> None:
        ...
    def disable_all(self) -> None:
        ...
    def enable_all(self) -> None:
        ...
    def get_device_collection(self) -> CANDeviceCollection:
        ...
    def get_motors(self) -> list[Motor]:
        ...
    def mit_control_all(self, mit_params: collections.abc.Sequence[MITParam]) -> None:
        ...
    def mit_control_one(self, index: int, mit_param: MITParam) -> None:
        ...
    def posforce_control_all(self, posforce_params: collections.abc.Sequence[PosForceParam]) -> None:
        ...
    def posforce_control_one(self, index: int, posforce_param: PosForceParam) -> None:
        ...
    def posvel_control_all(self, posvel_params: collections.abc.Sequence[PosVelParam]) -> None:
        ...
    def posvel_control_one(self, index: int, posvel_param: PosVelParam) -> None:
        ...
    def query_param_all(self, rid: int) -> None:
        ...
    def refresh_all(self) -> None:
        ...
    def set_callback_mode_all(self, callback_mode: CallbackMode) -> None:
        ...
    def set_control_mode_all(self, mode: ControlMode) -> None:
        ...
    def set_control_mode_one(self, index: int, mode: ControlMode) -> None:
        ...
    def set_zero_all(self) -> None:
        ...
class GripperComponent(DMDeviceCollection):
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self, can_socket: CANSocket) -> None:
        ...
    def get_motor(self) -> Motor:
        ...
    def init_motor_device(self, motor_type: MotorType, send_can_id: int, recv_can_id: int, use_fd: bool, control_mode: ControlMode = ...) -> None:
        ...
    def set_limit(self, speed_rad_s: float, torque_pu: float) -> None:
        """
        Set default gripper limits for pos-force control.
        speed_rad_s: max closing speed in rad/s.
        torque_pu: per-unit current limit [0, 1].
        """
    def set_position(self, position: float, speed_rad_s: float | None = None, torque_pu: float | None = None) -> None:
        """
        Command gripper position with optional per-call limit overrides.
        position: gripper target (0=closed, 1=open).
        speed_rad_s: max closing speed in rad/s.
        torque_pu: per-unit current limit [0, 1].
        """
    def set_position_mit(self, position: float, kp: float = 50.0, kd: float = 1.0) -> None:
        ...
    def set_zero(self) -> None:
        """
        Set current position as zero.
        """
class LimitParam:
    pMax: float
    tMax: float
    vMax: float
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self) -> None:
        ...
class MITParam:
    dq: float
    kd: float
    kp: float
    q: float
    tau: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, kp: float, kd: float, q: float, dq: float, tau: float) -> None:
        ...
class Motor:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    @staticmethod
    def get_limit_param(motor_type: MotorType) -> LimitParam:
        ...
    def __init__(self, motor_type: MotorType, send_can_id: int, recv_can_id: int) -> None:
        ...
    def get_motor_type(self) -> MotorType:
        ...
    def get_param(self, rid: int) -> float:
        ...
    def get_position(self) -> float:
        ...
    def get_recv_can_id(self) -> int:
        ...
    def get_send_can_id(self) -> int:
        ...
    def get_state_tmos(self) -> int:
        ...
    def get_state_trotor(self) -> int:
        ...
    def get_torque(self) -> float:
        ...
    def get_velocity(self) -> float:
        ...
    def is_enabled(self) -> bool:
        ...
class MotorDeviceCan(CANDevice):
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self, motor: Motor, recv_can_mask: int, use_fd: bool) -> None:
        ...
    def callback(self, frame: CanFrame) -> None:
        """
        callback(self, frame: openarm_can.openarm_can.CanFdFrame) -> None
        """
    def create_can_frame(self, send_can_id: int, data: collections.abc.Sequence[int]) -> CanFrame:
        ...
    def create_canfd_frame(self, send_can_id: int, data: collections.abc.Sequence[int]) -> CanFdFrame:
        ...
    def get_motor(self) -> Motor:
        ...
    def set_callback_mode(self, callback_mode: CallbackMode) -> None:
        ...
class MotorStateResult:
    position: float
    t_mos: int
    t_rotor: int
    torque: float
    valid: bool
    velocity: float
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self) -> None:
        ...
class MotorType(enum.Enum):
    COUNT: typing.ClassVar[MotorType]  # value = MotorType.COUNT
    DM10010: typing.ClassVar[MotorType]  # value = MotorType.DM10010
    DM10010L: typing.ClassVar[MotorType]  # value = MotorType.DM10010L
    DM3507: typing.ClassVar[MotorType]  # value = MotorType.DM3507
    DM4310: typing.ClassVar[MotorType]  # value = MotorType.DM4310
    DM4310_48V: typing.ClassVar[MotorType]  # value = MotorType.DM4310_48V
    DM4340: typing.ClassVar[MotorType]  # value = MotorType.DM4340
    DM4340_48V: typing.ClassVar[MotorType]  # value = MotorType.DM4340_48V
    DM6006: typing.ClassVar[MotorType]  # value = MotorType.DM6006
    DM8006: typing.ClassVar[MotorType]  # value = MotorType.DM8006
    DM8009: typing.ClassVar[MotorType]  # value = MotorType.DM8009
    DMG6220: typing.ClassVar[MotorType]  # value = MotorType.DMG6220
    DMH3510: typing.ClassVar[MotorType]  # value = MotorType.DMH3510
    DMH6215: typing.ClassVar[MotorType]  # value = MotorType.DMH6215
class MotorVariable(enum.Enum):
    ACC: typing.ClassVar[MotorVariable]  # value = MotorVariable.ACC
    COUNT: typing.ClassVar[MotorVariable]  # value = MotorVariable.COUNT
    CTRL_MODE: typing.ClassVar[MotorVariable]  # value = MotorVariable.CTRL_MODE
    DEC: typing.ClassVar[MotorVariable]  # value = MotorVariable.DEC
    Damp: typing.ClassVar[MotorVariable]  # value = MotorVariable.Damp
    Deta: typing.ClassVar[MotorVariable]  # value = MotorVariable.Deta
    ESC_ID: typing.ClassVar[MotorVariable]  # value = MotorVariable.ESC_ID
    Flux: typing.ClassVar[MotorVariable]  # value = MotorVariable.Flux
    GREF: typing.ClassVar[MotorVariable]  # value = MotorVariable.GREF
    Gr: typing.ClassVar[MotorVariable]  # value = MotorVariable.Gr
    IQ_c1: typing.ClassVar[MotorVariable]  # value = MotorVariable.IQ_c1
    I_BW: typing.ClassVar[MotorVariable]  # value = MotorVariable.I_BW
    Inertia: typing.ClassVar[MotorVariable]  # value = MotorVariable.Inertia
    KI_APR: typing.ClassVar[MotorVariable]  # value = MotorVariable.KI_APR
    KI_ASR: typing.ClassVar[MotorVariable]  # value = MotorVariable.KI_ASR
    KP_APR: typing.ClassVar[MotorVariable]  # value = MotorVariable.KP_APR
    KP_ASR: typing.ClassVar[MotorVariable]  # value = MotorVariable.KP_ASR
    KT_Value: typing.ClassVar[MotorVariable]  # value = MotorVariable.KT_Value
    LS: typing.ClassVar[MotorVariable]  # value = MotorVariable.LS
    MAX_SPD: typing.ClassVar[MotorVariable]  # value = MotorVariable.MAX_SPD
    MST_ID: typing.ClassVar[MotorVariable]  # value = MotorVariable.MST_ID
    NPP: typing.ClassVar[MotorVariable]  # value = MotorVariable.NPP
    OC_Value: typing.ClassVar[MotorVariable]  # value = MotorVariable.OC_Value
    OT_Value: typing.ClassVar[MotorVariable]  # value = MotorVariable.OT_Value
    OV_Value: typing.ClassVar[MotorVariable]  # value = MotorVariable.OV_Value
    PMAX: typing.ClassVar[MotorVariable]  # value = MotorVariable.PMAX
    Rs: typing.ClassVar[MotorVariable]  # value = MotorVariable.Rs
    SN: typing.ClassVar[MotorVariable]  # value = MotorVariable.SN
    TIMEOUT: typing.ClassVar[MotorVariable]  # value = MotorVariable.TIMEOUT
    TMAX: typing.ClassVar[MotorVariable]  # value = MotorVariable.TMAX
    UV_Value: typing.ClassVar[MotorVariable]  # value = MotorVariable.UV_Value
    VL_c1: typing.ClassVar[MotorVariable]  # value = MotorVariable.VL_c1
    VMAX: typing.ClassVar[MotorVariable]  # value = MotorVariable.VMAX
    V_BW: typing.ClassVar[MotorVariable]  # value = MotorVariable.V_BW
    can_br: typing.ClassVar[MotorVariable]  # value = MotorVariable.can_br
    dir: typing.ClassVar[MotorVariable]  # value = MotorVariable.dir
    hw_ver: typing.ClassVar[MotorVariable]  # value = MotorVariable.hw_ver
    k1: typing.ClassVar[MotorVariable]  # value = MotorVariable.k1
    k2: typing.ClassVar[MotorVariable]  # value = MotorVariable.k2
    m_off: typing.ClassVar[MotorVariable]  # value = MotorVariable.m_off
    p_m: typing.ClassVar[MotorVariable]  # value = MotorVariable.p_m
    sub_ver: typing.ClassVar[MotorVariable]  # value = MotorVariable.sub_ver
    sw_ver: typing.ClassVar[MotorVariable]  # value = MotorVariable.sw_ver
    u_off: typing.ClassVar[MotorVariable]  # value = MotorVariable.u_off
    v_off: typing.ClassVar[MotorVariable]  # value = MotorVariable.v_off
    xout: typing.ClassVar[MotorVariable]  # value = MotorVariable.xout
class OpenArm:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self, can_interface: str, enable_fd: bool = False) -> None:
        ...
    def disable_all(self) -> None:
        ...
    def enable_all(self) -> None:
        ...
    def flush_rx(self) -> int:
        ...
    def get_arm(self) -> ArmComponent:
        ...
    def get_gripper(self) -> GripperComponent:
        ...
    def get_master_can_device_collection(self) -> CANDeviceCollection:
        ...
    def init_arm_motors(self, motor_types: collections.abc.Sequence[MotorType], send_can_ids: collections.abc.Sequence[int], recv_can_ids: collections.abc.Sequence[int], control_modes: collections.abc.Sequence[ControlMode] = []) -> None:
        ...
    def init_gripper_motor(self, motor_type: MotorType, send_can_id: int, recv_can_id: int, control_mode: ControlMode = ...) -> None:
        ...
    def query_param_all(self, rid: int) -> None:
        ...
    def recv_all(self, first_timeout_us: int = 500) -> None:
        ...
    def refresh_all(self) -> None:
        ...
    def refresh_all_and_recv(self, timeout_us: int = 500) -> int:
        ...
    def set_callback_mode_all(self, callback_mode: CallbackMode) -> None:
        ...
    def set_zero_all(self) -> None:
        ...
class ParamResult:
    rid: int
    valid: bool
    value: float
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    def __init__(self) -> None:
        ...
class PosForceParam:
    dq: float
    i: float
    q: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, q: float, dq: float, i: float) -> None:
        ...
class PosVelParam:
    dq: float
    q: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, q: float, dq: float) -> None:
        ...
ACC: MotorVariable  # value = MotorVariable.ACC
COUNT: MotorVariable  # value = MotorVariable.COUNT
CTRL_MODE: MotorVariable  # value = MotorVariable.CTRL_MODE
DEC: MotorVariable  # value = MotorVariable.DEC
DM10010: MotorType  # value = MotorType.DM10010
DM10010L: MotorType  # value = MotorType.DM10010L
DM3507: MotorType  # value = MotorType.DM3507
DM4310: MotorType  # value = MotorType.DM4310
DM4310_48V: MotorType  # value = MotorType.DM4310_48V
DM4340: MotorType  # value = MotorType.DM4340
DM4340_48V: MotorType  # value = MotorType.DM4340_48V
DM6006: MotorType  # value = MotorType.DM6006
DM8006: MotorType  # value = MotorType.DM8006
DM8009: MotorType  # value = MotorType.DM8009
DMG6220: MotorType  # value = MotorType.DMG6220
DMH3510: MotorType  # value = MotorType.DMH3510
DMH6215: MotorType  # value = MotorType.DMH6215
Damp: MotorVariable  # value = MotorVariable.Damp
Deta: MotorVariable  # value = MotorVariable.Deta
ESC_ID: MotorVariable  # value = MotorVariable.ESC_ID
Flux: MotorVariable  # value = MotorVariable.Flux
GREF: MotorVariable  # value = MotorVariable.GREF
Gr: MotorVariable  # value = MotorVariable.Gr
IGNORE: CallbackMode  # value = CallbackMode.IGNORE
IQ_c1: MotorVariable  # value = MotorVariable.IQ_c1
I_BW: MotorVariable  # value = MotorVariable.I_BW
Inertia: MotorVariable  # value = MotorVariable.Inertia
KI_APR: MotorVariable  # value = MotorVariable.KI_APR
KI_ASR: MotorVariable  # value = MotorVariable.KI_ASR
KP_APR: MotorVariable  # value = MotorVariable.KP_APR
KP_ASR: MotorVariable  # value = MotorVariable.KP_ASR
KT_Value: MotorVariable  # value = MotorVariable.KT_Value
LS: MotorVariable  # value = MotorVariable.LS
MAX_SPD: MotorVariable  # value = MotorVariable.MAX_SPD
MIT: ControlMode  # value = ControlMode.MIT
MST_ID: MotorVariable  # value = MotorVariable.MST_ID
NPP: MotorVariable  # value = MotorVariable.NPP
OC_Value: MotorVariable  # value = MotorVariable.OC_Value
OT_Value: MotorVariable  # value = MotorVariable.OT_Value
OV_Value: MotorVariable  # value = MotorVariable.OV_Value
PARAM: CallbackMode  # value = CallbackMode.PARAM
PMAX: MotorVariable  # value = MotorVariable.PMAX
POS_FORCE: ControlMode  # value = ControlMode.POS_FORCE
POS_VEL: ControlMode  # value = ControlMode.POS_VEL
Rs: MotorVariable  # value = MotorVariable.Rs
SN: MotorVariable  # value = MotorVariable.SN
STATE: CallbackMode  # value = CallbackMode.STATE
TIMEOUT: MotorVariable  # value = MotorVariable.TIMEOUT
TMAX: MotorVariable  # value = MotorVariable.TMAX
UV_Value: MotorVariable  # value = MotorVariable.UV_Value
VEL: ControlMode  # value = ControlMode.VEL
VL_c1: MotorVariable  # value = MotorVariable.VL_c1
VMAX: MotorVariable  # value = MotorVariable.VMAX
V_BW: MotorVariable  # value = MotorVariable.V_BW
can_br: MotorVariable  # value = MotorVariable.can_br
dir: MotorVariable  # value = MotorVariable.dir
hw_ver: MotorVariable  # value = MotorVariable.hw_ver
k1: MotorVariable  # value = MotorVariable.k1
k2: MotorVariable  # value = MotorVariable.k2
m_off: MotorVariable  # value = MotorVariable.m_off
p_m: MotorVariable  # value = MotorVariable.p_m
sub_ver: MotorVariable  # value = MotorVariable.sub_ver
sw_ver: MotorVariable  # value = MotorVariable.sw_ver
u_off: MotorVariable  # value = MotorVariable.u_off
v_off: MotorVariable  # value = MotorVariable.v_off
xout: MotorVariable  # value = MotorVariable.xout
