"""

OpenArm CAN Python bindings for motor control via SocketCAN.

This package provides Python bindings for the OpenArm motor control system,
allowing you to control DAMIAO motors through SocketCAN.
"""
from __future__ import annotations
from openarm_can.openarm_can import ArmComponent
from openarm_can.openarm_can import CANDevice
from openarm_can.openarm_can import CANDeviceCollection
from openarm_can.openarm_can import CANPacket
from openarm_can.openarm_can import CANSocket
from openarm_can.openarm_can import CANSocketException
from openarm_can.openarm_can import CallbackMode
from openarm_can.openarm_can import CanFdFrame
from openarm_can.openarm_can import CanFrame
from openarm_can.openarm_can import CanPacketDecoder
from openarm_can.openarm_can import CanPacketEncoder
from openarm_can.openarm_can import ControlMode
from openarm_can.openarm_can import DMDeviceCollection
from openarm_can.openarm_can import GripperComponent
from openarm_can.openarm_can import LimitParam
from openarm_can.openarm_can import MITParam
from openarm_can.openarm_can import Motor
from openarm_can.openarm_can import MotorDeviceCan
from openarm_can.openarm_can import MotorStateResult
from openarm_can.openarm_can import MotorType
from openarm_can.openarm_can import MotorVariable
from openarm_can.openarm_can import OpenArm
from openarm_can.openarm_can import ParamResult
from openarm_can.openarm_can import PosForceParam
from openarm_can.openarm_can import PosVelParam
from openarm_can.openarm_can import VelParam
from . import openarm_can
__all__: list[str] = [
    "MotorType",
    "MotorVariable",
    "CallbackMode",
    "ControlMode",
    "LimitParam",
    "ParamResult",
    "MotorStateResult",
    "CANPacket",
    "CanFrame",
    "CanFdFrame",
    "MITParam",
    "PosVelParam",
    "VelParam",
    "PosForceParam",
    "CanPacketEncoder",
    "CanPacketDecoder",
    "Motor",
    "CANSocket",
    "CANDevice",
    "MotorDeviceCan",
    "CANDeviceCollection",
    "DMDeviceCollection",
    "ArmComponent",
    "GripperComponent",
    "OpenArm",
    "CANSocketException",
]
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
__author__: str = 'Enactic, Inc.'
__version__: str = '1.2.9'
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
