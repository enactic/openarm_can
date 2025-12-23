# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
OpenArm CAN Python bindings for motor control via SocketCAN.

This package provides Python bindings for the OpenArm motor control system,
allowing you to control DAMIAO motors through SocketCAN.
"""

from .openarm_can import *

__version__ = "1.2.2"
__author__ = "Enactic, Inc."

# Direct export of C++ classes - no wrappers
__all__ = [
    # Enums
    "MotorType",
    "MotorVariable",
    "CallbackMode",

    # Data structures
    "LimitParam",
    "ParamResult",
    "MotorStateResult",
    "CanFrame",
    "CanFdFrame",
    "MITParam",

    # Main C++ classes (1:1 mapping)
    "Motor",
    "MotorControl",
    "CANSocket",           # Low-level socket with file descriptor access
    "CANDevice",           # Base CAN device class
    "MotorDeviceCan",      # Motor device management
    "CANDeviceCollection",  # Device collection management

    # Exceptions
    "CANSocketException",
]
