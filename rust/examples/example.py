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

import time

import openarm_can as oa

# Create OpenArm instance

arm = oa.OpenArm("can0", True)

# Initialize arm motors
motor_types = [oa.MotorType.DM4310, oa.MotorType.DM4310]
send_ids = [0x01, 0x02]
recv_ids = [0x11, 0x12]
arm.init_arm_motors(motor_types, send_ids, recv_ids)

# Initialize gripper
arm.init_gripper_motor(oa.MotorType.DM4310, 0x7, 0x17)
arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
# Use high-level operations
arm.enable_all()
arm.recv_all()


# return to zero position
arm.set_callback_mode_all(oa.CallbackMode.STATE)
arm.get_arm().mit_control_all(
    [oa.MITParam(2, 0.5, 0, 0, 0), oa.MITParam(2, 0.5, 0, 0, 0)]
)

arm.recv_all()

# torque control test

arm.get_gripper().mit_control_all([oa.MITParam(0, 0, 0, 0, 0.15)])
arm.get_arm().mit_control_all(
    [oa.MITParam(0, 0, 0, 0, 0.15), oa.MITParam(0, 0, 0, 0, 0.15)]
)
arm.recv_all()

arm.disable_all()
# read motor position
while True:
    arm.refresh_all()
    arm.recv_all()
    for motor in arm.get_arm().get_motors():
        print(motor.get_position())
    for motor in arm.get_gripper().get_motors():
        print(motor.get_position())
