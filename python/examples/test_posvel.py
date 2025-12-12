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

import openarm_can as oa
import time
# Create OpenArm instance

arm = oa.OpenArm("can0", True)

# Initialize arm motors
motor_types = [oa.MotorType.DM4310]
send_ids = [0x0A]
recv_ids = [0x1A]
arm.init_arm_motors(motor_types, send_ids, recv_ids)

# Use high-level operations
arm.enable_all()
arm.recv_all()

# return to zero position
arm.set_callback_mode_all(oa.CallbackMode.STATE)
arm.get_arm().posvel_control_all([oa.PosVelParam(3.14 * 4, 20)])

arm.recv_all()

# read motor position
while True:
    arm.refresh_all()
    arm.recv_all()
    for motor in arm.get_arm().get_motors():
        print(motor.get_position())
    for motor in arm.get_gripper().get_motors():
        print(motor.get_position())
