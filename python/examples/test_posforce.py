# Copyright 2026 Enactic, Inc.
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
control_modes = [oa.ControlMode.POS_FORCE]
arm.init_arm_motors(motor_types, send_ids, recv_ids, control_modes)

# Enable motors
arm.enable_all()
arm.recv_all()

# PosForceParam(q, dq, i)
#   q   : target position (rad)
#   dq  : max velocity (rad/s)
#   i   : current limit (A)
arm.set_callback_mode_all(oa.CallbackMode.STATE)
arm.get_arm().posforce_control_all([oa.PosForceParam(0.0, 0.0, 0.0)])
arm.recv_all()

# Read motor position every 0.1s for 30 iterations
for _ in range(30):
    arm.refresh_all()
    arm.recv_all()
    for motor in arm.get_arm().get_motors():
        print(motor.get_position())
    time.sleep(0.1)

arm.disable_all()
arm.recv_all()
