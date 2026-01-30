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


def main() -> None:
    arm = oa.OpenArm("can1", True)
    arm.init_arm_motors([oa.MotorType.DM4310], [0x4], [0x14],
                        [oa.ControlMode.MIT])
    arm.init_gripper_motor(oa.MotorType.DM4310, 0x8,
                           0x18, oa.ControlMode.POS_FORCE)

    arm.set_callback_mode_all(oa.CallbackMode.PARAM)
    arm.enable_all()
    arm.recv_all()

    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    gripper = arm.get_gripper()
    armaa = arm.get_arm()
    time.sleep(0.2)

    # pos, rad/s limit, torque limit
    sequence = [
        (3.14/2.0, 25.0, 1.5),
        (0.0, 25.0, 1.5),
        (3.14/2.0, 25.0, 1.5),
        (0.0, 25.0, 1.5),
    ]

    for position, speed, torque in sequence:
        print(f"set_position({position}) speed={speed} torque={torque}")

        gripper.set_position(position, speed_rad_s=speed, torque_pu=torque/10)
        armaa.mit_control_all([oa.MITParam(0, 0, 0, 0, 0)])
        for _ in range(6):
            arm.refresh_all()
            arm.recv_all(500)
            for motor in gripper.get_motors():
                print("gripper position:", motor.get_position())
            for motor in arm.get_arm().get_motors():
                print(f"arm position:", motor.get_position())
            time.sleep(0.05)

    arm.disable_all()
    arm.recv_all()


if __name__ == "__main__":
    main()
