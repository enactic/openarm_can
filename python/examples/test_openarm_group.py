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
Example: refresh multiple CAN buses in parallel with OpenArmGroup.

This example creates one OpenArm instance per CAN interface through OpenArmGroup,
initializes motors on each bus, then calls group.refresh_all_and_recv() to refresh
all buses in parallel.

Edit BUS_CONFIGS for your own CAN interfaces and motor IDs before running.
"""

import time

import openarm_can as oa


# Edit this table for your setup.
#
# send_ids are motor command CAN IDs.
# recv_ids are motor feedback CAN IDs.
BUS_CONFIGS = {
    "can0": {
        "motor_types": [oa.MotorType.DM4310],
        "send_ids": [0x01],
        "recv_ids": [0x11],
        "control_modes": [oa.ControlMode.MIT],
    },
    "can1": {
        "motor_types": [oa.MotorType.DM4310],
        "send_ids": [0x02],
        "recv_ids": [0x12],
        "control_modes": [oa.ControlMode.MIT],
    },
}


def main() -> None:
    can_interfaces = list(BUS_CONFIGS)

    # True means CAN-FD enabled.
    group = oa.OpenArmGroup(can_interfaces, True)

    for can_interface, config in BUS_CONFIGS.items():
        arm = group.get_openarm(can_interface)

        arm.init_arm_motors(
            config["motor_types"],
            config["send_ids"],
            config["recv_ids"],
            config["control_modes"],
        )

        arm.set_callback_mode_all(oa.CallbackMode.STATE)

        print(
            f"{can_interface}: expected responses = "
            f"{arm.expected_response_count()}"
        )

    try:
        group.enable_all()

        for step in range(30):
            results = group.refresh_all_and_recv(timeout_us=500)

            print(f"\nstep {step}")

            for result in results:
                status = "OK" if result.ok else "MISS"
                print(
                    f"{result.interface}: "
                    f"{result.received}/{result.expected} "
                    f"{status}"
                )

                if result.error:
                    print(f"  error: {result.error}")

            for can_interface in can_interfaces:
                arm = group.get_openarm(can_interface)

                for i, motor in enumerate(arm.get_arm().get_motors()):
                    print(
                        f"  {can_interface} motor[{i}] "
                        f"q={motor.get_position(): .6f} rad "
                        f"dq={motor.get_velocity(): .6f} rad/s "
                        f"tau={motor.get_torque(): .6f} Nm"
                    )

            time.sleep(0.1)

    finally:
        group.disable_all()


if __name__ == "__main__":
    main()