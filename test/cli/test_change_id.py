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

from damiao_simulator import DamiaoSimulator, Motor


def test_save(interface, run_cli):
    motor = Motor(0x08, 0x18)
    with DamiaoSimulator(interface, [motor]) as simulator:
        result = run_cli("change_id", "-c", "0x08", "-s", "0x06", "-m",
                         "0x16", "--save")
        assert result.returncode == 0, result.stdout
        simulator.power_cycle()
    assert (motor.slave_id, motor.master_id) == (0x06, 0x16)


def test_no_save(interface, run_cli):
    motor = Motor(0x08, 0x18)
    with DamiaoSimulator(interface, [motor]) as simulator:
        result = run_cli("change_id", "-c", "0x08", "-s", "0x06", "-m",
                         "0x16")
        assert result.returncode == 0, result.stdout
        assert (motor.slave_id, motor.master_id) == (0x06, 0x16)
        simulator.power_cycle()
    assert (motor.slave_id, motor.master_id) == (0x08, 0x18)
