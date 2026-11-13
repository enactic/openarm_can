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

import re

from damiao_simulator import DamiaoSimulator, Motor


def test_id_order(interface, run_cli):
    motors = [Motor(0x01, 0x11), Motor(0x08, 0x18)]
    with DamiaoSimulator(interface, motors):
        # Not in recv ID order.
        result = run_cli("show_param", "--id", "0x08,0x01")
    assert result.returncode == 0, result.stdout
    # The ID in each block's header and the ESC_ID register shown in it.
    blocks = re.findall(r"^ MOTOR ID: (0x[0-9a-f]+) .*?"
                        r"^Motor \(ESC\) ID[^\n]* (\d+)$",
                        result.stdout, re.MULTILINE | re.DOTALL)
    assert blocks == [("0x8", "8"), ("0x1", "1")], result.stdout
