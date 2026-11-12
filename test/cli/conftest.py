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

# OPENARM_CAN_CLI is the openarm-can-cli to test and
# OPENARM_CAN_TEST_INTERFACE is a virtual CAN interface the tests can
# use exclusively. test/with-vcan.sh sets up the latter.

import os
import subprocess

import pytest


@pytest.fixture
def interface():
    interface = os.environ.get("OPENARM_CAN_TEST_INTERFACE")
    if not interface:
        pytest.skip("OPENARM_CAN_TEST_INTERFACE is not set")
    return interface


@pytest.fixture
def run_cli(interface):
    """Runs openarm-can-cli on the test interface."""
    cli = os.environ.get("OPENARM_CAN_CLI", "openarm-can-cli")

    def run(*args):
        return subprocess.run([cli, "-i", interface, *args],
                              stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT,
                              text=True,
                              timeout=30)
    return run
