#!/bin/bash
#
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

# Runs COMMAND with OPENARM_CAN_TEST_INTERFACE set to a virtual CAN
# interface that nothing else uses.
#
# Usage: with-vcan.sh COMMAND [ARG...]
#
# If OPENARM_CAN_TEST_INTERFACE is already set, it is used as is.
# Otherwise vcan0 is created in a new network namespace, which needs
# unprivileged user namespaces and the vcan kernel module. Exits with
# 77, which CTest treats as skipped, when neither is available.

set -eu

skip() {
    echo "Skipped: $1" >&2
    exit 77
}

if [ -n "${OPENARM_CAN_TEST_INTERFACE:-}" ]; then
    exec "$@"
fi

if [ "${1:-}" != "--in-namespace" ]; then
    if ! type ip >/dev/null 2>&1; then
        skip "ip (iproute2) is not installed"
    fi
    if ! unshare --user --map-root-user --net true 2>/dev/null; then
        skip "unprivileged user namespaces are not available; set OPENARM_CAN_TEST_INTERFACE"
    fi
    exec unshare --user --map-root-user --net "$0" --in-namespace "$@"
fi
shift

if ! ip link add dev vcan0 type vcan 2>/dev/null; then
    skip "vcan0 cannot be created: is the vcan kernel module loaded?"
fi
ip link set dev vcan0 up
export OPENARM_CAN_TEST_INTERFACE=vcan0
exec "$@"
