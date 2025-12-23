#!/bin/bash
#
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

set -exu

apt update
apt install -V -y curl lsb-release

distribution=$(lsb_release --short --id | tr '[:upper:]' '[:lower:]')
code_name=$(lsb_release --codename --short)
architecture=$(dpkg --print-architecture)

repositories_dir=/host/packages/apt/repositories
apt install -V -y \
    "${repositories_dir}"/"${distribution}"/pool/"${code_name}"/*/*/*/*_"${architecture}".deb

openarm-can-change-baudrate -h
openarm-can-configure-socketcan -h
openarm-can-configure-socketcan-4-arms -h
openarm-can-diagnosis -h
openarm-can-motor-check -h
openarm-can-set-zero -h

python3 -c "import openarm_can"
