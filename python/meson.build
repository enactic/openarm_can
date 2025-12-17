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

project(
    'openarm_can',
    'cpp',
    version: '1.1.0',
    default_options: ['cpp_std=c++17'],
)

py = import('python').find_installation(pure: false)
nanobind_dep = dependency('nanobind')

openarm_can_cpp_dep = dependency(
    'openarm-can',
    'OpenArmCAN',
    modules: ['OpenArmCAN::openarm_can'],
    required: false,
)
if not openarm_can_cpp_dep.found()
    cmake = import('cmake')
    options = cmake.subproject_options()
    options.add_cmake_defines(
        {
            'BUILD_SHARED_LIBS': true,
            # meson-python installs only libopenarm_can.so.1.1.0...
            # libopenarm_can.so.1 isn't installed...
            'OPENARM_CAN_USE_SHARED_OBJECT_VERSION': false,
        },
    )
    if get_option('buildtype') == 'debug'
        options.add_cmake_defines({'CMAKE_BUILD_TYPE': 'Debug'})
    elif get_option('buildtype') == 'release'
        options.add_cmake_defines({'CMAKE_BUILD_TYPE': 'Release'})
    endif
    openarm_can_cpp_subproject = cmake.subproject(
        'openarm-can',
        options: options,
    )
    openarm_can_cpp_dep = openarm_can_cpp_subproject.dependency('openarm_can')
endif

py.install_sources(
    'openarm/can/__init__.py',
    'openarm/can/core.py',
    subdir: 'openarm/can',
)
py.extension_module(
    'openarm_can',
    'src/openarm_can.cpp',
    install: true,
    dependencies: [openarm_can_cpp_dep, nanobind_dep],
)
