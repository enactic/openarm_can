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

# If you can use build, you should use pyproject.toml intead of
# setup.py. setup.py is only for old environment such as deb packages
# for Ubuntu 22.04.

import pathlib

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class cmake_build_ext(build_ext):
    def build_extensions(self):
        self.extensions = [
            ext for ext in self.extensions if ext.name != '__dummy__']
        super().run()

    def run(self):
        self._run_cmake()
        super().run()

    def _run_cmake(self):
        source = pathlib.Path().absolute()
        build_lib = pathlib.Path(self.build_lib).absolute()
        build_temp = pathlib.Path(self.build_temp).absolute()
        build_temp.mkdir(parents=True, exist_ok=True)
        self.spawn([
            "cmake",
            "-S", str(source),
            "-B", str(build_temp),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DCMAKE_INSTALL_PREFIX={build_lib}",
        ])
        self.spawn(["cmake", "--build", str(build_temp)])
        self.spawn(["cmake", "--install", str(build_temp)])


setup(
    name="openarm_can",
    version="1.2.2",
    license="Apache-2.0",
    license_files=["LICENSE.txt"],
    ext_modules=[Extension('__dummy__', [])],
    cmdclass={
        "build_ext": cmake_build_ext,
    }
)
