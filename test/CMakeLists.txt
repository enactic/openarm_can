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

find_package(Python3 COMPONENTS Interpreter)
if(NOT Python3_Interpreter_FOUND)
  message(STATUS "CLI tests are disabled: Python 3 is not found")
  return()
endif()
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import pytest"
  RESULT_VARIABLE OPENARM_CAN_PYTEST_RESULT
  OUTPUT_QUIET ERROR_QUIET)
if(NOT OPENARM_CAN_PYTEST_RESULT EQUAL 0)
  message(STATUS "CLI tests are disabled: pytest is not found")
  return()
endif()

add_test(
  NAME cli
  COMMAND
    ${CMAKE_CURRENT_SOURCE_DIR}/with-vcan.sh ${Python3_EXECUTABLE} -m pytest
    # Don't write .pytest_cache/ into the source directory.
    -p no:cacheprovider --verbose ${CMAKE_CURRENT_SOURCE_DIR}/cli)
set_tests_properties(
  cli
  PROPERTIES
    # Don't write __pycache__/ into the source directory.
    ENVIRONMENT
    "OPENARM_CAN_CLI=$<TARGET_FILE:openarm-can-cli>;PYTHONDONTWRITEBYTECODE=1"
    SKIP_RETURN_CODE 77 TIMEOUT 120)
