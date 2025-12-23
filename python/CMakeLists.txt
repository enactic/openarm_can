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

cmake_minimum_required(VERSION 3.22)
project(
  openarm_can_python
  VERSION 1.2.2
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  Python
  COMPONENTS Interpreter Development.Module
  REQUIRED)

include(FetchContent)
set(OCP_FETCH_CONTENT_COMMON_OPTIONS)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.28)
  list(APPEND OCP_FETCH_CONTENT_COMMON_OPTIONS EXCLUDE_FROM_ALL TRUE)
endif()
macro(ocp_prepare_fetchcontent)
  set(BUILD_SHARED_LIBS OFF)
  set(BUILD_TESTING OFF)
  set(CMAKE_COMPILE_WARNING_AS_ERROR FALSE)
  set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY TRUE)
  set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
endmacro()

execute_process(
  COMMAND Python::Interpreter -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT)
function(ocp_ensure_nanobind)
  # We can use FIND_PACKAGE_ARGS in fetchcontent_declare() instead of
  # explicit find_package() when we require CMake 3.24 or later.
  find_package(nanobind)
  if(nanobind_FOUND)
    return()
  endif()

  set(NANOBIND_BUNDLED_VERSION "2.10.2")
  set(NANOBIND_SOURCE_BASE_NAME "nanobind-${NANOBIND_BUNDLED_VERSION}.tar.gz")
  set(NANOBIND_SOURCE_LOCAL_PATH
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/${NANOBIND_SOURCE_BASE_NAME}")
  if(EXISTS ${NANOBIND_SOURCE_LOCAL_PATH})
    set(NANOBIND_SOURCE_ARGS URL ${NANOBIND_SOURCE_LOCAL_PATH})
  else()
    set(NANOBIND_SOURCE_ARGS
        GIT_REPOSITORY "https://github.com/wjakob/nanobind.git" GIT_TAG
        "v${NANOBIND_BUNDLED_VERSION}")
  endif()
  fetchcontent_declare(
    nanobind
    ${OCP_FETCH_CONTENT_COMMON_OPTIONS}
    ${NANOBIND_SOURCE_ARGS}
    # We can use FIND_PACKAGE_ARGS when we require CMake 3.24 or later.
    #
    # FIND_PACKAGE_ARGS
  )
  ocp_prepare_fetchcontent()
  fetchcontent_makeavailable(nanobind)
  if(nanobind_SOURCE_DIR)
    if(CMAKE_VERSION VERSION_LESS 3.28)
      set_property(DIRECTORY "${nanobind_SOURCE_DIR}" PROPERTY EXCLUDE_FROM_ALL
                                                               TRUE)
    endif()
  endif()
endfunction()
ocp_ensure_nanobind()

function(ocp_ensure_openarm_can)
  # We can use FIND_PACKAGE_ARGS in fetchcontent_declare() instead of
  # explicit find_package() when we require CMake 3.24 or later.
  find_package(OpenArmCAN ${openarm_can_python_VERSION})
  if(OpenArmCAN_FOUND)
    return()
  endif()

  fetchcontent_declare(
    openarm_can
    ${OCP_FETCH_CONTENT_COMMON_OPTIONS}
    GIT_REPOSITORY "https://github.com/enactic/openarm_can.git"
    GIT_TAG "${openarm_can_python_VERSION}"
    # We can use FIND_PACKAGE_ARGS when we require CMake 3.24 or later.
    #
    # FIND_PACKAGE_ARGS NAMES OpenArmCAN
  )
  ocp_prepare_fetchcontent()
  fetchcontent_makeavailable(openarm_can)
  if(openarm_can_SOURCE_DIR)
    if(CMAKE_VERSION VERSION_LESS 3.28)
      set_property(DIRECTORY "${openarm_can_SOURCE_DIR}"
                   PROPERTY EXCLUDE_FROM_ALL TRUE)
    endif()
    add_library(OpenArmCAN::openarm_can ALIAS openarm_can)
  endif()
endfunction()
ocp_ensure_openarm_can()

nanobind_add_module(openarm_can_python src/openarm_can.cpp)
set_target_properties(openarm_can_python PROPERTIES OUTPUT_NAME "openarm_can")
target_link_libraries(openarm_can_python PRIVATE OpenArmCAN::openarm_can)
install(TARGETS openarm_can_python LIBRARY DESTINATION "openarm_can")
install(FILES openarm_can/__init__.py DESTINATION "openarm_can")
