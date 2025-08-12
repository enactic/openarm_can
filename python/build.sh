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
#
# Build script for OpenArm Python bindings

set -eu

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Building OpenArm Python bindings..."
echo "Script directory: $SCRIPT_DIR"
echo "Project root: $PROJECT_ROOT"

# Check if we're in a virtual environment
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    echo "Using virtual environment (venv): $VIRTUAL_ENV"
elif [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; then
    echo "Using conda environment: $CONDA_DEFAULT_ENV"
else
    echo "Warning: Not in a virtual environment. Consider using:"
    echo "  python -m venv venv && source venv/bin/activate"
    echo "  # or"
    echo "  conda create -n myenv python=3.x && conda activate myenv"
fi

# Build the C++ library first if needed
if [ ! -d "$PROJECT_ROOT/build" ]; then
    echo "Building C++ library..."
    cmake \
        -S "$PROJECT_ROOT" \
        -B "$PROJECT_ROOT/build" \
        -DCMAKE_BUILD_TYPE=Debug \
        -GNinja
    cmake --build "$PROJECT_ROOT/build"
    cmake --install "$PROJECT_ROOT/build"
fi

# Install in development mode
echo "Installing in development mode..."
pip install .

echo "Build completed successfully!"
echo ""
echo "To test the installation, run:"
echo "  python -c 'import openarm; print(openarm.__version__)'"
echo ""
echo "See examples/ directory for usage examples."
