# OpenArm CAN Python Fork

> [!NOTE]
> This repository is a Python-focused fork of [`enactic/openarm_can`](https://github.com/enactic/openarm_can).
>
> The intended usage of this fork is to install and use the Python package directly from this Git repository.
>
> No separate `openarm-can` / `libopenarm-can-dev` system package is required when installing this fork from Git. During a Git-based Python install, the extension module is built against the C++ source tree included in this repository, so the Python bindings and the C++ core stay in sync.

This fork keeps the original OpenArm CAN / SocketCAN C++ structure, while adding Python-focused improvements for experimentation with OpenArm / DaMiao motors.

## What This Fork Adds

- Python package installation from the `python/` subdirectory through Git URLs.
- Python type stubs (`.pyi`) and `py.typed` for IDE completion and static analysis.
- Improved package-level Python exports such as `OpenArm`, `ArmComponent`, `GripperComponent`, and `DMDeviceCollection`.
- `flush_rx()` and `refresh_all_and_recv()` helpers for more reliable state refresh from Python.
- Pure velocity control support through `ControlMode.VEL`, `VelParam`, and `vel_control_one/all`.
- Additional Python examples for low-level motor-control usage.

---

## Requirements

This package is intended for Linux systems with SocketCAN support.

You need:

- Linux with SocketCAN support
- A working CAN or CAN-FD adapter
- Python 3.10+
- A C++17-capable compiler
- CMake / Ninja or an equivalent build backend

On Ubuntu, a typical setup is:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  python3-dev \
  python3-venv \
  can-utils
```

`can-utils` is optional but useful for debugging with tools such as `candump`, `cansend`, and `ip link`.

---

## Install

### Install with pip

```bash
pip install "git+https://github.com/umeow0716/openarm_can.git@main#subdirectory=python"
```

### Install with uv

```bash
uv add "git+https://github.com/umeow0716/openarm_can.git@main#subdirectory=python"
```

### Local editable install

```bash
git clone https://github.com/umeow0716/openarm_can.git
cd openarm_can/python
pip install -e .
```

Or with `uv`:

```bash
git clone https://github.com/umeow0716/openarm_can.git
cd openarm_can/python
uv pip install -e .
```

The Python package builds a native extension module. When installed from this repository, the build uses the C++ source tree included in this repo.

---

## Setup SocketCAN

### Classic CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### CAN-FD

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

Check the interface:

```bash
ip -details link show can0
```

Monitor frames:

```bash
candump can0
```

---

## Basic Python Usage

```python
import openarm_can as oa

arm = oa.OpenArm("can0", True)  # True = CAN-FD enabled

arm.init_arm_motors(
    [oa.MotorType.DM4310],
    [0x01],  # send CAN ID
    [0x11],  # receive CAN ID
    [oa.ControlMode.MIT],
)

arm.enable_all()

received = arm.refresh_all_and_recv()
print("received frames:", received)

for motor in arm.get_arm().get_motors():
    print("position:", motor.get_position())
    print("velocity:", motor.get_velocity())
    print("torque:", motor.get_torque())

arm.disable_all()
```

---

## Development

Clone the repository:

```bash
git clone https://github.com/umeow0716/openarm_can.git
cd openarm_can
```

Install the Python package locally:

```bash
cd python
pip install -e .
```

For C++ development, see:

```text
dev/README.md
```

---

## Relationship to Upstream

This repository is based on:

```text
enactic/openarm_can
```

Original project:

- Repository: <https://github.com/enactic/openarm_can>
- OpenArm main repository: <https://github.com/enactic/openarm>
- Documentation: <https://docs.openarm.dev/software/can>

This fork is not intended to replace the upstream project. It is mainly a Python-first fork for experimenting with OpenArm / DaMiao motor control from Python.

---

## License

Licensed under the Apache License 2.0. See `LICENSE.txt` for details.

Copyright 2025 Enactic, Inc.
