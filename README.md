# OpenArm CAN Library

A C++ library for CAN communication with OpenArm robotic hardware, supporting Damiao motors over CAN/CAN-FD interfaces.
This library is a part of [OpenArm](https://github.com/enactic/openarm/). See detailed setup guide and docs [here](https://docs.openarm.dev/software/can).


## Quick Start

### Prerequisites

- Linux with SocketCAN support
- CAN interface hardware

### 1. Install

#### Ubuntu

* 22.04 Jammy Jellyfish
* 24.04 Noble Numbat

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y \
  libopenarm-can-dev \
  openarm-can-utils
```

#### AlmaLinux, CentOS, Fedora, RHEL, and Rocky Linux

1. Enable [EPEL](https://docs.fedoraproject.org/en-US/epel/). (Not required for [Fedora](https://fedoraproject.org/))
   * AlmaLinux 8 / Rocky Linux 8
     ```bash
     sudo dnf install -y epel-release
     sudo dnf config-manager --set-enabled powertools
     ```
   * AlmaLinux 9 & 10 / Rocky Linux 9 & 10
     ```bash
     sudo dnf install -y epel-release
     sudo crb enable
     ```
   * CentOS Stream 9
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel{,-next}-release-latest-9.noarch.rpm
     ```
   * CentOS Stream 10
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-10.noarch.rpm
     ```
   * RHEL 8 & 9 & 10
     ```bash
     releasever="$(. /etc/os-release && echo $VERSION_ID | grep -oE '^[0-9]+')"
     sudo subscription-manager repos --enable codeready-builder-for-rhel-$releasever-$(arch)-rpms
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-$releasever.noarch.rpm
     ```
2. Install the package.
   ```bash
   sudo dnf update
   sudo dnf install -y \
     openarm-can-devel \
     openarm-can-utils
   ```

### 2. Setup CAN Interface

Configure your CAN interface using the provided script:

```bash
# CAN 2.0 (default)
/usr/libexec/openarm-can/configure_socketcan.sh can0

# CAN-FD with 5Mbps data rate
/usr/libexec/openarm-can/configure_socketcan.sh can0 -fd
```

### 3. C++ Library

```cpp
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

openarm::can::socket::OpenArm arm("can0", true);  // CAN-FD enabled
std::vector<openarm::damiao_motor::MotorType> motor_types = {
    openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};
std::vector<uint32_t> send_can_ids = {0x01, 0x02};
std::vector<uint32_t> recv_can_ids = {0x11, 0x12};

openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);
openarm.enable_all();
```

See [dev/README.md](dev/README.md) for how to build.

### 4. Python (🚧 EXPERIMENTAL - TEMPORARY 🚧)

> ⚠️ **WARNING: UNSTABLE API** ⚠️
> Python bindings are currently a direct low level **temporary port**, and will change **DRASTICALLY**.
> The interface is may break between versions.Use at your own risk! Discussions on the interface are welcomed.

**Build & Install:**

Please ensure that you install the C++ library first, as `1. Install` or [dev/README.md](dev/README.md).
```bash
cd python

# Create and activate virtual environment (recommended)
python -m venv venv
source venv/bin/activate

./build.sh
```

**Usage:**

```python
# WARNING: This API is unstable and will change!
import openarm_can as oa

arm = oa.OpenArm("can0", True)  # CAN-FD enabled
arm.init_arm_motors([oa.MotorType.DM4310], [0x01], [0x11])
arm.enable_all()
```

### Examples

- **C++**: `examples/demo.cpp` - Complete arm control demo
- **Python**: `python/examples/example.py` - Basic Python usage

## For developers

See [dev/README.md](dev/README.md).

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/can/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See `LICENSE.txt` for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
