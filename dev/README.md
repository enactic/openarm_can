# Development

## How to build C++ Library

### Prerequisites

- CMake 3.22+
- C++17 compiler

### Build

```bash
cd openarm_can
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

### Test

```bash
ctest --test-dir build --output-on-failure
```

The CLI tests in `test/cli/` run `openarm-can-cli` against fake
Damiao motors (`test/cli/damiao_simulator.py`) on a virtual CAN
interface. They need:

- Python 3 with pytest: `sudo apt install python3-pytest`
- `ip`: `sudo apt install iproute2`
- The `vcan` kernel module: `sudo modprobe vcan`

CMake disables them when pytest is not found.

`test/with-vcan.sh` creates `vcan0` in a new network namespace, so no
root privilege is needed. If unprivileged user namespaces are not
available, as on Ubuntu 24.04 by default, create an interface and pass
it instead:

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
OPENARM_CAN_TEST_INTERFACE=vcan0 ctest --test-dir build --output-on-failure
```

The tests are skipped when no interface is available.

The simulator follows Damiao's documentation and does not emulate
timing or undocumented behavior, so changes to how the CLI talks to
motors still need a check on real motors.

## How to release

```bash
export LAUNCHPAD_UPLOADER_PGP_KEY=YOUR_PGP_KEY # e.g. export  LAUNCHPAD_UPLOADER_PGP_KEY=08D3564B7C6A9CAFBFF6A66791D18FCF079F8007
git clone https://github.com/apache/arrow.git
export APACHE_ARROW_REPOSITORY=${PWD}/arrow
git clone https://github.com/groonga/groonga.git
export GROONGA_REPOSITORY=${PWD}/groonga
git clone git@github.com:enactic/openarm_can.git
cd openarm_can
rake release NEW_VERSION=X.Y.Z # e.g. rake release 1.0.0
```
