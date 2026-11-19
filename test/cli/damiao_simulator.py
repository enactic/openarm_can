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

"""Fake Damiao motors on a SocketCAN interface.

Only the parameter protocol on the 0x7FF config ID and the disable
command are emulated, as far as Damiao documents them:

* 0x33 reads, 0x55 writes and 0xAA saves a register. Reads and
  writes are answered with the request echoed back, carrying the
  value; the save with 0xAA 0x01.
* Replies are sent with the motor's master ID as the CAN ID.
* A write to ESC_ID or MST_ID takes effect right after its reply.
* Saving is refused while the motor is enabled.

Each motor keeps its registers in RAM and flash separately:
power_cycle() reloads RAM from flash, as a real power cycle does.

What a real motor does beyond this, such as timing, bit rates and
replies that Damiao leaves undocumented, is not emulated: a passing
test does not replace a check on real motors.
"""

import socket
import struct
import threading

CONFIG_ID = 0x7FF
CMD_READ = 0x33
CMD_WRITE = 0x55
CMD_SAVE = 0xAA
RID_MST_ID = 7
RID_ESC_ID = 8

_CAN_FRAME = struct.Struct("=IB3x8s")
_CANFD_MTU = 72
_DISABLE = bytes([0xFF] * 7 + [0xFD])


class Motor:
    def __init__(self, slave_id, master_id):
        self.flash = {RID_ESC_ID: slave_id, RID_MST_ID: master_id}
        self.ram = dict(self.flash)
        self.enabled = True

    @property
    def slave_id(self):
        return self.ram[RID_ESC_ID]

    @property
    def master_id(self):
        return self.ram[RID_MST_ID]

    def power_cycle(self):
        self.ram = dict(self.flash)
        self.enabled = True


class DamiaoSimulator:
    """Serves the motors from a background thread while in a with block."""

    def __init__(self, interface, motors):
        self.motors = motors
        self._lock = threading.Lock()
        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                     socket.CAN_RAW)
        self._socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES,
                                1)
        self._socket.settimeout(0.05)
        self._socket.bind((interface,))
        self._running = False
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self._running = True
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._running = False
        self._thread.join()
        self._socket.close()

    def power_cycle(self):
        with self._lock:
            for motor in self.motors:
                motor.power_cycle()

    def _send(self, can_id, data):
        self._socket.send(_CAN_FRAME.pack(can_id, 8, bytes(data)))

    def _find(self, slave_id):
        return next((m for m in self.motors if m.slave_id == slave_id), None)

    def _serve(self):
        while self._running:
            try:
                frame = self._socket.recv(_CANFD_MTU)
            except socket.timeout:
                continue
            can_id, length = struct.unpack_from("=IB", frame)
            data = frame[8:8 + length]
            if length != 8:
                continue
            with self._lock:
                if can_id == CONFIG_ID:
                    self._config(data)
                elif data == _DISABLE:
                    self._disable(can_id)

    def _config(self, data):
        slave_id = data[0] | (data[1] << 8)
        cmd, rid = data[2], data[3]
        (value,) = struct.unpack_from("<I", data, 4)
        motor = self._find(slave_id)
        if motor is None:
            return
        if cmd == CMD_READ:
            value = motor.ram.get(rid, 0)
            self._send(motor.master_id,
                       data[:4] + struct.pack("<I", value))
        elif cmd == CMD_WRITE:
            master_id = motor.master_id
            motor.ram[rid] = value
            self._send(master_id, data)
        elif cmd == CMD_SAVE:
            if motor.enabled:
                return
            motor.flash = dict(motor.ram)
            self._send(motor.master_id,
                       data[:2] + bytes([CMD_SAVE, 0x01, 0, 0, 0, 0]))

    def _disable(self, slave_id):
        motor = self._find(slave_id)
        if motor is None:
            return
        motor.enabled = False
        # Status reply: the ID in the low nibble, the state (0 = disabled)
        # in the high one, and then position, velocity, torque and
        # temperatures.
        self._send(motor.master_id,
                   [motor.slave_id & 0x0F, 0x80, 0x00, 0x80, 0x08, 0x00,
                    25, 25])
