# Damiao motor behavior observed on real motors

What real Damiao motors did when checked on 2026-11-27, as reference for
the simulators (`damiao_simulator.py` here and in openarm_teleop).
Frames were sent with `cansend`, `openarm-can-cli` or a small Python
script and watched with `candump -td -x`. Statements are what was
observed; inferences are marked as such. Where a simulator does not
match yet, it is listed in
[Differences from the simulators](#differences-from-the-simulators).

## Setup

* 8 motors of one OpenArm on one bus, slave IDs 0x01-0x08 and master IDs
  0x11-0x18:

  | Slave ID | Motor | Gr | PMAX | VMAX | TMAX |
  |---|---|---|---|---|---|
  | 0x01, 0x02 | DM-J8009P-2EC (1) | 9 | 12.5 | 45 | 54 |
  | 0x03, 0x04 | DM-J4340-2EC (2) | 40 | 12.5 | 10 | 28 |
  | 0x05-0x07 | DM-J4310-2EC (2) | 10 | 12.5 | 30 | 10 |
  | 0x08 | DM-J4310-2EC (3) | 10 | 12.5 | 30 | 10 |

  (1) The label is under a cover. Gr 9 and the limits are the 8009's,
  and OpenArm's motor documentation (enactic/openarm
  `website/docs/hardware/openarm-2.0/motor.mdx`) names DM-J8009P-2EC as
  the 8009 model it uses.
  (2) Read on the motors' labels.
  (3) Not read on the label: inferred from the registers, which match
  0x05-0x07.

* Firmware registers, as ASCII of the 4 value bytes in bus order:

  | Slave ID | hw_ver (13) | sw_ver (14) | SN (15) | sub_ver (36) |
  |---|---|---|---|---|
  | 0x01, 0x02 | `300V` | `6417` | `500T` | 52 |
  | 0x03, 0x04 | `300V` | `5117` | `500T` | 52 |
  | 0x05-0x07 | `300V` | `5017` | `500T` | 52 |
  | 0x08 | `300V` | `5015` | `500T` | 54 |

* All motors: can_br = 9 (5 Mbps CAN FD), CTRL_MODE = 1 (MIT),
  TIMEOUT = 0.
* Adapter: PEAK PCAN-USB Pro FD (`pcan_usb_pro_fd`), configured by
  `openarm-can-cli can_configure`: bitrate 1 Mbps (SP 0.75), dbitrate
  5 Mbps (DSP 0.75, DSJW 2), FD on.

## Frames

* Every reply is a CAN FD frame with BRS, whatever the request was.
  Classic CAN requests (parameters, enable, disable, clear error) and
  CAN FD requests with BRS (parameters, enable, disable, MIT) were
  answered with FD + BRS, and so was a parameter read sent as CAN FD
  without BRS.
  * So an interface with FD off receives no reply at all. With can0 at
    1 Mbps and FD off, a read of MST_ID got no reply, and the error
    counters (berr-counter, error-pass, bus-off) didn't change: the
    adapter drops the FD reply silently instead of flagging an error.
    This is why `discover` found nothing at 1 Mbps Classic and all 8
    motors at 5 Mbps FD. (That the motor sent the reply at all couldn't
    be seen with only this adapter, but the same request on an FD
    interface is always answered.)
* Replies are sent with the motor's master ID as the CAN ID.
* Requests to a slave ID no motor has are not answered: `discover`
  found nothing at 0x09-0x10, and an old slave ID stops answering right
  after an ID change.

## Parameters (0x7FF)

Request: `ID_L ID_H CMD RID V0 V1 V2 V3`, value little endian.

### Read (0x33)

* Answered with `ID_L ID_H 33 RID` and the value, 8 bytes:

  ```
  TX 7FF   [8]  01 00 33 07 00 00 00 00
  RX 011  [08]  01 00 33 07 11 00 00 00
  ```

* RIDs that don't exist (37-49, 56-79 and 100 were tried) are answered
  with value 0.
* Value types: RID 7, 8, 9, 10, 16, 35 and 36 are integers, 13, 14 and 15
  are ASCII, and the rest are IEEE 754 floats.
* The very first read of the session (MST_ID) once got no reply; it never
  happened again in later reads.

### Write (0x55)

The reply carries the value the register holds after the write, not the
requested one. There is no other error indication.

| Write | Reply value | Register after |
|---|---|---|
| TIMEOUT (9) = 1234, valid | 1234 | 1234 |
| CTRL_MODE (10) = 9, out of range [0, 4] | 1 (the old value) | 1 |
| OT_Value (2) = 50.0, out of range [80.0, 200) | 100.0 (the old value) | 100.0 |
| hw_ver (13) = 0x12345678, read-only | **0** | unchanged |
| Gr (20) = 1.0, read-only | **0** | unchanged (9.0) |
| RID 100 = 1, doesn't exist | **0** | still reads 0 |

* An out-of-range value is refused and the reply has the old value.
* A read-only or nonexistent register is refused and the reply has 0.
* The ranges are the ones in `RID_INFO` of
  `setup/cli/commands/motor_read_param_commands.cpp` (from Damiao's
  manual). Only the two out-of-range writes above were tried: the other
  ranges, the upper bounds and the IDs' range are not checked.

### Changing IDs

Writing ESC_ID (8) 0x01 -> 0x0A, then MST_ID (7) 0x11 -> 0x1A:

```
TX 7FF   [8]  01 00 55 08 0A 00 00 00
RX 011  [08]  0A 00 55 08 0A 00 00 00   <- new slave ID in data[0..1]
TX 7FF   [8]  01 00 33 08 00 00 00 00   <- old slave ID: no reply
TX 7FF   [8]  0A 00 55 07 1A 00 00 00
RX 01A  [08]  0A 00 55 07 1A 00 00 00   <- from the new master ID
```

* The ESC_ID reply has the **new** slave ID in data[0..1] and comes from
  the current master ID.
* The MST_ID reply comes from the **new** master ID.
* Both take effect right away: the old slave ID answers neither 0x7FF
  requests nor the disable command.
* `openarm-can-cli change_id` (no `--save`) does this with 50 ms between
  the writes, and all 82 registers could be read at the new IDs.

### RAM and flash

* Writes go to RAM: a power cycle restores the saved values of TIMEOUT,
  ESC_ID and MST_ID.

### Save (0xAA)

* Answered with a **4 byte** frame `ID_L ID_H AA 01`, whatever data[3]
  of the request is (0x00 and 0x01 were tried; both saved):

  ```
  TX 7FF   [8]  01 00 AA 00 00 00 00 00
  RX 011  [04]  01 00 AA 01
  ```

* Right after power on, saving works without a disable command first:
  motors start disabled. (An MIT zero command right after power on was
  answered with state 0.)
* While enabled, saving is refused with no reply.
* The reply comes in about 0.2 ms, **before** the flash is written: the
  motor then ignores requests for about 18 ms. Reads sent every 1-2 ms
  right after the save reply:

  | Read sent after the save request | Reply |
  |---|---|
  | 4.8, 6.8, 8.6, 10.0, 11.7, 12.9, 14.4, 16.0 ms | none |
  | 17.7 ms | after 1.1 ms (normally 0.2 ms) |
  | 19.1 ms and later | after 0.2 ms |

  The ignored requests are dropped, not answered later. Reads 50 ms or
  more after the save were always answered.

## Commands to the slave ID

All are `FF FF FF FF FF FF FF xx` and answered with a state frame.

| Command | Result |
|---|---|
| 0xFC enable | enabled; enabling again is answered the same way |
| 0xFD disable | disabled |
| 0xFB clear error | **disabled**, even when it was enabled |
| MIT control while disabled | answered, state stays 0, no motion |

### State frame

`ID|STATE<<4  P_H P_L  V_H  V_L|T_H  T_L  T_MOS T_ROTOR`

* The low nibble of data[0] is the low nibble of the slave ID (0xA for
  0x0A) and the high one is the state: 0 = disabled, 1 = enabled. Other
  states (errors) were not seen.
* Position 16 bits in [-PMAX, PMAX], velocity and torque 12 bits in
  [-VMAX, VMAX] and [-TMAX, TMAX]. At rest velocity and torque read 0x7FF
  or 0x800, that is, zero falls between the two codes.
* Temperatures in degrees Celsius: 26-31 (MOS) and 23-29 (rotor) in these
  checks.

### Torque in the state frame

It is the torque the motor outputs, including the kp and kd terms, not
the commanded tau. With 0x07 (DM4310) held by MIT at its position +
0.05 rad, kp = 10, kd = 0.5, tau = 0:

| t (s) | q (rad) | kp (q_des - q) | reported tau |
|---|---|---|---|
| 0.02 | -0.0399 | 0.458 | 0.330 |
| 0.04 | -0.0319 | 0.378 | 0.193 |
| 0.30- | -0.0170 | 0.229 | 0.212-0.217 |

* At rest it matches kp (q_des - q). While moving it is smaller by about
  kd * dq (at 0.04 s, dq is about 0.37 rad/s: 0.378 - 0.19 = 0.19).
* The joint stopped 0.023 rad short with 0.21-0.23 Nm. Held at its
  position - 0.05 rad instead, it stopped 0.023 rad short too, with
  -0.24 to -0.25 Nm. So it is static friction of about 0.23-0.25 Nm,
  nearly the same both ways: gravity has little effect on this joint in
  this pose.

## Timing and load

* A reply to a single request comes in 0.1-0.3 ms. In a burst of 8
  reads, the last reply came up to 0.7 ms after its request.
* `show_param` for 8 motors sends 8 reads within 0.1 ms every 35 ms for
  82 RIDs: all 656 were answered.
* MIT zero commands to 8 enabled motors at 500 Hz, 0x01 to 0x08 in each
  cycle as openarm_teleop does:

  | Order | Runs | Lost replies of 0x08 per run |
  |---|---|---|
  | 0x01 -> 0x08 | 1 x 2 s | 1 / 1000 |
  | 0x01 -> 0x08 | 9 x 10 s | 4, 3, 2, 0, 8, 2, 5, 5, 5 / 5000 |
  | 0x08 -> 0x01 | 1 x 10 s | 0 / 5000 |

  No other motor lost any reply. So 0x08 loses about 0.08 % of its
  replies when it is the last one in each cycle. The script sends the 8
  frames back to back from Python, which may not have the same spacing
  as openarm_teleop.

  Where they are lost was narrowed down with the interface's counters:

  * Bus errors: berr-counter, error-pass and bus-off didn't change over
    the 6th 10 s run (2 lost), and RX errors stayed 0 over the last 6
    runs. The replies are not lost to bus errors.
  * Sending: over the last 6 runs (0, 8, 2, 5, 5 and 5 lost), TX packets
    grew by exactly 40016 (8 enables, 40000 MIT commands and 8 disables)
    each run and TX dropped stayed the same. SocketCAN counts a TX packet
    when its transmission completes, so every request was transmitted
    and acknowledged on the bus.
  * Receiving: over the same runs, RX packets grew by exactly as many
    frames as the script received, and RX dropped and missed stayed 0.
    Nothing is lost in the kernel or the socket.

  So either 0x08 didn't answer some requests it was sent, or the adapter
  lost the replies without counting them. Telling these apart needs a
  second listener on the bus.

## Not checked

* 0xFE (set zero position) and changing can_br, as both change the
  motors' state. `change_baudrate` assumes a can_br write takes effect
  right after its reply, which comes at the old bitrate.
* Requests with a DLC other than 8.
* Error states (over-temperature, over-current and so on) and the
  TIMEOUT behavior.

## Differences from the simulators

openarm_can `test/cli/damiao_simulator.py`:

* Replies are Classic CAN; they should be CAN FD with BRS.
* Write replies echo the request; they should carry the resulting value,
  refuse out-of-range values (old value) and read-only registers (0).
  Nonexistent registers are refused like read-only ones. Only some
  ranges are confirmed, see [Write](#write-0x55).
* The ESC_ID write reply has the old slave ID; it should have the new one.
* The MST_ID write reply comes from the old master ID; it should come from
  the new one.
* The save reply is 8 bytes; it should be the 4 bytes `ID_L ID_H AA 01`.
* Requests right after a save are answered; real motors ignore them for
  about 18 ms.
* Motors start enabled after `power_cycle()`; they should start
  disabled.
* Registers other than ESC_ID and MST_ID read 0; the values above could be
  used, per motor type.
* 0xFB is not emulated.

openarm_teleop `test/control/damiao_simulator.py`:

* Classic CAN requests are answered with Classic CAN, and CAN FD replies
  have no BRS; all replies should be CAN FD with BRS.
* The reported torque is kp (q_des - q) + tau; it should also include
  kd (dq_des - dq).
* The static friction is 0.5 N m for every joint; J7 (DM4310) has about
  0.23-0.25 N m in both directions.
