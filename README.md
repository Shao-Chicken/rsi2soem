# rsi2soem

**RSI RMP-style motion-control API in pure Python, driving an EtherCAT
machine through a USB-serial SOEM master.**

`rsi2soem` — *the RSI RMP API surface, served by a SOEM master* — lets a PC
control CiA402 servo / step-servo drives over EtherCAT **without any EtherCAT
master card, real-time OS, or vendor runtime**: an inexpensive STM32H743
board runs the open-source [SOEM](https://github.com/OpenEtherCATsociety/SOEM)
EtherCAT master stack and exposes a simple ASCII line protocol over a CH340
USB-serial link. This package wraps that protocol in a faithful
re-implementation of the RSI RMP / RapidCode API surface
(`MotionController` / `Axis` / `MultiAxis` / `NetworkNode`), so code written
for RSI hardware ports over with minimal changes.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org)
[![CI](https://github.com/Shao-Chicken/rsi2soem/actions/workflows/ci.yml/badge.svg)](https://github.com/Shao-Chicken/rsi2soem/actions/workflows/ci.yml)

[中文文档](README_zh.md)

## Why

- **No master card, no RTOS, no vendor runtime** — just a USB cable.
- **RMP-compatible method names** — port existing RapidCode programs fast;
  class names, method names, and argument order follow the RSI RMP API.
- **Multi-channel drives supported** — "one-to-four" CiA402 drives (one
  EtherCAT slave hosting several axis channels at object address
  `index + 0x800 × channel`) are auto-detected; channels appear as plain
  `AxisGet(0..N)`.
- **Safe by default** — software travel limits, velocity caps, enable checks,
  and a firmware-side following-error watchdog.
- **Honest about gaps** — features the firmware cannot do (multi-axis
  interpolation, PT/PVT streaming, gearing, cam, filters, hardware IO,
  Recorder, ...) exist as same-named methods that raise
  `RsiNotImplementedError`, so ported code fails loudly at the call site
  instead of silently misbehaving.

## Architecture

```text
┌──────────────────────────┐
│ PC, this package         │   pip install rsi2soem
└────────────┬─────────────┘
             │ CH340 USB-serial, 115200 8N1, ASCII lines
             ▼
┌──────────────────────────┐
│ STM32H743 SOEM master    │   companion firmware: 1 ms EtherCAT cycle,
│ (bare-metal)             │   CSP streaming, watchdogs, homing/limits
└────────────┬─────────────┘
             │ EtherCAT 100 Mbit
             ▼
┌──────────────────────────┐
│ CiA402 drive(s)          │   tested: MOONS' AMA one-to-four step-servo
│ CSP mode                 │   (VendorID 0x168; ch0/ch1/ch2 = 3 axes)
└──────────────────────────┘
```

## Hardware requirements

| Piece | Requirement |
|---|---|
| PC | Windows / Linux / macOS, Python 3.8+, one USB port |
| Bridge board | STM32H743 running the companion SOEM-master firmware (`rsi2soem-firmware`, to be published as a separate repo) |
| USB link | CH340 USB-serial, 115200 8N1 |
| Drives | Any CiA402 EtherCAT drive with CSP support; multi-channel ("one-to-N") drives with `+0x800`-per-channel object addressing are supported out of the box |

Developed and continuously tested on a MOONS' **AMA one-to-four step-servo**
drive (VendorID `0x168`): one EtherCAT slave hosting four axis channels,
three of them wired to a 3-axis linear stage —

| Axis index | Channel | Stage axis | Travel (measured) |
|---|---|---|---|
| 0 | ch0 | Z | 92.1 mm |
| 1 | ch1 | X | 110.7 mm |
| 2 | ch2 | Y | 111.5 mm |

Stage mechanics: 2 mm screw lead, 2048 counts/rev encoder user units
(⇒ `COUNTS_PER_MM = 1024`).

## Installation

```bash
pip install rsi2soem        # from PyPI (once published)
pip install .               # from a clone of this repo
pip install .[test]         # with the test dependencies
```

The only runtime dependency is `pyserial`.

## 60-second quickstart

```python
from rsi2soem import MotionController

# optional: name your axes and set software limits (user units = mm)
AXES = {0: ("Z", (0.0, 91.5)),   # name, (low, high) limits
        1: ("X", (0.0, 110.0)),
        2: ("Y", (0.0, 111.0))}

with MotionController.create("COM12", axes_config=AXES) as mc:
    print(mc.AxisCountGet(), "axes, network:", mc.NetworkStateGet())

    z = mc.AxisGet("Z")
    z.PositionSet(0.0)            # calibrate zero at the current spot
    z.AmpEnableSet(True)

    z.MoveTrapezoidal(10.0, 5.0)  # absolute move: 10 mm at 5 mm/s
    z.MotionDoneWait(15000)
    print("Z at", z.ActualPositionGet(), "mm")

    z.MoveRelative(-2.0)          # relative move at the default velocity
    z.MoveVelocity(0.5)           # jog
    z.Stop(); z.Resume()          # latch / release

    z.AmpEnableSet(False)
```

More in [`examples/`](examples/): a full 3-axis machine setup, and manual
homing against physical limit switches.

## Configuring axes

`axes_config` maps axis index → `(name, (low_limit, high_limit))` in user
units. Names let you write `mc.AxisGet("Z")` instead of `mc.AxisGet(0)`;
limits are enforced in software before any command reaches the drive —
a move that would exceed them raises `SoftLimitError` immediately.

```python
AXES = {0: ("Z", (0.0, 91.5)), 1: ("X", (0.0, 110.0)), 2: ("Y", (0.0, 111.0))}
```

If you omit `axes_config`, axes are unnamed (`AxisGet(0)`, `AxisGet(1)`, ...)
and no software limits apply.

## Multi-channel (one-to-N) drives

One-to-N drives host several axis channels inside a single EtherCAT slave.
Each channel's CiA402 objects live at `base_index + 0x800 × channel`:

| Channel | Controlword 0x6040 | Statusword 0x6041 | Target pos 0x607A | Actual pos 0x6064 |
|---|---|---|---|---|
| ch0 | 0x6040 | 0x6041 | 0x607A | 0x6064 |
| ch1 | 0x6840 | 0x6841 | 0x687A | 0x6864 |
| ch2 | 0x7040 | 0x7041 | 0x707A | 0x7064 |
| ch3 | 0x7840 | 0x7841 | 0x787A | 0x7864 |

The firmware detects the channel count at start-up; `rsi2soem` turns each
channel into an ordinary `Axis` — `AxisGet(1)` on a one-to-four drive talks
to channel 1 automatically. `NetworkNode` gives raw SDO access when you need
a channel-specific object the convenience API does not cover.

## Feature coverage vs RSI RMP

| Family | Status |
|---|---|
| Motion: trapezoidal / S-curve / relative / velocity jog | ✅ real, tested on hardware |
| Stop / Resume / Abort / EStop | ✅ |
| Enable, fault reset, full CiA402 state machine | ✅ |
| Position / velocity / statusword reads, unit scaling (UserUnits) | ✅ |
| Software limits, feed-rate override, motion delay | ✅ |
| MultiAxis "start together" groups | ✅ (no interpolation) |
| NetworkNode raw SDO read/write + CiA402 convenience getters | ✅ |
| PT/PVT streaming, gearing, cam, filters, hardware IO, Recorder | ❌ named stubs raising `RsiNotImplementedError` |

## Testing

Two layers, both runnable from a checkout:

```bash
pytest                        # 43 unit tests over a fake serial link (CI-safe, no hardware)
pytest --ecat-port COM12      # + 30 hardware-in-the-loop tests on the real machine
```

Without `--ecat-port` the hardware layer skips itself automatically, so CI
runs the unit layer only. The hardware layer exercises enable/disable,
absolute and relative moves, velocity jog, stop/resume, limit handling and
statusword decoding on the real 3-axis stage (it moves the Z axis within a
small safe window).

## Documentation

| Document | Contents |
|---|---|
| [Full API reference (English)](docs/api-reference.md) | every class and method, signatures, semantics, error behaviour |
| [完整中文 API 文档](docs/api-zh.md) | 中文完整 API 参考 |
| [Serial wire protocol](docs/protocol.md) | the ASCII line protocol between PC and firmware — useful if you want to port the host side to another language |
| [`examples/quickstart.py`](examples/quickstart.py) | minimal bring-up |
| [`examples/three_axis_machine.py`](examples/three_axis_machine.py) | full 3-axis stage configuration |
| [`examples/limit_homing.py`](examples/limit_homing.py) | manual homing against physical limit switches |

## Roadmap

- Publish the companion firmware as `rsi2soem-firmware` (separate repo)
- PyPI release
- GitHub Actions CI for the unit-test layer
- Firmware-side hardware homing (CiA402 methods 17–30) exposed as
  `RSIHomeMethod`

## Disclaimer

`rsi2soem` is an independent open-source project. It is **not affiliated
with, endorsed by, or sponsored by Robotic Systems Inc. (RSI)**. RMP,
RapidCode, and related names are trademarks of Robotic Systems Inc.; they
are used here solely to describe API compatibility. EtherCAT® is a
registered trademark and patented technology licensed by Beckhoff
Automation GmbH.

## Contributing

Issues and pull requests are welcome. Please run both test layers before
submitting (the unit layer at minimum) and keep the RMP naming surface
stable — compatibility with RSI-style code is the point of the project.

## Acknowledgements

Built on [SOEM](https://github.com/OpenEtherCATsociety/SOEM) (Simple Open
Source EtherCAT Master) by the OpenEtherCAT Society — the firmware runs
SOEM on the STM32H743 so the PC side can stay pure Python.

## License

MIT — see [LICENSE](LICENSE).
