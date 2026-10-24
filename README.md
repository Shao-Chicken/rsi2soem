# sub2rsi

**RSI RMP-style motion-control API in pure Python, talking to an EtherCAT
machine through a USB-serial SOEM master.**

`sub2rsi` (**S**OEM **U**SB **B**ridge **to** **RSI**) lets a PC control
CiA402 servo/stepper drives over EtherCAT **without any EtherCAT master card
or real-time OS**: an inexpensive STM32H743 board runs the open-source
[SOEM](https://github.com/OpenEtherCATsociety/SOEM) master stack and exposes a
simple ASCII line protocol over a CH340 USB-serial link. This package wraps
that protocol in a faithful re-implementation of the RSI RMP / RapidCode API
surface (`MotionController` / `Axis` / `MultiAxis` / `NetworkNode`), so code
written for RSI hardware ports over with minimal changes.

[中文文档](README_zh.md)

## Why

- **No master card, no RTOS, no vendor runtime** — just a USB cable.
- **RMP-compatible method names** — port existing RapidCode programs fast.
- **Multi-channel drives supported** — "one-to-four" CiA402 drives (one
  EtherCAT slave hosting several axis channels at `index + 0x800×ch`) are
  auto-detected; axes appear as plain `AxisGet(0..2)`.
- **Safe by default** — software limits, velocity caps, enable checks and a
  firmware-side following-error watchdog.
- **Honest about gaps** — features the firmware cannot do (interpolation,
  PT/PVT streaming, gearing, ...) exist as stubs raising
  `RsiNotImplementedError`, never silently misbehave.

## Architecture

```text
┌──────────────────────────┐
│ PC, this package         │   pip install sub2rsi
└────────────┬─────────────┘
             │ CH340 USB-serial, 115200 8N1, ASCII lines
             ▼
┌──────────────────────────┐
│ STM32H743 SOEM master    │   firmware: 1 ms EtherCAT cycle, CSP streaming
│ (companion firmware)     │   https://github.com/your-org/sub2rsi-firmware
└────────────┬─────────────┘
             │ EtherCAT 100 Mbit
             ▼
┌──────────────────────────┐
│ CiA402 drive(s)          │   tested: MOONS' AMA one-to-four step-servo
│ CSP mode                 │   (ch0/ch1/ch2 = three axes, ch3 spare)
└──────────────────────────┘
```

## Install

```bash
pip install sub2rsi          # from PyPI (once published)
pip install .                # from a clone of this repo
pip install .[test]          # with the test dependencies
```

## 60-second quickstart

```python
from sub2rsi import MotionController

# optional: name your axes and set software limits (user units = mm)
AXES = {0: ("Z", (0.0, 91.5)),   # name, (low, high) limits
        1: ("X", (0.0, 110.0)),
        2: ("Y", (0.0, 111.0))}

with MotionController.create("COM12", axes_config=AXES) as mc:
    print(mc.AxisCountGet(), "axes, network:", mc.NetworkStateGet())

    z = mc.AxisGet("Z")
    z.PositionSet(0.0)            # calibrate zero at the current spot
    z.AmpEnableSet(True)

    z.MoveTrapezoidal(10.0, 5.0)  # absolute move, 10 mm at 5 mm/s
    z.MotionDoneWait(15000)
    print("Z at", z.ActualPositionGet(), "mm")

    z.MoveRelative(-2.0)          # relative, default velocity
    z.MoveVelocity(0.5)           # jog
    z.Stop(); z.Resume()          # latch / release

    z.AmpEnableSet(False)
```

## Feature coverage vs RSI RMP

| Family | Status |
|---|---|
| Motion: trapezoidal / S-curve / relative / velocity jog | ✅ real, tested on hardware |
| Stop / Resume / Abort / EStop | ✅ |
| Enable, fault reset, CiA402 state machine | ✅ |
| Position/velocity/status reads, unit scaling (UserUnits) | ✅ |
| Software limits, feed-rate override, motion delay | ✅ |
| MultiAxis "start together" groups | ✅ (no interpolation) |
| NetworkNode raw SDO read/write + CiA402 convenience getters | ✅ |
| PT/PVT streaming, gearing, cam, filters, hardware IO, Recorder | ❌ named stubs raising `RsiNotImplementedError` |

## Testing

```bash
pytest                        # 43 unit tests over a fake serial link (CI-safe)
pytest --ecat-port COM12      # + 30 hardware-in-the-loop tests on the machine
```

## Documentation

- [Full API reference](docs/api-reference.md) (English)
- [完整中文 API 文档](docs/api-zh.md)
- [Serial wire protocol](docs/protocol.md)
- [examples/](examples/) — quickstart, 3-axis machine setup, manual homing

## License

MIT — see [LICENSE](LICENSE).
