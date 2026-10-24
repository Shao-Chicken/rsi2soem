# rsi2soem API Reference

Complete method reference, mapped to the RSI RMP / RapidCode surface.
Unless noted otherwise, positions and velocities are in **user units**
(default: 1 user unit = 1 mm, i.e. 1024 encoder counts per mm on the
reference 2 mm-pitch / 2048 counts-per-rev machine).

All methods exist with their PascalCase RMP names; common Axis methods also
carry snake_case aliases (`ax.move_absolute(...)` ≡ `ax.MoveTrapezoidal(...)`).

```python
from rsi2soem import (MotionController, Axis, MultiAxis, NetworkNode,
                     CiaState, RSIOperationMode, RSIEventType, RSIAction,
                     RsiError, SoftLimitError, NotEnabledError, FaultError,
                     MotionTimeoutError, CommunicationError, NetworkError,
                     AxisNotPresentError, RsiNotImplementedError)
```

---

## 1. MotionController

### Creation & lifecycle

| Method | Description |
|---|---|
| `MotionController.create(port=None, baud=115200, axes_config=None)` | Connect. `port=None` auto-detects a CH340/USB-serial adapter. `axes_config = {index: (name, (soft_lo, soft_hi))}` names axes and presets limits. |
| `MotionController(port, baud=115200, axes_config=None, serial_factory=None)` | Direct constructor (serial_factory is the test hook). |
| `close()` / `Delete()` | Close the serial link. The controller is a context manager (`with ... as mc`). |

### Axis / node factories

| Method | Description |
|---|---|
| `AxisGet(axis=0)` / `axis_get` | Axis by index **or configured name** (`"Z"`, any case). Unknown → `AxisNotPresentError`. |
| `AxisCountGet()` | Logical axis count on the chain. |
| `ChannelsPerSlaveGet()` | CiA402 channels hosted per EtherCAT node (auto-probed; 3 on the reference one-to-four drive, 1 for cascaded single-channel drives). |
| `MultiAxisGet(axes=None)` | Coordination group; default = all axes. Same tuple → same cached object. |
| `NetworkNodeGet(node=0)` | SDO access object for a node. |
| `mc.z_axis` / `mc.x_axis` / `mc.y_axis` | Named shortcuts (need an `axes_config` with those names). |

### Network

| Method | Description |
|---|---|
| `NetworkStateGet()` | `"OP"` when the network is operational. |
| `NetworkStart(timeout=15)` / `network_wait_op` | Block until every axis reports OP; else `NetworkError`. |
| `NetworkShutdown()` | Disable all axes (the EtherCAT link stays up). |

### Information

`SampleRateGet()` → 1000.0 (firmware cycle) · `FirmwareVersionGet()` ·
`VersionGet()` → library version · `SerialNumberGet()` → stub.

### Error handling

Errors are always raised as exceptions. `ErrorLogCountGet()` → 0,
`ErrorLogGet()` → None, `ErrorLogClear()`, `ThrowExceptions(True)` (only
`True` accepted).

### Explicit stubs (raise `RsiNotImplementedError`)

`RecorderCountGet` · `UserLimitCountGet` · `CompensatorCountGet` ·
`MathBlockCountGet` · `InterruptWait` · `MemoryGet` · `Trace`

### Raw protocol

`raw_command(line, timeout=3.0)` sends one protocol line and returns the
reply (`OK`/`ERR`/`STS`/`SDO`/`AXES`); asynchronous `EVT` lines are
filtered out. See [protocol.md](protocol.md).

---

## 2. Axis

### Motion

| Method | Description |
|---|---|
| `MoveTrapezoidal(position, vel=None, accel=None, decel=None, finalVel=None, jerkPct=None)` | Absolute point-to-point. `vel` defaults to `DefaultVelocityGet()`. accel/decel/jerk are accepted for signature compatibility; ramps are planned in firmware. Out-of-limit targets raise `SoftLimitError` **before** anything is sent. |
| `MoveSCurve(...)` | Same call, drive-side smoothing. |
| `MoveRelative(distance, vel=None, ...)` | Signed relative move. |
| `MoveVelocity(velocity)` | Signed jog in user units/s; `0` stops. Refused past a soft limit or above the velocity cap. |
| `MoveVelocitySCurve(velocity)` | Same, drive-side smoothing. |
| `MovePT / MovePVT / MovePVAJT / MovePTF / MovePVTF` | Stubs — no streamed-point buffer. |

All motion commands are asynchronous: they arm the firmware's CSP stream and
return immediately; wait with `MotionDoneWait`.

### Stop & resume

`Stop()` decelerates and **latches** (further motion is rejected until
`Resume()`). `Abort()`, `EStop()`, `EStopAbort()`, `EStopModify()`,
`EStopModifyAbort()`, `TriggeredModify()` are equivalent to `Stop()` on this
firmware.

### Enable & faults

| Method | Description |
|---|---|
| `AmpEnableSet(bool, timeout_ms=500)` | Enable/disable; verifies the state actually changed, else `RsiError`. |
| `AmpEnableGet()` / `AmpFaultGet()` | Live enable / fault flags. |
| `ClearFaults(timeout=15)` | CiA402 fault reset; the firmware rebuilds the link (~2.5 s) and this waits for OP. Still-faulted afterwards → `FaultError`. |

### Status

| Method | Description |
|---|---|
| `StatusGet()` / `status_get()` | `AxisStatus` snapshot: `status_word`, `ready`, `switched_on`, `enabled`, `fault`, `warning`, `target_reached`, `position_counts`, `velocity_counts_s`, `error_code`, `mode`, `al_state`, `wkc`, plus `.state` / `.moving`. |
| `StateGet()` | `CiaState` enum (the DS402 state machine). |
| `StatusWordGet()` / `StatusBitsGet()` / `StatusBitGet(RSIEventType.X)` | Raw statusword access. |
| `SourceGet()` | `"AXIS"` on fault, else `"NONE"`. |
| `MotionDoneGet()` | Target reached and no fault. |
| `MotionDoneWait(timeout_ms=-1)` | Block until arrival; returns the final position. `FaultError` on drive fault, `MotionTimeoutError` on timeout. |

### Position & velocity

| Method | Description |
|---|---|
| `ActualPositionGet()` | Position in user units (origin offset applied). |
| `PositionSet(x)` / `ActualPositionSet(x)` / `CommandPositionSet(x)` | Define the current spot as coordinate `x` — the zero-calibration call. |
| `EncoderPositionGet()` | Raw counts, no offset. |
| `TargetPositionGet()` / `CommandPositionGet()` | Last commanded target. |
| `OriginPositionGet/Set(x)` | The raw-count origin offset, in user units. |
| `PositionErrorGet()` | Target − actual. |
| `ActualVelocityGet()` / `CommandVelocityGet()` | Stream velocity (user units/s). |
| `FramesToExecuteGet()` | 0 (no streamed buffer). |

### Units

`UserUnitsGet/Set(counts_per_unit)` — default 1024 (1 unit = 1 mm); 2048
makes 1 unit = 1 rev = 2 mm. Limits, velocity cap, tolerances and the pending
target rescale automatically. `UserUnitsToCounts(x)` / `CountsToUserUnits(c)`.

### Limits & protection

| Method | Description |
|---|---|
| `software_limits_set(lo, hi)` / `software_limits_get()` / `software_limits_clear()` | Travel range in user units; **disabled** (`None`) unless configured. While disabled, no range check runs. |
| `SoftwarePosLimitTriggerValueGet/Set` · `SoftwareNegLimitTriggerValueGet/Set` | Same range, RMP names. |
| `max_velocity_set(v)` | Velocity cap (default 5 user units/s). |
| `NegativeLimitGet()` / `PositiveLimitGet()` | "On the limit" flags (software-limit based; the machine has no wired limit switches). |
| `ErrorLimitTriggerValueGet()` | Following-error guard: fixed 20000 counts (firmware clamp). Set → stub. |
| `ErrorLimitActionGet()` → `RSIAction.STOP` · `HardwareNeg/PosLimitActionGet()` → `NONE` · `AmpFaultActionGet()` → `ABORT` | Fixed firmware behaviors; Setters are stubs. |

### Homing (no home switch — manual procedure)

`Home()` is a stub. Manual homing: crawl onto a stop with `MoveVelocity`,
watch the position stall, then `PositionSet(0.0)` — see
[examples/limit_homing.py](../examples/limit_homing.py).
`HomeStateGet/Set`, `HomeMethodGet/Set`, `HomeOffsetGet/Set` are live
bookkeeping; `HomeCancelSet` is a stub.

### Default trajectory parameters

`DefaultVelocityGet/Set` (used when `vel` is omitted) ·
`DefaultAcceleration/Deceleration/JerkPercentGet/Set` (recorded only) ·
`DefaultPosition1/2Get/Set` · `DefaultRelativeIncrementGet/Set`.

### Settling criteria

`PositionToleranceFineGet/Set` (the MotionDoneWait acceptance window, default
30 counts ≈ 0.03 mm) · `PositionToleranceCoarseGet/Set` ·
`VelocityToleranceGet/Set` · `SettlingTimeGet/Set`.

### Miscellaneous

`OperationModeGet()` → `RSIOperationMode.CSP`; `OperationModeSet(non-CSP)` →
stub. `UserLabelGet/Set` · `FeedRateGet/Set` (0..2.0 velocity override,
effective) · `MotionDelayGet/Set` (pre-motion delay, effective) ·
`MotionIdGet/Set` · `StopTimeGet/Set` · `EStopTimeGet/Set` ·
`EStopDecelerationGet/Set` · `NumberGet()` · `AxisCountGet()` · `name_get()`.

### IO stubs

`HomeLimitGet` · `HomeSwitchGet` · `DedicatedInGet/OutGet` ·
`DigitalInGet/OutGet/OutSet` · `AnalogInGet` — no IO is wired to the master.

### Gearing / cam / filters / backlash stubs

`GearingEnable/Disable/RatioChange` · `MoveCamLinear/Cubic` ·
`FilterCoeffGet/Set` · `BacklashWidthGet/Set`.

---

## 3. MultiAxis

A group of axes **started together** — each axis still runs its own CSP
profile, so this is coordination, not interpolation.

| Method | Description |
|---|---|
| `Map()` / `Unmap()` / `IsMapped()` | Mapping gate; unmapped motion raises `RsiError`. |
| `AxisCountGet()` / `NumberGet()` | Group size / first axis index. |
| `AmpEnableSet(bool)` / `ClearFaults()` | All axes together. |
| `MoveTrapezoidal(positions, vel=None)` / `MoveSCurve` / `MoveRelative(deltas, ...)` | Per-axis lists; `vel` single value or per-axis list. |
| `MoveVelocity(velocities)` | Per-axis jog. |
| `Stop()` / `Resume()` / `Abort()` / `EStop()` / `EStopAbort()` | All axes. |
| `StateGet()` | Worst state (FAULT > QUICK_STOP > …). |
| `MotionDoneGet()` / `MotionDoneWait(timeout_ms)` | All arrived; wait returns the per-axis positions. |
| `ActualPositionGet()` / `ActualVelocityGet()` / `PositionErrorGet()` | Per-axis lists. |
| `FeedRateSet(rate)` | All axes. |
| `MovePT` / `MovePVT` | Stubs. |

---

## 4. NetworkNode (raw SDO)

| Method | Description |
|---|---|
| `NumberGet()` | Node index. |
| `SDORead(index, sub=0, size=4, signed=False)` / `SDOReadSigned` / `SDOU16Read` / `SDOU8Read` | Raw CoE SDO reads. Missing object → `CommunicationError`. |
| `SDOWrite(index, sub, size, value)` | Raw write; size ∈ {1, 2, 4}. |
| `status_word()` · `error_code()` · `position_actual()` · `velocity_actual()` · `mode_display()` · `digital_inputs()` · `supported_drive_modes()` | CiA402 convenience getters. **Channel-aware**: on a multi-channel drive they read `base + 0x800×(node % channels_per_slave)` automatically. |

> The drive's software ESC is SDO-sensitive: transactions are bounded to
> ≤ 30 ms, but keep SDO out of real-time loops. See
> [protocol.md](protocol.md#sdo-budget--sdo-预算).

---

## 5. Exceptions

```text
RsiError                       # base of everything
├─ CommunicationError          # serial/SDO failure (incl. missing object)
├─ NetworkError                # network not in OP
├─ SoftLimitError              # limit / velocity-cap rejection (not sent)
├─ FaultError                  # drive fault; carries .error_code (0x603F)
├─ MotionTimeoutError          # MotionDoneWait timeout
├─ NotEnabledError             # motion while disabled
├─ AxisNotPresentError         # bad axis index/name
└─ RsiNotImplementedError      # stubbed families (also a NotImplementedError)
```

## 6. Enums

`CiaState` (alias `RSIState`) — the DS402 states · `RSIOperationMode` —
CSP/PP/PV/HM · `RSIAction` — NONE/STOP/ESTOP/ABORT · `RSIHomeMethod` —
NONE/MANUAL · `RSIEventType` — statusword bit numbers for `StatusBitGet`.

---

## Appendix A. Safety layers

| Layer | Where | Behavior |
|---|---|---|
| Software limits | this library | out-of-range commands never sent |
| Velocity cap | this library | default 5 units/s |
| Enable gate | this library | motion while disabled → `NotEnabledError` |
| Following-error watchdog | firmware | target–actual > 20000 counts for 0.5 s → clamp the stream, stop motion |
| Fault auto-recovery | firmware | drive fault → reset + link rebuild; `REINIT` rebuilds on demand |

## Appendix B. Homing & the relative encoder

Encoder counts are relative to drive power-up. After every drive power
cycle, re-establish zero: crawl onto the negative-end stop at ≤ 0.5 mm/s,
watch for a position stall, back off, then `PositionSet(offset)`. The
reference machine's measured travels: Z 92.1 mm / X 110.7 mm / Y 111.5 mm.
