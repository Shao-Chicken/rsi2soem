"""Hardware-in-the-loop test matrix against the real machine.

Runs the full RSI-API surface on the actual STM32 master + drive.
Disabled by default; enable with:

    pytest tests/test_hardware.py --ecat-port COM12

Machine under test (reference setup): one MOONS' one-to-four drive,
axis0=Z / axis1=X / axis2=Y, 2 mm screws. Only the Z axis is moved,
within 0..+3 mm of its parked position above the lower hard stop.
"""

import time

import pytest

from sub2rsi import (AxisNotPresentError, CiaState, CommunicationError,
                     FaultError, MotionController, MotionTimeoutError,
                     NotEnabledError, RsiError, RsiNotImplementedError,
                     RSIEventType, RSIOperationMode, SoftLimitError)

# Reference machine axis table (measured travels, see docs/api-zh.md)
MACHINE_AXES = {0: ("Z", (0.0, 91.5)),
                1: ("X", (0.0, 110.0)),
                2: ("Y", (0.0, 111.0))}

pytestmark = pytest.mark.hardware


@pytest.fixture(scope="module")
def mc(ecat_port):
    ctl = MotionController.create(ecat_port, axes_config=MACHINE_AXES)
    yield ctl
    ctl.close()


@pytest.fixture(scope="module")
def ax(mc):
    return mc.AxisGet(0)   # Z


@pytest.fixture(scope="module")
def ma(mc):
    return mc.MultiAxisGet([0])   # Z only; default maps all three channels


@pytest.fixture(scope="module")
def nd(mc):
    return mc.NetworkNodeGet(0)


# ---------------------------------------------------------------------------
# MotionController
# ---------------------------------------------------------------------------

def test_01_create_and_axes(mc):
    assert mc.AxisCountGet() == 3
    assert mc.ChannelsPerSlaveGet() == 3


def test_02_axis_lookup(mc, ax):
    assert mc.AxisGet("Z") is ax and mc.z_axis is ax
    assert ax.name_get() == "Z"
    assert mc.AxisGet(1).name_get() == "X"
    assert mc.AxisGet("Y") is mc.AxisGet(2)
    assert mc.x_axis.name_get() == "X"
    with pytest.raises(AxisNotPresentError):
        mc.AxisGet(3)
    with pytest.raises(AxisNotPresentError):
        mc.AxisGet("Q")


def test_03_network_state(mc):
    assert mc.NetworkStateGet() == "OP"


def test_04_info(mc):
    assert mc.SampleRateGet() == 1000.0
    assert "ecat" in mc.FirmwareVersionGet()
    assert "sub2rsi" in mc.VersionGet()


def test_05_error_log_empty(mc):
    assert mc.ErrorLogCountGet() == 0
    assert mc.ErrorLogGet() is None


def test_06_controller_stubs(mc):
    with pytest.raises(RsiNotImplementedError):
        mc.RecorderCountGet()
    with pytest.raises(RsiNotImplementedError):
        mc.UserLimitCountGet()
    with pytest.raises(RsiNotImplementedError):
        mc.InterruptWait(10)


def test_07_factories(ma, nd):
    assert ma.AxisCountGet() == 1
    assert nd.NumberGet() == 0


# ---------------------------------------------------------------------------
# Axis: coordinates / units / status
# ---------------------------------------------------------------------------

def test_10_position_set(ax):
    ax.PositionSet(0.0)
    assert abs(ax.ActualPositionGet()) < 0.01


def test_11_user_units_default(ax):
    assert ax.UserUnitsGet() == 1024.0
    assert ax.UserUnitsToCounts(1.0) == 1024
    assert abs(ax.CountsToUserUnits(1024) - 1.0) < 1e-9


def test_12_status_decode(ax):
    st = ax.StatusGet()
    assert not st.fault and st.al_state == "OP"
    assert ax.StateGet() in (CiaState.READY_TO_SWITCH_ON,
                             CiaState.SWITCH_ON_DISABLED,
                             CiaState.OPERATION_ENABLED)
    assert ax.StatusWordGet() == ax.StatusBitsGet()
    assert ax.SourceGet() == "NONE"
    assert ax.AmpFaultGet() is False
    assert ax.EncoderPositionGet() == st.position_counts
    assert ax.error_code_get() is not None   # drive may latch a history code
    assert ax.NumberGet() == 0 and ax.AxisCountGet() == 3
    assert ax.OperationModeGet() == RSIOperationMode.CSP


# ---------------------------------------------------------------------------
# Enable
# ---------------------------------------------------------------------------

def test_20_enable(ax):
    ax.AmpEnableSet(True)
    assert ax.StateGet() == CiaState.OPERATION_ENABLED
    assert ax.AmpEnableGet()
    assert ax.StatusBitGet(RSIEventType.MOTION_DONE) in (True, False)


# ---------------------------------------------------------------------------
# Software limits / protection
# ---------------------------------------------------------------------------

def test_30_soft_limits(ax):
    with pytest.raises(SoftLimitError):
        ax.MoveTrapezoidal(95.0, 1.0)
    with pytest.raises(SoftLimitError):
        ax.MoveTrapezoidal(-0.5, 1.0)
    time.sleep(0.2)
    assert abs(ax.ActualPositionGet()) < 0.05     # nothing was sent
    lo, hi = ax.software_limits_get()
    assert lo == 0.0 and hi == 91.5


# ---------------------------------------------------------------------------
# Point-to-point motion
# ---------------------------------------------------------------------------

def test_40_move_trapezoidal(ax):
    ax.MoveTrapezoidal(1.0, 1.0)
    x = ax.MotionDoneWait(15000)
    assert abs(x - 1.0) < 0.05
    assert abs(ax.TargetPositionGet() - 1.0) < 1e-9
    assert abs(ax.PositionErrorGet()) < 0.05
    assert ax.MotionDoneGet() is True


def test_41_move_relative_roundtrip(ax):
    ax.MoveRelative(1.0, 1.0)
    assert abs(ax.MotionDoneWait(15000) - 2.0) < 0.05
    ax.MoveRelative(-1.0, 1.0)
    assert abs(ax.MotionDoneWait(15000) - 1.0) < 0.05


def test_42_move_scurve(ax):
    ax.MoveSCurve(1.5, 1.0)
    assert abs(ax.MotionDoneWait(15000) - 1.5) < 0.05


def test_43_default_velocity(ax):
    ax.MoveTrapezoidal(1.0)      # vel defaults to DefaultVelocity = 2.0
    t0 = time.time()
    x = ax.MotionDoneWait(15000)
    dt = time.time() - t0
    assert abs(x - 1.0) < 0.05 and dt < 1.5


# ---------------------------------------------------------------------------
# Velocity jog + Stop/Resume
# ---------------------------------------------------------------------------

def test_50_velocity_jog_and_stop(ax):
    xa = ax.ActualPositionGet()
    ax.MoveVelocity(0.5)
    time.sleep(1.2)
    v = ax.ActualVelocityGet()
    ax.Stop()
    time.sleep(0.4)
    xb = ax.ActualPositionGet()
    assert 0.45 < (xb - xa) / 1.2 < 0.65 and abs(v) > 0.3
    assert abs(ax.ActualVelocityGet()) < 0.05


def test_51_stop_latch_resume(ax):
    with pytest.raises(RsiError):
        ax.MoveTrapezoidal(2.0, 1.0)     # latch rejects motion
    ax.Resume()
    ax.MoveTrapezoidal(1.0, 2.0)
    assert abs(ax.MotionDoneWait(15000) - 1.0) < 0.05


def test_52_abort(ax):
    ax.MoveVelocity(0.3)
    time.sleep(0.5)
    ax.Abort()
    time.sleep(0.3)
    assert abs(ax.ActualVelocityGet()) < 0.05
    ax.Resume()


def test_53_motion_done_timeout(ax):
    ax.MoveTrapezoidal(3.0, 0.5)         # 4 mm @0.5 mm/s = 8 s travel
    with pytest.raises(MotionTimeoutError):
        ax.MotionDoneWait(600)
    ax.Abort()
    ax.Resume()
    ax.MoveTrapezoidal(1.0, 2.0)
    ax.MotionDoneWait(15000)
    assert abs(ax.ActualPositionGet() - 1.0) < 0.05


# ---------------------------------------------------------------------------
# FeedRate / MotionDelay
# ---------------------------------------------------------------------------

def test_60_feed_rate(ax):
    ax.FeedRateSet(0.5)
    t0 = time.time()
    ax.MoveRelative(0.4, 1.0)            # effective 0.5 mm/s -> ~0.8 s
    ax.MotionDoneWait(15000)
    dt = time.time() - t0
    ax.FeedRateSet(1.0)
    assert 0.6 < dt < 1.6


def test_61_motion_delay(ax):
    ax.MotionDelaySet(0.4)
    t0 = time.time()
    ax.MoveRelative(-0.2, 2.0)
    ax.MotionDoneWait(15000)
    dt = time.time() - t0
    ax.MotionDelaySet(0.0)
    assert dt >= 0.35


# ---------------------------------------------------------------------------
# UserUnits switching
# ---------------------------------------------------------------------------

def test_70_user_units_switch(ax):
    ax.MoveTrapezoidal(1.0, 2.0)
    ax.MotionDoneWait(15000)
    ax.UserUnitsSet(2048)
    assert abs(ax.ActualPositionGet() - 0.5) < 0.03     # 1 mm = 0.5 rev
    lim = ax.software_limits_get()
    assert abs(lim[1] - 45.75) < 0.01                   # limits follow along
    ax.UserUnitsSet(1024)
    assert abs(ax.ActualPositionGet() - 1.0) < 0.05


# ---------------------------------------------------------------------------
# Configuration accessors
# ---------------------------------------------------------------------------

def test_80_config_accessors(ax):
    ax.DefaultVelocitySet(3.0)
    assert ax.DefaultVelocityGet() == 3.0
    ax.DefaultVelocitySet(2.0)
    ax.DefaultAccelerationSet(12.0)
    ax.DefaultJerkPercentSet(5.0)
    ax.DefaultPosition1Set(0.5)
    ax.DefaultRelativeIncrementSet(0.25)
    assert ax.DefaultAccelerationGet() == 12.0
    assert ax.DefaultJerkPercentGet() == 5.0
    assert ax.DefaultPosition1Get() == 0.5
    assert ax.DefaultRelativeIncrementGet() == 0.25

    ax.PositionToleranceFineSet(0.02)
    ax.VelocityToleranceSet(0.1)
    ax.SettlingTimeSet(0.05)
    assert ax.PositionToleranceFineGet() == 0.02
    assert ax.VelocityToleranceGet() == 0.1
    assert ax.SettlingTimeGet() == 0.05
    ax.PositionToleranceFineSet(30.0 / 1024.0)          # restore default

    ax.UserLabelSet("Z-lift")
    assert ax.UserLabelGet() == "Z-lift"
    ax.MotionIdSet(42)
    ax.StopTimeSet(0.2)
    ax.EStopTimeSet(0.1)
    ax.EStopDecelerationSet(60.0)
    assert ax.MotionIdGet() == 42
    assert ax.StopTimeGet() == 0.2
    assert ax.EStopTimeGet() == 0.1
    assert ax.EStopDecelerationGet() == 60.0

    ax.HomeStateSet(True)
    assert ax.HomeStateGet() is True
    org = ax.OriginPositionGet()
    ax.OriginPositionSet(org)
    assert abs(ax.OriginPositionGet() - org) < 1e-9
    assert abs(ax.ErrorLimitTriggerValueGet() - 20000.0 / 1024.0) < 0.01


def test_81_axis_stubs(ax):
    with pytest.raises(RsiNotImplementedError):
        ax.MovePVT([0], [0], [0], 1, 0, True, True)
    with pytest.raises(RsiNotImplementedError):
        ax.GearingEnable(0, 0, 1, 1)
    with pytest.raises(RsiNotImplementedError):
        ax.FilterCoeffSet(0, 0, 1.0)
    with pytest.raises(RsiNotImplementedError):
        ax.Home()
    with pytest.raises(RsiNotImplementedError):
        ax.AnalogInGet(0)
    with pytest.raises(RsiNotImplementedError):
        ax.OperationModeSet(RSIOperationMode.PP)


def test_82_not_enabled_protection(ax):
    ax.AmpEnableSet(False)
    with pytest.raises(NotEnabledError):
        ax.MoveTrapezoidal(1.0, 1.0)
    ax.AmpEnableSet(True)


# ---------------------------------------------------------------------------
# NetworkNode (SDO)
# ---------------------------------------------------------------------------

def test_90_sdo(ax, nd):
    sw_sdo = nd.status_word()
    sw_sts = ax.StatusWordGet()
    assert sw_sdo == sw_sts
    assert nd.position_actual() == ax.EncoderPositionGet()
    assert nd.supported_drive_modes() == 0x3AD
    assert nd.error_code() is not None    # latched history code allowed
    assert nd.mode_display() == 8
    nd.SDOWrite(0x6060, 0, 1, 8)          # write the current mode (CSP): safe
    time.sleep(0.1)
    assert nd.SDORead(0x6060, 0, size=1) == 8
    with pytest.raises(CommunicationError):
        nd.SDORead(0x2999, 0)


def test_91_sdo_channel_offset(mc):
    """X/Y channel convenience reads must return exactly the same statusword
    the PDO stream reports for those axes (proves the +0x800 offset hits the
    right channel on the one-to-four drive)."""
    for i in (1, 2):
        assert mc.NetworkNodeGet(i).status_word() == mc.AxisGet(i).StatusWordGet()


# ---------------------------------------------------------------------------
# MultiAxis
# ---------------------------------------------------------------------------

def test_95_multiaxis(ma):
    ma.AmpEnableSet(True)
    assert ma.StateGet() == CiaState.OPERATION_ENABLED
    ma.MoveTrapezoidal([2.0], 1.0)
    xs = ma.MotionDoneWait(15000)
    assert abs(xs[0] - 2.0) < 0.05
    ma.MoveRelative([-1.0], 1.0)
    xs = ma.MotionDoneWait(15000)
    assert abs(xs[0] - 1.0) < 0.05
    assert abs(ma.ActualPositionGet()[0] - 1.0) < 0.05
    assert ma.MotionDoneGet() is True
    ma.MoveVelocity([0.3])
    time.sleep(0.5)
    ma.Stop()
    time.sleep(0.3)
    assert abs(ma.ActualVelocityGet()[0]) < 0.05
    ma.Resume()
    with pytest.raises(RsiNotImplementedError):
        ma.MovePVT([0], [0], [0], 1, 0, True, True)
    ma.Unmap()
    with pytest.raises(RsiError):
        ma.MoveTrapezoidal([1.0], 1.0)
    ma.Map()
    assert ma.IsMapped() is True


# ---------------------------------------------------------------------------
# Shutdown: back to the reference point, disable
# ---------------------------------------------------------------------------

def test_99_park(ax):
    ax.MoveTrapezoidal(0.0, 2.0)
    x = ax.MotionDoneWait(15000)
    assert abs(x) < 0.05
    assert ax.NegativeLimitGet() is True
    ax.AmpEnableSet(False)
    assert ax.StateGet() != CiaState.OPERATION_ENABLED
