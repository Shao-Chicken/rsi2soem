"""Pure-logic unit tests over the fake serial link (no hardware needed)."""

import pytest

import rsi2soem as s2r
from rsi2soem import (AxisNotPresentError, CiaState, CommunicationError,
                     MotionTimeoutError, NotEnabledError, RsiError,
                     RsiNotImplementedError, RSIOperationMode, SoftLimitError)


# ---------------------------------------------------------------------------
# Controller creation / object factories
# ---------------------------------------------------------------------------

class TestController:
    def test_axis_count(self, mc):
        assert mc.AxisCountGet() == 3

    def test_channel_probe(self, mc):
        assert mc.ChannelsPerSlaveGet() == 3

    def test_single_channel_chain(self):
        from conftest import FakeSerial
        mc = s2r.MotionController(
            "COM_FAKE",
            serial_factory=lambda p, b: FakeSerial(p, b, n_axes=2, channels=1))
        assert mc.ChannelsPerSlaveGet() == 1
        assert mc.AxisCountGet() == 2

    def test_axis_by_index_and_name(self, mc):
        assert mc.AxisGet(0) is mc.AxisGet("Z") is mc.z_axis
        assert mc.AxisGet(1) is mc.AxisGet("x") is mc.x_axis
        assert mc.AxisGet("Y") is mc.y_axis

    def test_axis_out_of_range(self, mc):
        with pytest.raises(AxisNotPresentError):
            mc.AxisGet(3)
        with pytest.raises(AxisNotPresentError):
            mc.AxisGet("Q")

    def test_generic_names_without_config(self, mc_generic):
        assert mc_generic.AxisGet(0).name_get() == "AX0"
        assert mc_generic.AxisGet(1).name_get() == "AX1"
        with pytest.raises(AxisNotPresentError):
            mc_generic.AxisGet("Z")   # no names configured

    def test_version_strings(self, mc):
        assert "rsi2soem" in mc.VersionGet()
        assert "ecat" in mc.FirmwareVersionGet()
        assert mc.SampleRateGet() == 1000.0

    def test_error_log_api(self, mc):
        assert mc.ErrorLogCountGet() == 0
        assert mc.ErrorLogGet() is None
        mc.ErrorLogClear()

    def test_controller_stubs(self, mc):
        for fn in (mc.RecorderCountGet, mc.UserLimitCountGet,
                   mc.CompensatorCountGet, mc.MathBlockCountGet,
                   lambda: mc.InterruptWait(10), lambda: mc.MemoryGet(0),
                   lambda: mc.Trace(True), mc.SerialNumberGet,
                   lambda: mc.ThrowExceptions(False)):
            with pytest.raises(RsiNotImplementedError):
                fn()
        mc.ThrowExceptions(True)  # accepted


# ---------------------------------------------------------------------------
# Units, coordinates, limits (no motion needed)
# ---------------------------------------------------------------------------

class TestUnitsAndCoordinates:
    def test_default_units_mm(self, mc):
        ax = mc.AxisGet(0)
        assert ax.UserUnitsGet() == 1024.0
        assert ax.UserUnitsToCounts(1.0) == 1024
        assert abs(ax.CountsToUserUnits(1024) - 1.0) < 1e-9

    def test_user_units_rescale(self, mc):
        ax = mc.AxisGet(0)
        lo, hi = ax.software_limits_get()
        ax.UserUnitsSet(2048)          # 1 unit = 1 rev = 2 mm
        assert ax.UserUnitsGet() == 2048.0
        lo2, hi2 = ax.software_limits_get()
        assert abs(hi2 - hi / 2.0) < 1e-9   # same physical travel
        assert abs(lo2 - lo / 2.0) < 1e-9

    def test_position_set_origin(self, mc):
        ax = mc.AxisGet(0)
        ax.PositionSet(0.0)
        assert abs(ax.ActualPositionGet()) < 1e-9
        ax.PositionSet(5.0)
        assert abs(ax.ActualPositionGet() - 5.0) < 1e-9

    def test_soft_limit_reject(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        with pytest.raises(SoftLimitError):
            ax.MoveTrapezoidal(95.0, 1.0)   # beyond +91.5
        with pytest.raises(SoftLimitError):
            ax.MoveTrapezoidal(-0.5, 1.0)   # below 0

    def test_soft_limits_disabled_by_default(self, mc_generic):
        ax = mc_generic.AxisGet(0)
        assert ax.software_limits_get() is None
        ax.AmpEnableSet(True)
        ax.MoveTrapezoidal(10000.0, 1.0)    # no limit check -> sent
        assert mc_generic._ser.sent[-1].startswith("AMOV 0 10240000")

    def test_max_velocity_reject(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        with pytest.raises(SoftLimitError):
            ax.MoveTrapezoidal(1.0, 99.0)

    def test_feed_rate(self, mc):
        ax = mc.AxisGet(0)
        ax.FeedRateSet(0.5)
        assert ax.FeedRateGet() == 0.5
        with pytest.raises(RsiError):
            ax.FeedRateSet(2.5)
        ax.FeedRateSet(1.0)

    def test_feed_rate_zero_blocks_motion(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        ax.FeedRateSet(0.0)
        with pytest.raises(RsiError):
            ax.MoveTrapezoidal(1.0, 1.0)
        ax.FeedRateSet(1.0)


# ---------------------------------------------------------------------------
# Motion over the fake link
# ---------------------------------------------------------------------------

class TestMotion:
    def test_not_enabled_rejected(self, mc):
        ax = mc.AxisGet(0)
        with pytest.raises(NotEnabledError):
            ax.MoveTrapezoidal(1.0, 1.0)

    def test_enable_sequence(self, mc):
        ax = mc.AxisGet(0)
        assert ax.AmpEnableSet(True) == 0
        assert ax.StateGet() == CiaState.OPERATION_ENABLED
        assert ax.AmpEnableGet() is True
        ax.AmpEnableSet(False)
        assert ax.StateGet() != CiaState.OPERATION_ENABLED

    def test_move_trapezoidal_wire(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        ax.PositionSet(0.0)
        ax.MoveTrapezoidal(1.0, 1.0)
        assert mc._ser.sent[-1] == "AMOV 0 1024 1024"
        assert abs(ax.MotionDoneWait(5000) - 1.0) < 0.05

    def test_move_relative_wire(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        ax.PositionSet(5.0)                # park mid-travel
        ax.MoveRelative(-2.0, 2.0)
        assert mc._ser.sent[-1] == "RMOV 0 -2048 2048"

    def test_move_velocity_wire(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        ax.MoveVelocity(0.5)
        assert mc._ser.sent[-1] == "VEL 0 512"
        ax.MoveVelocity(0.0)
        assert mc._ser.sent[-1] == "VEL 0 0"

    def test_velocity_at_limit_refused(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        ax.PositionSet(0.0)          # parked at the negative limit (0.0)
        with pytest.raises(SoftLimitError):
            ax.MoveVelocity(-0.5)    # cannot move further negative
        ax.MoveVelocity(0.5)         # positive is fine
        ax.MoveVelocity(0.0)

    def test_stop_resume_latch(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        ax.Stop()
        assert "HALT 0" in mc._ser.sent
        with pytest.raises(RsiError):
            ax.MoveTrapezoidal(1.0, 1.0)     # firmware rejects while halted
        ax.Resume()
        assert "CONT 0" in mc._ser.sent
        ax.MoveTrapezoidal(1.0, 1.0)         # works again
        ax.MotionDoneWait(5000)

    def test_abort_and_estop_aliases(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        for fn in (ax.Abort, ax.EStop, ax.EStopAbort, ax.TriggeredModify):
            fn()
            ax.Resume()
        assert mc._ser.sent.count("HALT 0") >= 4

    def test_motion_done_timeout(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        # force the fake drive to report 'not there yet'
        ax._mc._ser.axes[0].pos = 50000   # far from any target
        ax._last_target = 99.0            # unreachable target
        with pytest.raises(MotionTimeoutError):
            ax.MotionDoneWait(300)

    def test_streaming_stubs(self, mc):
        ax = mc.AxisGet(0)
        for fn in (ax.MovePT, ax.MovePVT, ax.MovePVAJT, ax.MovePTF,
                   ax.MovePVTF):
            with pytest.raises(RsiNotImplementedError):
                fn()

    def test_misc_stubs(self, mc):
        ax = mc.AxisGet(0)
        with pytest.raises(RsiNotImplementedError):
            ax.OperationModeSet(RSIOperationMode.PP)
        assert ax.OperationModeGet() == RSIOperationMode.CSP
        with pytest.raises(RsiNotImplementedError):
            ax.Home()
        with pytest.raises(RsiNotImplementedError):
            ax.GearingEnable(1, 1)
        with pytest.raises(RsiNotImplementedError):
            ax.DigitalOutSet(0, True)


# ---------------------------------------------------------------------------
# Status decoding
# ---------------------------------------------------------------------------

class TestStatus:
    def test_statusword_decode(self):
        assert s2r.decode_cia_state(0x1637) == CiaState.OPERATION_ENABLED
        assert s2r.decode_cia_state(0x0250) == CiaState.SWITCH_ON_DISABLED
        assert s2r.decode_cia_state(0x0631) == CiaState.READY_TO_SWITCH_ON
        assert s2r.decode_cia_state(0x0618) == CiaState.FAULT

    def test_status_snapshot(self, mc):
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        st = ax.StatusGet()
        assert st.enabled and not st.fault and st.al_state == "OP"
        assert st.state == CiaState.OPERATION_ENABLED
        assert st.wkc == 3
        assert repr(st)

    def test_status_bit_get(self, mc):
        from rsi2soem import RSIEventType
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        assert ax.StatusBitGet(RSIEventType.ENABLED) is True
        assert ax.StatusBitGet(RSIEventType.FAULT) is False

    def test_source_get(self, mc):
        ax = mc.AxisGet(0)
        assert ax.SourceGet() == "NONE"


# ---------------------------------------------------------------------------
# MultiAxis
# ---------------------------------------------------------------------------

class TestMultiAxis:
    def test_group_motion(self, mc):
        ma = mc.MultiAxisGet()
        assert ma.AxisCountGet() == 3
        ma.AmpEnableSet(True)
        assert ma.StateGet() == CiaState.OPERATION_ENABLED
        ma.MoveTrapezoidal([1.0, 2.0, 3.0], 1.0)
        xs = ma.MotionDoneWait(5000)
        assert abs(xs[0] - 1.0) < 0.05 and abs(xs[2] - 3.0) < 0.05
        assert ma.MotionDoneGet() is True

    def test_subset_group(self, mc):
        ma = mc.MultiAxisGet([0, 2])
        assert ma.AxisCountGet() == 2
        with pytest.raises(RsiError):
            ma.MoveTrapezoidal([1.0], 1.0)   # count mismatch

    def test_unmap_blocks_motion(self, mc):
        ma = mc.MultiAxisGet([0])
        ma.Unmap()
        assert ma.IsMapped() is False
        with pytest.raises(RsiError):
            ma.MoveTrapezoidal([1.0], 1.0)
        ma.Map()
        assert ma.IsMapped() is True

    def test_expand_vel(self, mc):
        ma = mc.MultiAxisGet([0, 1])
        with pytest.raises(RsiError):
            ma._expand_vel([1.0, 2.0, 3.0])
        assert ma._expand_vel(1.0) == [1.0, 1.0]


# ---------------------------------------------------------------------------
# NetworkNode (SDO)
# ---------------------------------------------------------------------------

class TestNetworkNode:
    def test_sdo_read(self, mc):
        nd = mc.NetworkNodeGet(0)
        assert nd.status_word() == 0x0250
        assert nd.mode_display() == 8
        assert nd.supported_drive_modes() == 0x3AD

    def test_channel_offset(self, mc):
        """Node 1 on a 3-channel drive must read 0x6841 (ch1 statusword)."""
        nd1 = mc.NetworkNodeGet(1)
        assert nd1.status_word() == 0x0250   # fake answers 0x6841
        # verify the wire really carried the offset index
        assert any("26689" in s for s in mc._ser.sent)   # 0x6841 = 26689

    def test_sdo_write_readback(self, mc):
        nd = mc.NetworkNodeGet(0)
        nd.SDOWrite(0x6060, 0, 1, 8)
        assert nd.SDORead(0x6060, 0, size=1) == 8

    def test_missing_object(self, mc):
        nd = mc.NetworkNodeGet(0)
        with pytest.raises(CommunicationError):
            nd.SDORead(0x2999, 0)

    def test_sdo_size_guard(self, mc):
        nd = mc.NetworkNodeGet(0)
        with pytest.raises(RsiError):
            nd.SDOWrite(0x6060, 0, 3, 0)


# ---------------------------------------------------------------------------
# Protocol edge cases
# ---------------------------------------------------------------------------

class TestProtocol:
    def test_evt_lines_skipped(self, mc):
        """Asynchronous EVT lines must not be mistaken for command replies."""
        ax = mc.AxisGet(0)
        ax.AmpEnableSet(True)
        ax.PositionSet(0.0)
        ax.MoveTrapezoidal(1.0, 1.0)   # fake pushes OK then EVT MOVE_DONE
        st = ax.status_get()           # next reply must be the STS line
        assert st.axis == 0

    def test_no_reply_raises(self):
        from conftest import FakeSerial

        class DeafSerial(FakeSerial):
            def _push(self, line):
                pass  # never answers

        # the constructor's AXES probe is the first command to time out
        with pytest.raises(CommunicationError):
            s2r.MotionController("COM_FAKE",
                                 serial_factory=lambda p, b: DeafSerial(p, b))
