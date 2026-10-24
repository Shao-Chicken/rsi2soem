"""Axis: motion, status, limits, unit scaling. Full RSI Axis method surface.

Naming: PascalCase methods are the formal interface, identical to RSI;
common methods also carry snake_case aliases (Python convention).
"""

import re
import time

from .constants import (COUNTS_PER_MM, DONE_TOL_COUNTS_DEFAULT,
                        DEFAULT_MAX_VEL_MM_S, AxisStatus, RSIAction,
                        RSIHomeMethod, RSIOperationMode)
from .exceptions import (CommunicationError, FaultError, MotionTimeoutError,
                         NotEnabledError, NetworkError, RsiError,
                         SoftLimitError, _ni)


class Axis(object):
    """One axis: motion, status, limits, units. Mirrors RSI::RapidCode::Axis.

    Software limits are optional: an axis starts with limits DISABLED unless
    the controller's ``axes_config`` supplies a range. Disabled limits mean
    no travel-range check is applied (use with care on real hardware).
    """

    def __init__(self, controller, index):
        self._mc = controller
        self._index = index
        name, limits = controller._axes_config.get(
            index, ("AX%d" % index, None))
        self._name = name
        # units: 1 user unit = 1 mm  =>  counts_per_unit = 1024
        self._counts_per_unit = COUNTS_PER_MM
        # coordinate offset: user_x = (raw_counts - origin_offset)/counts_per_unit
        self._origin_offset = 0
        # software limits (user units); None = disabled
        if limits is None:
            self._soft_lo = None
            self._soft_hi = None
        else:
            self._soft_lo = float(limits[0])
            self._soft_hi = float(limits[1])
        self._max_vel = DEFAULT_MAX_VEL_MM_S
        # default trajectory parameters (RMP Default* family; accel is only
        # recorded - the firmware plans ramps internally)
        self._def_vel = 2.0
        self._def_accel = 10.0
        self._def_decel = 10.0
        self._def_jerk = 0.0
        self._def_pos1 = 0.0
        self._def_pos2 = 0.0
        self._def_rel_inc = 1.0
        # settling / done-decision windows (user units)
        self._tol_fine = DONE_TOL_COUNTS_DEFAULT / self._counts_per_unit
        self._tol_coarse = 4 * DONE_TOL_COUNTS_DEFAULT / self._counts_per_unit
        self._vel_tol = 0.05
        self._settle_time = 0.0
        # other RMP configuration records
        self._feed_rate = 1.0          # velocity override (effective)
        self._motion_delay = 0.0       # delay before motion, s (effective)
        self._motion_id = 0
        self._user_label = ""
        self._home_state = False
        self._home_method = RSIHomeMethod.MANUAL
        self._stop_time = 0.1
        self._estop_time = 0.05
        self._estop_decel = 50.0
        self._last_target = None
        self._last_stream_vel = 0.0

    # ==================================================================
    # Internal helpers
    # ==================================================================

    def _read_raw_position(self):
        return self.status_get().position_counts

    def _require_enabled(self):
        st = self.status_get()
        if st.fault:
            raise FaultError("axis %s drive fault, err=0x%04X - ClearFaults() "
                             "first" % (self._name, st.error_code),
                             st.error_code)
        if not st.enabled:
            raise NotEnabledError("axis %s not enabled - AmpEnableSet(True) "
                                  "first" % self._name)
        if st.al_state != "OP":
            raise NetworkError("axis %s EtherCAT not in OP: %s"
                               % (self._name, st.al_state))
        return st

    def _check_limits(self, target, vel):
        if self._soft_lo is not None and not (
                self._soft_lo - 1e-9 <= target <= self._soft_hi + 1e-9):
            raise SoftLimitError(
                "axis %s target %.3f outside software limits [%.3f, %.3f] "
                "(user units); command NOT sent"
                % (self._name, target, self._soft_lo, self._soft_hi))
        if abs(vel) > self._max_vel:
            raise SoftLimitError("axis %s velocity %g exceeds limit %g "
                                 "(user units/s)"
                                 % (self._name, abs(vel), self._max_vel))

    def _effective_vel(self, vel):
        """Apply the FeedRate override and take the absolute value."""
        v = abs(float(vel)) * self._feed_rate
        if v < 1e-9:
            raise RsiError("effective velocity is 0 (check FeedRateGet() "
                           "is not 0)")
        return v

    def _vel_to_counts(self, v):
        return max(1, abs(self.UserUnitsToCounts(v)))

    def _send_motion(self, wire):
        if self._motion_delay > 0:
            time.sleep(self._motion_delay)
        line = self._mc.raw_command(wire)
        if line.startswith("ERR"):
            raise RsiError("axis %s motion command rejected: %s"
                           % (self._name, line))

    def _done_tol_counts(self):
        return max(1, self.UserUnitsToCounts(self._tol_fine))

    # ==================================================================
    # Motion (MoveTrapezoidal / MoveSCurve / MoveRelative / MoveVelocity)
    # ==================================================================

    def MoveTrapezoidal(self, position, vel=None, accel=None, decel=None,
                        finalVel=None, jerkPct=None):
        """Trapezoidal absolute point-to-point move (RMP MoveTrapezoidal).
        accel/decel/finalVel/jerkPct are signature-compatible only (the
        firmware plans ramps internally). ``vel`` defaults to
        DefaultVelocityGet(). Targets beyond the software limits raise
        SoftLimitError before anything is sent."""
        v = self._effective_vel(vel if vel is not None else self._def_vel)
        self._require_enabled()
        self._check_limits(float(position), v)
        raw_target = self._origin_offset + self.UserUnitsToCounts(position)
        self._last_target = float(position)
        self._last_stream_vel = v
        self._send_motion("AMOV %d %d %d" % (self._index, raw_target,
                                             self._vel_to_counts(v)))

    def MoveSCurve(self, position, vel=None, accel=None, decel=None,
                   jerkPct=None, finalVel=None):
        """S-curve absolute move (RMP MoveSCurve). The drive smooths the
        streamed position internally; jerkPct is signature-compatible only."""
        self.MoveTrapezoidal(position, vel, accel, decel, finalVel, jerkPct)

    def MoveRelative(self, relativePosition, vel=None, accel=None, decel=None,
                     jerkPct=None, finalVel=None):
        """Relative point-to-point move (RMP MoveRelative). Signed distance."""
        v = self._effective_vel(vel if vel is not None else self._def_vel)
        self._require_enabled()
        target = self.ActualPositionGet() + relativePosition
        self._check_limits(target, v)
        raw_delta = self.UserUnitsToCounts(relativePosition)
        self._last_target = float(target)
        self._last_stream_vel = v
        self._send_motion("RMOV %d %d %d" % (self._index, raw_delta,
                                             self._vel_to_counts(v)))

    def MoveVelocity(self, velocity, accel=None):
        """Constant-velocity jog (RMP MoveVelocity). Signed; 0 = stop."""
        st = self._require_enabled()
        v_signed = float(velocity) * self._feed_rate
        if abs(v_signed) > self._max_vel:
            raise SoftLimitError("axis %s velocity %g exceeds limit %g "
                                 "(user units/s)"
                                 % (self._name, abs(v_signed), self._max_vel))
        if velocity != 0 and self._soft_lo is not None:
            x = self.CountsToUserUnits(st.position_counts - self._origin_offset)
            if velocity > 0 and x >= self._soft_hi:
                raise SoftLimitError("axis %s already at the positive soft "
                                     "limit (%.3f); cannot move further +"
                                     % (self._name, self._soft_hi))
            if velocity < 0 and x <= self._soft_lo:
                raise SoftLimitError("axis %s already at the negative soft "
                                     "limit (%.3f); cannot move further -"
                                     % (self._name, self._soft_lo))
        self._last_target = None
        self._last_stream_vel = v_signed
        c = self.UserUnitsToCounts(v_signed)
        if c == 0 and v_signed != 0:
            c = 1 if v_signed > 0 else -1
        self._send_motion("VEL %d %d" % (self._index, c))

    def MoveVelocitySCurve(self, velocity, accel=None, jerkPct=None):
        """S-curve velocity jog (RMP MoveVelocitySCurve); smoothing is done
        inside the drive."""
        self.MoveVelocity(velocity, accel)

    # ---- streamed point buffers: not supported by the firmware ---------

    def MovePT(self, *args, **kwargs):
        _ni("MovePT", "no streamed-point buffer in firmware; "
            "use a MoveTrapezoidal sequence instead")

    def MovePVT(self, *args, **kwargs):
        _ni("MovePVT", "no streamed-point buffer in firmware")

    def MovePVAJT(self, *args, **kwargs):
        _ni("MovePVAJT", "no streamed-point buffer in firmware")

    def MovePTF(self, *args, **kwargs):
        _ni("MovePTF", "no streamed-point buffer in firmware")

    def MovePVTF(self, *args, **kwargs):
        _ni("MovePVTF", "no streamed-point buffer in firmware")

    # ==================================================================
    # Stop and resume (Stop / Resume / EStop / Abort / TriggeredModify)
    # ==================================================================

    def Stop(self):
        """Decelerate to a stop and latch (RMP Stop). Any later motion needs
        Resume() first."""
        line = self._mc.raw_command("HALT %d" % self._index)
        if line.startswith("ERR"):
            raise RsiError("axis %s Stop failed: %s" % (self._name, line))
        self._last_stream_vel = 0.0

    def Resume(self):
        """Release the Stop latch (RMP Resume)."""
        line = self._mc.raw_command("CONT %d" % self._index)
        if line.startswith("ERR"):
            raise RsiError("axis %s Resume failed: %s" % (self._name, line))

    def Abort(self):
        """Abort the current motion (RMP Abort); equivalent to Stop here."""
        self.Stop()

    def EStop(self):
        """Fastest deceleration stop (RMP EStop); equivalent to Stop here."""
        self.Stop()

    def EStopAbort(self):
        """RMP EStopAbort; equivalent to Stop here."""
        self.Stop()

    def EStopModify(self):
        self.Stop()

    def EStopModifyAbort(self):
        self.Stop()

    def TriggeredModify(self):
        """RMP TriggeredModify: decelerate to zero and idle; equals Stop."""
        self.Stop()

    # ==================================================================
    # Enable and faults (AmpEnableSet / ClearFaults)
    # ==================================================================

    def AmpEnableSet(self, enable, ampActiveTimeoutMilliseconds=500):
        """Enable / disable the drive (RMP AmpEnableSet)."""
        line = self._mc.raw_command("%s %d" % ("EN" if enable else "DIS",
                                               self._index))
        if line.startswith("ERR"):
            raise RsiError("axis %s enable operation failed: %s"
                           % (self._name, line))
        time.sleep(max(0.3, ampActiveTimeoutMilliseconds / 1000.0))
        st = self.status_get()
        if enable and not st.enabled:
            raise RsiError("axis %s enable did not take effect, sw=0x%04X"
                           % (self._name, st.status_word))
        if not enable and st.enabled:
            raise RsiError("axis %s disable did not take effect, sw=0x%04X"
                           % (self._name, st.status_word))
        return 0

    def ClearFaults(self, wait_op_timeout=15.0):
        """Fault reset (RMP ClearFaults). The firmware then rebuilds the
        EtherCAT link (~2.5 s back to OP); this method waits for it."""
        line = self._mc.raw_command("RESET %d" % self._index, timeout=5.0)
        if line.startswith("ERR") and "no fault" not in line:
            raise RsiError("axis %s fault reset failed: %s"
                           % (self._name, line))
        self._mc.network_wait_op(wait_op_timeout)
        st = self.status_get()
        if st.fault:
            raise FaultError("axis %s still in fault after reset, err=0x%04X"
                             % (self._name, st.error_code), st.error_code)

    # ==================================================================
    # Status (StateGet / MotionDoneGet/Wait / StatusBits / StatusWordGet)
    # ==================================================================

    def status_get(self):
        """One full status query (RMP StateGet + StatusBitsGet combined)."""
        line = self._mc.raw_command("STS %d" % self._index)
        if not line.startswith("STS"):
            raise CommunicationError("axis %s malformed STS reply: %r"
                                     % (self._name, line))
        fields = dict(re.findall(r"(\w+)=([^\s]+)", line))
        return AxisStatus(self._index, fields)

    StatusGet = status_get

    def StateGet(self):
        """CiA402 state-machine state (RMP StateGet)."""
        return self.status_get().state

    def SourceGet(self):
        """Error source (RMP SourceGet): 'AXIS' on fault, else 'NONE'."""
        return "AXIS" if self.status_get().fault else "NONE"

    def StatusWordGet(self):
        """Raw DS402 statusword (RMP StatusWordGet)."""
        return self.status_get().status_word

    def StatusBitGet(self, bitMask):
        """One statusword bit (RMP StatusBitGet; use RSIEventType members)."""
        return bool(self.status_get().status_word & (1 << int(bitMask)))

    def StatusBitsGet(self):
        """All status bits (RMP StatusBitsGet); returns the statusword."""
        return self.status_get().status_word

    def MotionDoneGet(self):
        """Whether the motion is complete (RMP MotionDoneGet):
        target reached and no fault."""
        st = self.status_get()
        if st.fault:
            raise FaultError("axis %s drive fault, err=0x%04X"
                             % (self._name, st.error_code), st.error_code)
        return st.target_reached

    def MotionDoneWait(self, waitTimeoutMilliseconds=-1):
        """Wait for motion completion (RMP MotionDoneWait; -1 = forever).
        Fault -> FaultError; timeout -> MotionTimeoutError; on completion
        returns the final position."""
        timeout = None if waitTimeoutMilliseconds is None or waitTimeoutMilliseconds < 0 \
            else waitTimeoutMilliseconds / 1000.0
        deadline = None if timeout is None else time.time() + timeout
        while True:
            st = self.status_get()
            if st.fault:
                raise FaultError("axis %s drive fault during motion, "
                                 "err=0x%04X"
                                 % (self._name, st.error_code), st.error_code)
            if st.target_reached:
                if self._last_target is not None:
                    got = self.CountsToUserUnits(st.position_counts - self._origin_offset)
                    if abs(got - self._last_target) <= self._tol_fine:
                        return got
                else:
                    return self.CountsToUserUnits(st.position_counts - self._origin_offset)
            if deadline is not None and time.time() > deadline:
                raise MotionTimeoutError(
                    "axis %s MotionDoneWait timed out (%s ms)"
                    % (self._name, waitTimeoutMilliseconds))
            time.sleep(0.1)

    # ==================================================================
    # Position and trajectory state
    # ==================================================================

    def ActualPositionGet(self):
        """Actual position (user units, origin offset applied)."""
        return self.CountsToUserUnits(self._read_raw_position() - self._origin_offset)

    def ActualPositionSet(self, position):
        """Redefine the current actual position's coordinate
        (RMP ActualPositionSet)."""
        self.PositionSet(position)

    def PositionSet(self, position):
        """Define the current actual position as the given user coordinate
        (RMP PositionSet). With the axis parked on a reference stop after
        power-up, PositionSet(0.0) completes the zero calibration."""
        raw = self._read_raw_position()
        self._origin_offset = raw - self.UserUnitsToCounts(position)

    def CommandPositionGet(self):
        """Commanded position (RMP CommandPositionGet): latest target sent."""
        return self.TargetPositionGet()

    def CommandPositionSet(self, position):
        self.PositionSet(position)

    def TargetPositionGet(self):
        """Most recent commanded target (RMP TargetPositionGet)."""
        if self._last_target is not None:
            return self._last_target
        return self.ActualPositionGet()

    def OriginPositionGet(self):
        """Origin offset in user units (RMP OriginPositionGet)."""
        return self.CountsToUserUnits(self._origin_offset)

    def OriginPositionSet(self, position):
        self._origin_offset = self.UserUnitsToCounts(position)

    def PositionErrorGet(self):
        """Following error: target - actual (RMP PositionErrorGet)."""
        return self.TargetPositionGet() - self.ActualPositionGet()

    def ActualVelocityGet(self):
        """Actual velocity (user units/s; the commanded stream rate - the
        drive-side velocity is available over SDO)."""
        st = self.status_get()
        return self.CountsToUserUnits(st.velocity_counts_s)

    def CommandVelocityGet(self):
        """Commanded stream velocity (RMP CommandVelocityGet)."""
        return self.ActualVelocityGet()

    def CommandAccelGet(self):
        _ni("CommandAccelGet", "firmware does not report instantaneous "
            "acceleration")

    def EncoderPositionGet(self, encoder=0):
        """Raw encoder position in counts (RMP EncoderPositionGet)."""
        return self._read_raw_position()

    def FramesToExecuteGet(self):
        return 0  # no streamed buffer

    # ==================================================================
    # Unit scaling (UserUnits)
    # ==================================================================

    def UserUnitsGet(self):
        """Counts per user unit. Default 1024 (1 unit = 1 mm)."""
        return self._counts_per_unit

    def UserUnitsSet(self, countsPerUserUnit):
        """Set counts per user unit (RMP UserUnitsSet).
        1024 -> 1 unit = 1 mm (default); 2048 -> 1 unit = 1 rev = 2 mm.
        Limits/velocity cap/tolerances rescale automatically so the physical
        values stay unchanged."""
        if countsPerUserUnit <= 0:
            raise RsiError("countsPerUserUnit must be positive")
        ratio = self._counts_per_unit / float(countsPerUserUnit)
        if self._soft_lo is not None:
            self._soft_lo *= ratio
            self._soft_hi *= ratio
        self._max_vel *= ratio
        self._def_vel *= ratio
        self._tol_fine *= ratio
        self._tol_coarse *= ratio
        if self._last_target is not None:
            self._last_target *= ratio
        self._counts_per_unit = float(countsPerUserUnit)

    def UserUnitsToCounts(self, userUnits):
        return int(round(userUnits * self._counts_per_unit))

    def CountsToUserUnits(self, counts):
        return counts / self._counts_per_unit

    # ==================================================================
    # Limits and action configuration
    # ==================================================================

    def SoftwarePosLimitTriggerValueGet(self):
        return self._soft_hi

    def SoftwarePosLimitTriggerValueSet(self, triggerValue):
        lo = self._soft_lo if self._soft_lo is not None else float("-inf")
        self.software_limits_set(lo, triggerValue)

    def SoftwareNegLimitTriggerValueGet(self):
        return self._soft_lo

    def SoftwareNegLimitTriggerValueSet(self, triggerValue):
        hi = self._soft_hi if self._soft_hi is not None else float("inf")
        self.software_limits_set(triggerValue, hi)

    def software_limits_set(self, low, high):
        """Set the software limits (user units)."""
        if low >= high:
            raise RsiError("software limit low must be less than high")
        self._soft_lo = float(low)
        self._soft_hi = float(high)

    def software_limits_get(self):
        """(low, high) in user units, or None when disabled."""
        if self._soft_lo is None:
            return None
        return (self._soft_lo, self._soft_hi)

    def software_limits_clear(self):
        """Disable the software limits (no travel-range checking)."""
        self._soft_lo = None
        self._soft_hi = None

    def max_velocity_set(self, v):
        """Velocity cap (user units/s, default 5 mm/s)."""
        self._max_vel = abs(float(v))

    def NegativeLimitGet(self):
        """Negative-limit triggered state (RMP NegativeLimitGet; no hardware
        switch - the software limit is used, with a 10-count steady-state
        tolerance). False when limits are disabled."""
        if self._soft_lo is None:
            return False
        return self.ActualPositionGet() <= self._soft_lo + abs(self.CountsToUserUnits(10))

    def PositiveLimitGet(self):
        """Positive-limit triggered state (RMP PositiveLimitGet; software
        limit based). False when limits are disabled."""
        if self._soft_hi is None:
            return False
        return self.ActualPositionGet() >= self._soft_hi - abs(self.CountsToUserUnits(10))

    def ErrorLimitTriggerValueGet(self):
        """Following-error protection threshold (fixed at 20000 counts in
        firmware; returned in user units)."""
        return self.CountsToUserUnits(20000)

    def ErrorLimitTriggerValueSet(self, triggerValue):
        _ni("ErrorLimitTriggerValueSet", "following-error threshold is fixed "
            "at 20000 counts in firmware")

    def ErrorLimitActionGet(self):
        return RSIAction.STOP

    def ErrorLimitActionSet(self, action):
        _ni("ErrorLimitActionSet", "following-error action is fixed to "
            "clamp-and-stop")

    def HardwareNegLimitActionGet(self):
        return RSIAction.NONE

    def HardwareNegLimitActionSet(self, action):
        _ni("HardwareNegLimitActionSet", "no hardware limit switches wired")

    def HardwarePosLimitActionGet(self):
        return RSIAction.NONE

    def HardwarePosLimitActionSet(self, action):
        _ni("HardwarePosLimitActionSet", "no hardware limit switches wired")

    def AmpFaultActionGet(self):
        return RSIAction.ABORT

    def AmpFaultActionSet(self, action):
        _ni("AmpFaultActionSet", "fault action is decided by the drive "
            "firmware")

    # ==================================================================
    # Homing (no home switch: manual homing procedure)
    # ==================================================================

    def Home(self, moveToZero=True):
        """RMP Home. No home switch on this machine, so automatic homing is
        not supported: use the manual procedure - jog slowly onto a limit
        with MoveVelocity (or push with power off), then PositionSet(0.0)."""
        _ni("Home", "no home switch; manual homing: touch the limit, "
            "then PositionSet(0.0)")

    def HomeStateGet(self):
        """Whether zero calibration has been done (RMP HomeStateGet)."""
        return self._home_state

    def HomeStateSet(self, homed):
        self._home_state = bool(homed)

    def HomeMethodGet(self):
        return self._home_method

    def HomeMethodSet(self, method):
        self._home_method = method

    def HomeOffsetGet(self):
        """Homing offset (RMP HomeOffsetGet), i.e. the current origin offset."""
        return self.OriginPositionGet()

    def HomeOffsetSet(self, offset):
        self.OriginPositionSet(offset)

    def HomeCancelSet(self, cancel):
        _ni("HomeCancelSet", "no automatic homing to cancel")

    # ==================================================================
    # Default trajectory parameters (RMP Default* family; accel recorded only)
    # ==================================================================

    def DefaultVelocityGet(self):
        return self._def_vel

    def DefaultVelocitySet(self, velocity):
        self._def_vel = abs(float(velocity))

    def DefaultAccelerationGet(self):
        return self._def_accel

    def DefaultAccelerationSet(self, acceleration):
        self._def_accel = abs(float(acceleration))  # recorded; firmware ramps

    def DefaultDecelerationGet(self):
        return self._def_decel

    def DefaultDecelerationSet(self, deceleration):
        self._def_decel = abs(float(deceleration))

    def DefaultJerkPercentGet(self):
        return self._def_jerk

    def DefaultJerkPercentSet(self, jerkPercent):
        self._def_jerk = float(jerkPercent)

    def DefaultPosition1Get(self):
        return self._def_pos1

    def DefaultPosition1Set(self, position1):
        self._def_pos1 = float(position1)

    def DefaultPosition2Get(self):
        return self._def_pos2

    def DefaultPosition2Set(self, position2):
        self._def_pos2 = float(position2)

    def DefaultRelativeIncrementGet(self):
        return self._def_rel_inc

    def DefaultRelativeIncrementSet(self, relativeIncrement):
        self._def_rel_inc = float(relativeIncrement)

    # ==================================================================
    # Settling criteria
    # ==================================================================

    def PositionToleranceFineGet(self):
        return self._tol_fine

    def PositionToleranceFineSet(self, tolerance):
        self._tol_fine = abs(float(tolerance))

    def PositionToleranceCoarseGet(self):
        return self._tol_coarse

    def PositionToleranceCoarseSet(self, tolerance):
        self._tol_coarse = abs(float(tolerance))

    def VelocityToleranceGet(self):
        return self._vel_tol

    def VelocityToleranceSet(self, tolerance):
        self._vel_tol = abs(float(tolerance))

    def SettlingTimeGet(self):
        return self._settle_time

    def SettlingTimeSet(self, time_s):
        self._settle_time = abs(float(time_s))

    # ==================================================================
    # Miscellaneous configuration
    # ==================================================================

    def OperationModeGet(self):
        """Operating mode (RMP OperationModeGet): fixed CSP."""
        return RSIOperationMode.CSP

    def OperationModeSet(self, mode):
        if mode != RSIOperationMode.CSP:
            _ni("OperationModeSet", "only CSP is reliable on this drive "
                "(software-ESC limitation)")

    def UserLabelGet(self):
        return self._user_label

    def UserLabelSet(self, userLabel):
        self._user_label = str(userLabel)

    def FeedRateGet(self):
        """Velocity override (RMP FeedRateGet), 1.0 = 100%. Effective."""
        return self._feed_rate

    def FeedRateSet(self, rate):
        if not (0.0 <= rate <= 2.0):
            raise RsiError("FeedRate range is 0.0 ~ 2.0")
        self._feed_rate = float(rate)

    def MotionDelayGet(self):
        """Delay before motion starts, seconds (RMP MotionDelayGet)."""
        return self._motion_delay

    def MotionDelaySet(self, seconds):
        self._motion_delay = max(0.0, float(seconds))

    def MotionIdGet(self):
        return self._motion_id

    def MotionIdSet(self, motion_id):
        self._motion_id = int(motion_id)

    def StopTimeGet(self):
        return self._stop_time

    def StopTimeSet(self, seconds):
        self._stop_time = abs(float(seconds))

    def EStopTimeGet(self):
        return self._estop_time

    def EStopTimeSet(self, seconds):
        self._estop_time = abs(float(seconds))

    def EStopDecelerationGet(self):
        return self._estop_decel

    def EStopDecelerationSet(self, decel):
        self._estop_decel = abs(float(decel))

    def NumberGet(self):
        """Axis index (RMP NumberGet)."""
        return self._index

    def AxisCountGet(self):
        return self._mc.axis_count_get()

    def name_get(self):
        """Configured machine axis name ('AX0' etc. by default)."""
        return self._name

    # ==================================================================
    # IO state
    # ==================================================================

    def AmpEnableGet(self):
        """Enable output state (RMP AmpEnableGet)."""
        return self.status_get().enabled

    def AmpFaultGet(self):
        """Drive fault state (RMP AmpFaultGet)."""
        return self.status_get().fault

    def HomeLimitGet(self):
        _ni("HomeLimitGet", "no home switch wired")

    def HomeSwitchGet(self):
        _ni("HomeSwitchGet", "no home switch wired")

    def DedicatedInGet(self, n):
        _ni("DedicatedInGet", "drive dedicated IO is not wired to the master")

    def DedicatedOutGet(self, n):
        _ni("DedicatedOutGet", "drive dedicated IO is not wired to the master")

    def DigitalInGet(self, n):
        _ni("DigitalInGet", "general IO is not wired to the master")

    def DigitalOutGet(self, n):
        _ni("DigitalOutGet", "general IO is not wired to the master")

    def DigitalOutSet(self, n, value):
        _ni("DigitalOutSet", "general IO is not wired to the master")

    def AnalogInGet(self, channel):
        _ni("AnalogInGet", "analog inputs are not wired to the master")

    # ==================================================================
    # Gearing / cam / filters / backlash: explicitly not supported
    # ==================================================================

    def GearingEnable(self, *args, **kwargs):
        _ni("GearingEnable", "no electronic gearing in a single-axis CSP "
            "system")

    def GearingDisable(self):
        _ni("GearingDisable", "no electronic gearing in a single-axis CSP "
            "system")

    def GearingRatioChange(self, *args):
        _ni("GearingRatioChange", "no electronic gearing in a single-axis "
            "CSP system")

    def MoveCamLinear(self, *args, **kwargs):
        _ni("MoveCamLinear", "no electronic cam in firmware")

    def MoveCamCubic(self, *args, **kwargs):
        _ni("MoveCamCubic", "no electronic cam in firmware")

    def FilterCoeffGet(self, coeff, gainTable=0):
        _ni("FilterCoeffGet", "loop tuning lives inside the drive, not the "
            "master")

    def FilterCoeffSet(self, coeff, gainTable, value):
        _ni("FilterCoeffSet", "loop tuning lives inside the drive, not the "
            "master")

    def BacklashWidthGet(self):
        _ni("BacklashWidthGet", "no backlash compensation in firmware")

    def BacklashWidthSet(self, width):
        _ni("BacklashWidthSet", "no backlash compensation in firmware")

    # ==================================================================
    # snake_case aliases (Python convention; kept for compatibility)
    # ==================================================================

    move_absolute = MoveTrapezoidal
    move_trapezoidal = MoveTrapezoidal
    move_scurve = MoveSCurve
    move_relative = MoveRelative
    move_velocity = MoveVelocity
    stop = Stop
    resume = Resume
    abort = Abort
    estop = EStop
    amp_enable_set = AmpEnableSet
    clear_faults = ClearFaults
    state_get = StateGet
    status_word_get = StatusWordGet
    motion_done_get = MotionDoneGet
    motion_done_wait = MotionDoneWait
    wait_done = MotionDoneWait
    actual_position_get = ActualPositionGet
    actual_position_counts_get = EncoderPositionGet
    actual_velocity_get = ActualVelocityGet
    target_position_get = TargetPositionGet
    position_error_get = PositionErrorGet
    position_set = PositionSet
    set_origin_here = PositionSet
    user_units_get = UserUnitsGet
    user_units_set = UserUnitsSet
    user_units_to_counts = UserUnitsToCounts
    counts_to_user_units = CountsToUserUnits
    negative_limit_get = NegativeLimitGet
    positive_limit_get = PositiveLimitGet

    def error_code_get(self):
        """CiA402 error code 0x603F (0 = no error; the drive may latch the
        most recent code - check StatusGet().fault for the live state)."""
        return self.status_get().error_code
