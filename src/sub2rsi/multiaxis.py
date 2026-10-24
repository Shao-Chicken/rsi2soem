"""MultiAxis: coordinated start across axes (RMP MultiAxis).

NOTE: each axis runs its own independent CSP stream, so a multi-axis move is
"start all at once, each arrives on its own" - NOT strict interpolation.
Good for "fire together, all arrive" coordination.
"""

import time

from .constants import CiaState
from .exceptions import RsiError, _ni


class MultiAxis(object):
    """Multi-axis coordination object (RMP MultiAxis, RapidCodeMotion)."""

    def __init__(self, controller, axis_indices):
        self._mc = controller
        self._indices = tuple(axis_indices)
        self._axes = [controller.axis_get(i) for i in self._indices]
        self._mapped = True

    # -- mapping (Map / Unmap / IsMapped) --

    def Map(self):
        self._mapped = True

    def Unmap(self):
        self._mapped = False

    def IsMapped(self):
        return self._mapped

    def AxisCountGet(self):
        return len(self._axes)

    def NumberGet(self):
        return self._indices[0] if self._indices else -1

    def _require_mapped(self):
        if not self._mapped:
            raise RsiError("MultiAxis not mapped (call Map() first)")

    # -- enable / faults --

    def AmpEnableSet(self, enable, ampActiveTimeoutMilliseconds=500):
        """Enable / disable every axis (RMP AmpEnableSet)."""
        for ax in self._axes:
            ax.AmpEnableSet(enable, ampActiveTimeoutMilliseconds)
        return 0

    def ClearFaults(self):
        for ax in self._axes:
            ax.ClearFaults()

    # -- motion: all axes start together, no interpolation --

    def _targets(self, positions):
        if len(positions) != len(self._axes):
            raise RsiError("position count %d does not match axis count %d"
                           % (len(positions), len(self._axes)))
        return list(positions)

    def MoveTrapezoidal(self, positions, vel=None, accel=None, decel=None,
                        jerkPct=None):
        """Multi-axis absolute move (RMP MoveTrapezoidal): simultaneous start.
        ``vel``: a single value (same for all) or a per-axis list."""
        self._require_mapped()
        positions = self._targets(positions)
        vels = self._expand_vel(vel)
        for ax, p, v in zip(self._axes, positions, vels):
            ax.MoveTrapezoidal(p, v)

    def MoveSCurve(self, positions, vel=None, accel=None, decel=None,
                   jerkPct=None):
        self.MoveTrapezoidal(positions, vel, accel, decel, jerkPct)

    def MoveRelative(self, deltas, vel=None, accel=None, decel=None,
                     jerkPct=None):
        """Multi-axis relative move (RMP MoveRelative)."""
        self._require_mapped()
        deltas = self._targets(deltas)
        vels = self._expand_vel(vel)
        for ax, d, v in zip(self._axes, deltas, vels):
            ax.MoveRelative(d, v)

    def MoveVelocity(self, velocities, accel=None):
        """Multi-axis velocity jog (RMP MoveVelocity): per-axis list."""
        self._require_mapped()
        velocities = self._targets(velocities)
        for ax, v in zip(self._axes, velocities):
            ax.MoveVelocity(v)

    def _expand_vel(self, vel):
        if vel is None:
            return [None] * len(self._axes)
        if isinstance(vel, (list, tuple)):
            if len(vel) != len(self._axes):
                raise RsiError("velocity count %d does not match axis count %d"
                               % (len(vel), len(self._axes)))
            return list(vel)
        return [vel] * len(self._axes)

    # -- stop --

    def Stop(self):
        for ax in self._axes:
            ax.Stop()

    def Resume(self):
        for ax in self._axes:
            ax.Resume()

    def Abort(self):
        self.Stop()

    def EStop(self):
        self.Stop()

    def EStopAbort(self):
        self.Stop()

    # -- status --

    def StateGet(self):
        """Worst axis state (FAULT > QUICK_STOP > others; RMP StateGet)."""
        states = [ax.StateGet() for ax in self._axes]
        for s in (CiaState.FAULT, CiaState.FAULT_REACTION_ACTIVE,
                  CiaState.QUICK_STOP_ACTIVE):
            if s in states:
                return s
        if all(s == CiaState.OPERATION_ENABLED for s in states):
            return CiaState.OPERATION_ENABLED
        return states[0] if states else CiaState.UNKNOWN

    def MotionDoneGet(self):
        return all(ax.MotionDoneGet() for ax in self._axes)

    def MotionDoneWait(self, waitTimeoutMilliseconds=-1):
        """Wait for every axis to arrive (RMP MotionDoneWait)."""
        deadline = None
        if waitTimeoutMilliseconds is not None and waitTimeoutMilliseconds >= 0:
            deadline = time.time() + waitTimeoutMilliseconds / 1000.0
        results = []
        for ax in self._axes:
            remain = -1 if deadline is None else max(
                1, int((deadline - time.time()) * 1000))
            results.append(ax.MotionDoneWait(remain))
        return results

    def ActualPositionGet(self):
        """Per-axis actual positions (user units)."""
        return [ax.ActualPositionGet() for ax in self._axes]

    def ActualVelocityGet(self):
        return [ax.ActualVelocityGet() for ax in self._axes]

    def PositionErrorGet(self):
        return [ax.PositionErrorGet() for ax in self._axes]

    def FeedRateSet(self, rate):
        for ax in self._axes:
            ax.FeedRateSet(rate)

    # streamed/interpolated families: not supported
    def MovePVT(self, *args, **kwargs):
        _ni("MultiAxis.MovePVT", "no streamed-point buffer in firmware")

    def MovePT(self, *args, **kwargs):
        _ni("MultiAxis.MovePT", "no streamed-point buffer in firmware")
