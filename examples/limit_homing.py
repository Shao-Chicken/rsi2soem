#!/usr/bin/env python3
"""Manual homing without a home switch (relative encoders lose their raw
reference at every drive power cycle).

Procedure per axis: crawl slowly onto the negative-end hard stop while
watching the position; when it stops moving (stall), halt, back off a
little, and define the stop as user zero.

Usage: python limit_homing.py COM12 Z
"""

import sys
import time

from rsi2soem import MotionController

CRAWL_MM_S = 0.5          # gentle: 0.5 mm/s
POLL_S = 0.3
STALL_POLLS = 2           # consecutive polls with < 5 counts of change
BACKOFF_MM = 2.0


def home_on_negative_stop(ax):
    print("homing %s: crawling negative at %.1f mm/s ..."
          % (ax.name_get(), CRAWL_MM_S))
    ax.AmpEnableSet(True)
    ax.MoveVelocity(-CRAWL_MM_S)
    prev = ax.EncoderPositionGet()
    stall = 0
    while True:
        time.sleep(POLL_S)
        pos = ax.EncoderPositionGet()
        stall = stall + 1 if abs(pos - prev) < 5 else 0
        prev = pos
        if stall >= STALL_POLLS:
            break
    ax.Stop()
    ax.Resume()
    print("  stop found at raw %d; backing off %.1f mm" % (pos, BACKOFF_MM))
    ax.MoveRelative(BACKOFF_MM, 1.0)
    ax.MotionDoneWait(10000)
    # define the hard stop as zero: we are BACKOFF_MM above it
    ax.PositionSet(BACKOFF_MM)
    ax.HomeStateSet(True)
    print("  %s homed: zero at the negative stop, now at %.2f mm"
          % (ax.name_get(), ax.ActualPositionGet()))


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else None
    names = sys.argv[2:] or ["Z"]
    with MotionController.create(port) as mc:
        for name in names:
            home_on_negative_stop(mc.AxisGet(name))
        mc.NetworkShutdown()
