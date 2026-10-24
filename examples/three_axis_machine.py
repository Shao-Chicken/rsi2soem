#!/usr/bin/env python3
"""Three-axis machine setup: named axes, software limits, group motion.

Reference machine: one MOONS' one-to-four drive, ch0=Z ch1=X ch2=Y,
2 mm ball screws (1024 counts = 1 mm), travels measured by gentle
limit probing (see docs/api-reference.md, section "Homing").

Usage: python three_axis_machine.py [COM12]
"""

import sys

from sub2rsi import MotionController

# axis index -> (name, (soft_low, soft_high)) in user units (default: mm)
MACHINE_AXES = {
    0: ("Z", (0.0, 91.5)),    # zero at the bottom stop, up is positive
    1: ("X", (0.0, 110.0)),   # zero at the left stop, right is positive
    2: ("Y", (0.0, 111.0)),   # zero at the top stop, down is positive
}

port = sys.argv[1] if len(sys.argv) > 1 else None

with MotionController.create(port, axes_config=MACHINE_AXES) as mc:
    # Z/X/Y names and the z_axis/x_axis/y_axis shortcuts are now live
    z, x, y = mc.z_axis, mc.x_axis, mc.y_axis

    for ax in (z, x, y):
        print("axis %s: sw=0x%04X pos=%.2f mm limits=%s"
              % (ax.name_get(), ax.StatusWordGet(), ax.ActualPositionGet(),
                 ax.software_limits_get()))

    # enable everything at once, then start a coordinated (non-interpolated)
    # move on all three axes
    xyz = mc.MultiAxisGet()
    xyz.AmpEnableSet(True)
    xyz.MoveTrapezoidal([5.0, 10.0, 10.0], 2.0)
    print("arrived:", ["%.2f" % p for p in xyz.MotionDoneWait(30000)])

    xyz.MoveTrapezoidal([0.0, 0.0, 0.0], 2.0)
    xyz.MotionDoneWait(30000)
    xyz.AmpEnableSet(False)
    print("done.")
