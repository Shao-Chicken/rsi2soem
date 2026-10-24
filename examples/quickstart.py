#!/usr/bin/env python3
"""Quickstart: connect, enable one axis, move it, read it back.

Usage: python quickstart.py [COM12]
"""

import sys

from sub2rsi import MotionController

port = sys.argv[1] if len(sys.argv) > 1 else None   # None = auto-detect

with MotionController.create(port) as mc:
    print("firmware:", mc.FirmwareVersionGet())
    print("library :", mc.VersionGet())
    print("axes    :", mc.AxisCountGet(),
          "| channels/node:", mc.ChannelsPerSlaveGet(),
          "| network:", mc.NetworkStateGet())

    ax = mc.AxisGet(0)
    ax.PositionSet(0.0)               # define the current spot as zero
    ax.AmpEnableSet(True)

    ax.MoveTrapezoidal(1.0, 1.0)      # +1 user unit at 1 unit/s
    x = ax.MotionDoneWait(15000)
    print("moved to %.3f" % x)

    ax.MoveRelative(-1.0, 1.0)        # back again
    print("back at %.3f" % ax.MotionDoneWait(15000))

    ax.AmpEnableSet(False)
    print("done.")
