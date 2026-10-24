"""NetworkNode: CoE SDO read/write on one EtherCAT node (RMP NetworkNode).

Multi-channel ("one-to-four") drives: a single physical node hosts several
CiA402 channels; channel ch lives at object base 0x6000 + 0x800*ch. The
convenience getters (status_word / error_code / ...) apply the channel offset
automatically from the node number; SDORead/SDOWrite take raw indices (use
0x6841 etc. explicitly for a specific channel).

CAUTION: the drive's software ESC is sensitive to SDO traffic; occasional
failures raise CommunicationError, and heavy continuous SDO polling can
disturb OP (the firmware auto-recovers). Use for configuration and
diagnostics, not for high-rate loops.
"""

import re

from .exceptions import CommunicationError, RsiError


class NetworkNode(object):
    """One EtherCAT node (drive): CoE SDO access (RMP NetworkNode)."""

    def __init__(self, controller, index):
        self._mc = controller
        self._index = index

    def NumberGet(self):
        return self._index

    def SDORead(self, index, subindex=0, size=4, signed=False):
        """Read an SDO object (RMP NetworkNode SDO read; raw index).
        Returns the integer value; missing object/failure raises
        CommunicationError."""
        line = self._mc.raw_command("SDORD %d %d %d" % (self._index, index,
                                                        subindex))
        if line.startswith("ERR"):
            raise CommunicationError("SDO read 0x%04X:%d failed: %s"
                                     % (index, subindex, line))
        m = re.search(r"val=0x([0-9A-Fa-f]+)", line)
        if not m:
            raise CommunicationError("malformed SDO reply: %r" % line)
        val = int(m.group(1), 16)
        if signed and size in (1, 2, 4):
            sign_bit = 1 << (size * 8 - 1)
            if val & sign_bit:
                val -= 1 << (size * 8)
        return val

    def SDOReadSigned(self, index, subindex=0, size=4):
        return self.SDORead(index, subindex, size, signed=True)

    def SDOWrite(self, index, subindex, size, value):
        """Write an SDO object (RMP NetworkNode SDO write; raw index).
        ``size``: 1/2/4 bytes."""
        if size not in (1, 2, 4):
            raise RsiError("SDOWrite size must be 1/2/4 bytes")
        line = self._mc.raw_command("SDOWR %d %d %d %d %d"
                                    % (self._index, index, subindex, size,
                                       value & 0xFFFFFFFF))
        if line.startswith("ERR"):
            raise CommunicationError("SDO write 0x%04X:%d failed: %s"
                                     % (index, subindex, line))

    def SDOU16Read(self, index, subindex=0):
        return self.SDORead(index, subindex, size=2)

    def SDOU8Read(self, index, subindex=0):
        return self.SDORead(index, subindex, size=1)

    # Convenience reads of common CiA402 objects.
    # Multi-channel drives: node number == logical axis; the channel offset
    # 0x800*(index % channels_per_slave) is applied (0x6041 -> 0x6841 etc.).
    # Cascaded single-channel drives keep offset 0 - behaviour unchanged.
    def _sdo_idx(self, base):
        cps = max(1, getattr(self._mc, "_ch_per_slave", 1))
        return base + 0x800 * (self._index % cps)

    def status_word(self):
        return self.SDORead(self._sdo_idx(0x6041), 0, size=2)

    def error_code(self):
        return self.SDORead(self._sdo_idx(0x603F), 0, size=2)

    def position_actual(self):
        return self.SDORead(self._sdo_idx(0x6064), 0, size=4, signed=True)

    def velocity_actual(self):
        return self.SDORead(self._sdo_idx(0x606C), 0, size=4, signed=True)

    def mode_display(self):
        return self.SDORead(self._sdo_idx(0x6061), 0, size=1, signed=True)

    def digital_inputs(self):
        """CiA402 digital inputs 0x60FD (bit0 = negative limit,
        bit1 = positive limit - read 0 when no switches are wired)."""
        return self.SDORead(self._sdo_idx(0x60FD), 0, size=4)

    def supported_drive_modes(self):
        return self.SDORead(self._sdo_idx(0x6502), 0, size=4)
