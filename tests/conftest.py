"""Test fixtures: a fake serial port emulating the STM32 firmware's line
protocol, so the whole library can be unit-tested without hardware.

Protocol recap (115200 8N1, '\n'-terminated ASCII):
    host -> AXES | EN/Dis/RESET/HALT/CONT <ax> | VEL <ax> <c/s>
            AMOV <ax> <pos> <vel> | RMOV <ax> <d> <vel> | STS <ax>
            SDORD <ax> <idx> <sub> | SDOWR <ax> <idx> <sub> <size> <val>
            ODLIST [ax] | REINIT
    fw   -> OK / ERR ... / STS ... / SDO ... / AXES n=N
            (plus asynchronous 'EVT ...' lines, ignored by the library)
"""

import pytest

from sub2rsi import MotionController


def pytest_addoption(parser):
    parser.addoption("--ecat-port", default=None,
                     help="serial port of the real STM32 master (e.g. COM12); "
                          "enables the hardware-in-the-loop tests")


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "hardware: needs the real machine (pass --ecat-port)")


@pytest.fixture(scope="session")
def ecat_port(request):
    port = request.config.getoption("--ecat-port")
    if not port:
        pytest.skip("no --ecat-port given; hardware tests skipped")
    return port


class FakeDrive(object):
    """One logical axis' state inside the fake firmware."""

    def __init__(self):
        self.enabled = False
        self.fault = False
        self.pos = 0            # raw counts
        self.tgt = 0
        self.vel = 0            # commanded counts/s
        self.err = 0
        self.halt = False


class FakeSerial(object):
    """Minimal pyserial stand-in: write() parses a command and queues the
    reply; readline() pops it. Motion completes instantly (target jumped)."""

    def __init__(self, port, baud, n_axes=3, channels=3):
        self.port = port
        self.baud = baud
        self.axes = [FakeDrive() for _ in range(n_axes)]
        self.channels = channels
        self.sdo_store = {}
        self._rx = bytearray()
        self.sent = []          # every command line, for assertions
        self.closed = False

    # -- pyserial surface -------------------------------------------------

    def write(self, data):
        line = data.decode().strip()
        self.sent.append(line)
        self._handle(line)
        return len(data)

    def readline(self):
        if not self._rx:
            return b""
        end = self._rx.find(b"\n")
        if end < 0:
            out = bytes(self._rx)
            self._rx.clear()
            return out
        out = bytes(self._rx[:end + 1])
        del self._rx[:end + 1]
        return out

    def reset_input_buffer(self):
        self._rx.clear()

    def close(self):
        self.closed = True

    # -- firmware emulation ------------------------------------------------

    def _push(self, line):
        self._rx += (line + "\n").encode()

    def _handle(self, line):
        parts = line.split()
        cmd = parts[0].upper()
        if cmd == "AXES":
            self._push("AXES n=%d" % len(self.axes))
        elif cmd in ("EN", "DIS"):
            ax = self.axes[int(parts[1])]
            ax.enabled = (cmd == "EN")
            self._push("OK")
        elif cmd == "HALT":
            self.axes[int(parts[1])].halt = True
            self.axes[int(parts[1])].vel = 0
            self._push("OK")
        elif cmd == "CONT":
            self.axes[int(parts[1])].halt = False
            self._push("OK")
        elif cmd == "RESET":
            self.axes[int(parts[1])].fault = False
            self._push("OK")
        elif cmd == "VEL":
            ax = self.axes[int(parts[1])]
            if ax.halt:
                self._push("ERR halt active, use CONT")
            else:
                ax.vel = int(parts[2])
                self._push("OK")
        elif cmd in ("AMOV", "RMOV"):
            ax = self.axes[int(parts[1])]
            if ax.halt:
                self._push("ERR halt active, use CONT")
            elif not ax.enabled:
                self._push("ERR not enabled")
            else:
                ax.tgt = int(parts[2]) if cmd == "AMOV" else ax.pos + int(parts[2])
                ax.pos = ax.tgt      # instant arrival
                ax.vel = 0
                self._push("OK")
                self._push("EVT MOVE_DONE ax=%s" % parts[1])
        elif cmd == "STS":
            i = int(parts[1])
            ax = self.axes[i]
            sw = 0x1637 if ax.enabled else 0x0250
            if ax.fault:
                sw = 0x0618
            self._push("STS ax=%d sw=0x%04X rdy=%d on=%d en=%d fault=%d "
                       "warn=0 tgt=1 pos=%d vel=%d err=0x%04X mode=%s al=OP "
                       "wkc=3"
                       % (i, sw, 1, 1 if ax.enabled else 0,
                          1 if ax.enabled else 0, 1 if ax.fault else 0,
                          ax.pos, ax.vel, ax.err, "CSP" if ax.enabled
                          else "NONE"))
        elif cmd == "SDORD":
            idx = int(parts[2])
            # multi-channel probe objects: 0x6841 / 0x7041 exist iff the
            # fake drive hosts that many channels
            if idx in (0x6841, 0x7041):
                ch = 1 if idx == 0x6841 else 2
                if ch < self.channels:
                    self._push("SDO ax=0 idx=0x%04X sub=0 val=0x00000250 "
                               "size=4 wkc=1" % idx)
                else:
                    self._push("ERR sdo timeout")
                return
            val = self.sdo_store.get(idx)
            if val is None:
                defaults = {0x6041: 0x0250, 0x6064: 0, 0x6061: 8,
                            0x606C: 0, 0x603F: 0, 0x6502: 0x3AD,
                            0x6841: 0x0250, 0x7041: 0x0250}
                val = defaults.get(idx)
            if val is None:
                self._push("ERR sdo timeout")
            else:
                self._push("SDO ax=%s idx=0x%04X sub=%s val=0x%08X size=4 "
                           "wkc=1" % (parts[1], idx, parts[3], val))
        elif cmd == "SDOWR":
            idx = int(parts[2])
            val = int(parts[5])
            self.sdo_store[idx] = val
            self._push("OK")
        else:
            self._push("ERR unknown command")


@pytest.fixture
def fake_serial():
    return FakeSerial


@pytest.fixture
def machine_axes():
    """The reference 3-axis machine: Z/X/Y with measured travels."""
    return {0: ("Z", (0.0, 91.5)),
            1: ("X", (0.0, 110.0)),
            2: ("Y", (0.0, 111.0))}


@pytest.fixture
def mc(machine_axes):
    """A MotionController on the fake link, machine axes configured."""
    return MotionController("COM_FAKE", axes_config=machine_axes,
                            serial_factory=FakeSerial)


@pytest.fixture
def mc_generic():
    """A MotionController without a machine config (generic AX0.. names,
    limits disabled)."""
    return MotionController("COM_FAKE", serial_factory=FakeSerial)
