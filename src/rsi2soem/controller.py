"""MotionController: serial link, EtherCAT network state, object factories.

Mirrors RSI::RapidCode::MotionController.
"""

import re
import time

from .constants import (CMD_TIMEOUT, DEFAULT_AXES_CONFIG, FIRMWARE_VERSION,
                        READ_TIMEOUT, SAMPLE_RATE_HZ)
from .exceptions import (AxisNotPresentError, CommunicationError,
                         NetworkError, RsiError, _ni)
from . import __version__


class MotionController(object):
    """Master controller: serial connection, network state, axis/node access."""

    # Lines starting with these prefixes are valid replies; EVT lines are
    # asynchronous events and are skipped while waiting for a reply.
    _REPLY_PREFIXES = ("OK", "ERR", "STS", "SDO", "AXES")

    def __init__(self, port, baud=115200, axes_config=None, serial_factory=None):
        """
        :param port: serial device ('COM12', '/dev/ttyUSB0', ...).
        :param baud: line baud rate (firmware uses 115200 8N1).
        :param axes_config: optional machine axis table
            ``{index: (name, (soft_lo, soft_hi))}`` in user units (mm with the
            default scaling). Without it axes are named AX0..AXn and software
            limits stay disabled until ``Axis.software_limits_set``.
        :param serial_factory: test hook, a callable(port, baud) returning an
            object with write/readline/close/reset_input_buffer.
        """
        if serial_factory is None:
            import serial  # pyserial, imported lazily so tests can mock
            serial_factory = lambda p, b: serial.Serial(p, b, timeout=READ_TIMEOUT)
        self._ser = serial_factory(port, baud)
        time.sleep(0.3)
        if hasattr(self._ser, "reset_input_buffer"):
            self._ser.reset_input_buffer()
        self.port = port
        self._axes = {}
        self._multiaxes = {}
        self._nodes = {}
        self._axes_config = (DEFAULT_AXES_CONFIG if axes_config is None
                             else dict(axes_config))
        self._name_to_index = {name: ix for ix, (name, _)
                               in self._axes_config.items()}
        self._axis_count = self._query_axis_count()
        # Multi-channel ("one-to-four") drives: one EtherCAT node hosting
        # several CiA402 channels; channel ch objects live at index+0x800*ch.
        # Probed once; NetworkNode convenience getters apply the offset.
        # Cascaded single-channel drives keep 1 (node == slave).
        self._ch_per_slave = self._detect_channels()

    # ------------------------------------------------------------------
    # Construction (RMP MotionController::Create / Get)
    # ------------------------------------------------------------------

    @classmethod
    def create(cls, port=None, baud=115200, axes_config=None):
        """Create and connect. Without ``port`` the first CH340 / USB-serial
        adapter is picked automatically."""
        if port is None:
            port = cls._find_port()
        return cls(port, baud, axes_config=axes_config)

    Create = create

    @staticmethod
    def _find_port():
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        if not ports:
            raise CommunicationError("no serial ports found - check the "
                                     "CH340/USB-serial connection")
        for p in ports:
            d = (p.description or "").lower()
            if "ch340" in d or "usb serial" in d or "usb-serial" in d:
                return p.device
        return ports[0].device

    def _query_axis_count(self):
        line = self.raw_command("AXES")
        m = re.search(r"n=(\d+)", line)
        if not m:
            raise CommunicationError("malformed AXES reply: %r" % line)
        return int(m.group(1))

    def _detect_channels(self):
        """Probe the CiA402 channel count inside one node (one-to-four drive).

        Tries 0x6841 (ch1 statusword) and 0x7041 (ch2 statusword) in turn.
        A missing object makes the drive stay silent; the firmware bounds the
        transaction to ~30 ms - far below the 100 ms SM watchdog, so this
        one-shot probe is safe even in OP."""
        n = 1
        for idx in (0x6841, 0x7041):
            line = self.raw_command("SDORD 0 %d 0" % idx)
            if line.startswith("ERR") or "wkc=0" in line:
                break
            n += 1
        return n

    def channels_per_slave_get(self):
        """CiA402 channels hosted by one EtherCAT node (1 = single-channel)."""
        return self._ch_per_slave

    ChannelsPerSlaveGet = channels_per_slave_get

    # ------------------------------------------------------------------
    # Object factories (AxisGet / MultiAxisGet / NetworkNodeGet / IOGet)
    # ------------------------------------------------------------------

    def _resolve_axis_index(self, axis):
        """``axis`` may be an index 0..N-1 or a configured name (any case)."""
        if isinstance(axis, str):
            key = axis.strip().upper()
            if key not in self._name_to_index:
                raise AxisNotPresentError(
                    "unknown axis name %r (configured: %s)"
                    % (axis, "/".join(sorted(self._name_to_index)) or "none"))
            axis = self._name_to_index[key]
        axis = int(axis)
        if axis < 0 or axis >= self._axis_count:
            raise AxisNotPresentError(
                "axis %d not present: only %d axes on the chain"
                % (axis, self._axis_count))
        return axis

    def axis_get(self, axis=0):
        """Get an Axis object (RMP AxisGet). ``axis``: index or name."""
        from .axis import Axis
        ix = self._resolve_axis_index(axis)
        if ix not in self._axes:
            self._axes[ix] = Axis(self, ix)
        return self._axes[ix]

    AxisGet = axis_get

    def axis_count_get(self):
        """Number of axes (RMP AxisCountGet)."""
        return self._axis_count

    AxisCountGet = axis_count_get

    def multi_axis_get(self, axes=None):
        """Get a MultiAxis group (RMP MultiAxisGet).
        ``axes``: list of indices/names; default = every axis on the chain."""
        from .multiaxis import MultiAxis
        if axes is None:
            axes = list(range(self._axis_count))
        resolved = tuple(self._resolve_axis_index(a) for a in axes)
        if resolved not in self._multiaxes:
            self._multiaxes[resolved] = MultiAxis(self, resolved)
        return self._multiaxes[resolved]

    MultiAxisGet = multi_axis_get
    LoadExistingMultiAxis = multi_axis_get

    def network_node_get(self, node=0):
        """Get an EtherCAT node object (RMP NetworkNodeGet) for SDO access."""
        from .node import NetworkNode
        ix = self._resolve_axis_index(node)
        if ix not in self._nodes:
            self._nodes[ix] = NetworkNode(self, ix)
        return self._nodes[ix]

    NetworkNodeGet = network_node_get
    IOGet = network_node_get  # IO goes through the same node object

    # Named-axis shortcuts (work when axes_config assigns these names)
    @property
    def z_axis(self):
        return self.axis_get("Z")

    @property
    def x_axis(self):
        return self.axis_get("X")

    @property
    def y_axis(self):
        return self.axis_get("Y")

    # ------------------------------------------------------------------
    # Network (NetworkStart / NetworkStateGet / NetworkShutdown)
    # ------------------------------------------------------------------

    def network_state_get(self):
        """EtherCAT AL state string: 'OP' when every axis is operational."""
        return self.axis_get(0).status_get().al_state

    NetworkStateGet = network_state_get

    def network_wait_op(self, timeout=15.0):
        """Wait until the network reaches OP (RMP NetworkStart semantics)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                ok = all(self.axis_get(i).status_get().al_state == "OP"
                         for i in range(self._axis_count))
                if ok:
                    return
            except (CommunicationError, AxisNotPresentError):
                pass
            time.sleep(0.3)
        raise NetworkError("timed out (%g s) waiting for EtherCAT OP" % timeout)

    NetworkStart = network_wait_op

    def network_shutdown(self):
        """RMP NetworkShutdown: disable every axis (the link stays up)."""
        for i in range(self._axis_count):
            try:
                self.axis_get(i).AmpEnableSet(False)
            except RsiError:
                pass

    NetworkShutdown = network_shutdown

    # ------------------------------------------------------------------
    # Information (SampleRateGet / FirmwareVersionGet / ...)
    # ------------------------------------------------------------------

    def sample_rate_get(self):
        """Firmware control cycle rate in Hz (RMP SampleRateGet)."""
        return SAMPLE_RATE_HZ

    SampleRateGet = sample_rate_get

    def firmware_version_get(self):
        return FIRMWARE_VERSION

    FirmwareVersionGet = firmware_version_get

    def version_get(self):
        """This API library's version (RMP VersionGet)."""
        return "rsi2soem %s (RMP-compatible)" % __version__

    VersionGet = version_get

    def serial_number_get(self):
        _ni("SerialNumberGet", "firmware does not assign a serial number")

    SerialNumberGet = serial_number_get

    # ------------------------------------------------------------------
    # Error log (ErrorLogCountGet/ErrorLogGet/ErrorLogClear)
    # ------------------------------------------------------------------

    def error_log_count_get(self):
        return 0  # errors are raised as exceptions, not parked in a log

    ErrorLogCountGet = error_log_count_get

    def error_log_get(self):
        return None

    ErrorLogGet = error_log_get

    def error_log_clear(self):
        pass

    ErrorLogClear = error_log_clear

    def throw_exceptions(self, state):
        """RMP ThrowExceptions: this library always raises; only True accepted."""
        if not state:
            _ni("ThrowExceptions(False)", "this library always raises; "
                "disabling exceptions is not supported")

    ThrowExceptions = throw_exceptions

    # ------------------------------------------------------------------
    # Firmware-level features that do not exist: named stubs so ported code
    # fails loudly at the call site instead of silently misbehaving.
    # ------------------------------------------------------------------

    def recorder_count_get(self):
        _ni("RecorderCountGet", "no firmware Recorder; poll Axis status instead")

    RecorderCountGet = recorder_count_get

    def user_limit_count_get(self):
        _ni("UserLimitCountGet", "no firmware user limits; use Axis software limits")

    UserLimitCountGet = user_limit_count_get

    def compensator_count_get(self):
        _ni("CompensatorCountGet", "no firmware compensation tables")

    CompensatorCountGet = compensator_count_get

    def math_block_count_get(self):
        _ni("MathBlockCountGet", "no firmware MathBlock")

    MathBlockCountGet = math_block_count_get

    def interrupt_wait(self, milliseconds):
        _ni("InterruptWait", "no firmware interrupts; use MotionDoneWait / polling")

    InterruptWait = interrupt_wait

    def memory_get(self, address):
        _ni("MemoryGet", "no firmware memory map; use the OpenOCD debug channel")

    MemoryGet = memory_get

    def trace(self, state):
        _ni("Trace", "no firmware trace; log raw_command traffic instead")

    Trace = trace

    # ------------------------------------------------------------------
    # Wire protocol (the raw-access layer)
    # ------------------------------------------------------------------

    def _read_reply(self, timeout=CMD_TIMEOUT):
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if line.startswith(self._REPLY_PREFIXES):
                return line
            # asynchronous events (EVT ...) and noise: drop, keep waiting
        raise CommunicationError("no serial reply (%s)" % self.port)

    def raw_command(self, line, timeout=CMD_TIMEOUT):
        """Send one raw protocol line, return the reply line
        (OK/ERR/STS/SDO/AXES)."""
        self._ser.write((line + "\n").encode())
        return self._read_reply(timeout)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self):
        """Close the serial port (RMP Delete)."""
        try:
            self._ser.close()
        except Exception:
            pass

    Delete = close

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
