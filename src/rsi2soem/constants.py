"""Constants, enumerations and the status-word decoder."""

import enum

COUNTS_PER_REV = 2048.0                       # encoder counts per motor rev
SCREW_PITCH_MM = 2.0                          # ball-screw lead, mm per rev
COUNTS_PER_MM = COUNTS_PER_REV / SCREW_PITCH_MM   # 1024.0

FIRMWARE_VERSION = "ecat_h743 multi-axis v2.1 (multi-channel)"
SAMPLE_RATE_HZ = 1000.0                       # firmware EtherCAT cycle: 1 ms

READ_TIMEOUT = 0.2
CMD_TIMEOUT = 3.0
DONE_TOL_COUNTS_DEFAULT = 30                  # default in-position tolerance

DEFAULT_MAX_VEL_MM_S = 5.0

#: Default axis table used when no ``axes_config`` is supplied: generic names
#: AX0..AX2 and no software limits (limits are opt-in, see Axis.software_limits_set).
DEFAULT_AXES_CONFIG = {}


class CiaState(enum.Enum):
    """CiA402 / DS402 drive state machine (RMP's RSIState)."""
    NOT_READY_TO_SWITCH_ON = "NOT_READY_TO_SWITCH_ON"
    SWITCH_ON_DISABLED = "SWITCH_ON_DISABLED"
    READY_TO_SWITCH_ON = "READY_TO_SWITCH_ON"
    SWITCHED_ON = "SWITCHED_ON"
    OPERATION_ENABLED = "OPERATION_ENABLED"
    QUICK_STOP_ACTIVE = "QUICK_STOP_ACTIVE"
    FAULT_REACTION_ACTIVE = "FAULT_REACTION_ACTIVE"
    FAULT = "FAULT"
    UNKNOWN = "UNKNOWN"


#: Alias that keeps ported RapidCode programs readable.
RSIState = CiaState


class RSIAction(enum.Enum):
    """Action taken on limits/events (RMP RSIAction).

    The firmware actions are fixed; these values are configuration records.
    """
    NONE = "NONE"
    STOP = "STOP"
    ESTOP = "ESTOP"
    ABORT = "ABORT"


class RSIOperationMode(enum.Enum):
    CSP = "CSP"          # cyclic synchronous position - the only mode here
    PP = "PP"
    PV = "PV"
    HM = "HM"


class RSIHomeMethod(enum.Enum):
    NONE = 0
    MANUAL = 1           # manual homing: touch the limit, then PositionSet(0)


class RSIEventType(enum.IntEnum):
    """StatusBitGet event bits (RMP RSIEventType subset mapped onto DS402)."""
    MOTION_DONE = 10        # statusword bit10 target reached
    WARNING = 7
    FAULT = 3
    ENABLED = 2
    REMOTE = 9


def decode_cia_state(sw):
    """Decode the CiA402 state machine from a statusword (masks per DS402)."""
    if (sw & 0x4F) == 0x00:
        return CiaState.NOT_READY_TO_SWITCH_ON
    if (sw & 0x4F) == 0x40:
        return CiaState.SWITCH_ON_DISABLED
    if (sw & 0x6F) == 0x21:
        return CiaState.READY_TO_SWITCH_ON
    if (sw & 0x6F) == 0x23:
        return CiaState.SWITCHED_ON
    if (sw & 0x6F) == 0x27:
        return CiaState.OPERATION_ENABLED
    if (sw & 0x6F) == 0x07:
        return CiaState.QUICK_STOP_ACTIVE
    if (sw & 0x4F) == 0x0F:
        return CiaState.FAULT_REACTION_ACTIVE
    if (sw & 0x4F) == 0x08:
        return CiaState.FAULT
    return CiaState.UNKNOWN


class AxisStatus(object):
    """Fully decoded snapshot of one STS query. All attributes are read-only."""

    def __init__(self, axis_index, fields):
        self.axis = int(fields.get("ax", str(axis_index)))
        self.status_word = int(fields.get("sw", "0x0"), 16)
        self.ready = fields.get("rdy") == "1"           # bit0 ready to switch on
        self.switched_on = fields.get("on") == "1"      # bit1
        self.enabled = fields.get("en") == "1"          # bit2 operation enabled
        self.fault = fields.get("fault") == "1"         # bit3
        self.warning = fields.get("warn") == "1"        # bit7
        self.target_reached = fields.get("tgt") == "1"  # bit10
        self.position_counts = int(fields.get("pos", "0"))
        self.velocity_counts_s = int(fields.get("vel", "0"))
        self.error_code = int(fields.get("err", "0x0"), 16)
        self.mode = fields.get("mode", "?")             # CSP / NONE
        self.al_state = fields.get("al", "?")           # EtherCAT AL: OP/...
        self.wkc = int(fields.get("wkc", "0"))          # working counter

    @property
    def state(self):
        return decode_cia_state(self.status_word)

    @property
    def moving(self):
        return not self.target_reached

    def __repr__(self):
        return ("<AxisStatus ax=%d %s en=%d fault=%d tgt=%d pos=%d vel=%d "
                "err=0x%04X al=%s>"
                % (self.axis, self.state.value, self.enabled, self.fault,
                   self.target_reached, self.position_counts,
                   self.velocity_counts_s, self.error_code, self.al_state))
