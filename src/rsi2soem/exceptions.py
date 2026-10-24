"""Exception hierarchy, mirroring RSI::RapidCode::RsiError."""


class RsiError(Exception):
    """Base class of all motion-control errors (RSI::RapidCode::RsiError)."""


MotionError = RsiError  # compatibility alias


class CommunicationError(RsiError):
    """No serial reply, or a malformed reply."""


class NetworkError(RsiError):
    """The EtherCAT network is not in the OPERATIONAL state."""


class SoftLimitError(RsiError):
    """Target outside the software limits; the command was NOT sent."""


class FaultError(RsiError):
    """Drive fault (carries the CiA402 error code of object 0x603F)."""

    def __init__(self, message, error_code=None):
        super().__init__(message)
        self.error_code = error_code


class MotionTimeoutError(RsiError):
    """MotionDoneWait timed out."""


class NotEnabledError(RsiError):
    """A motion was attempted while the axis was not enabled."""


class AxisNotPresentError(RsiError):
    """The requested axis does not exist on the EtherCAT chain."""


class RsiNotImplementedError(RsiError, NotImplementedError):
    """This RSI feature is not supported by the rsi2soem firmware/hardware."""


def _ni(name, reason):
    """Raise a uniform not-implemented error for an RSI API stub."""
    raise RsiNotImplementedError("%s: %s" % (name, reason))
