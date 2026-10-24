"""sub2rsi - RSI RMP-style motion-control API over a USB-serial SOEM master.

Hardware chain:
    PC (this package) --CH340 USB-serial--> STM32H743 SOEM EtherCAT master
        --EtherCAT--> MOONS' AMA step-servo drive(s) (CiA402, CSP mode)

Class mapping:
    RSI::RapidCode::MotionController  ->  sub2rsi.MotionController
    RSI::RapidCode::Axis              ->  sub2rsi.Axis
    RSI::RapidCode::MultiAxis         ->  sub2rsi.MultiAxis
    RSI::RapidCode::NetworkNode       ->  sub2rsi.NetworkNode
    RSI::RapidCode::RsiError          ->  sub2rsi.RsiError (and subclasses)
    RSIState                          ->  sub2rsi.CiaState

Coverage policy:
    The core single-axis surface is fully implemented and tested on real
    hardware. Advanced families the firmware/hardware cannot support
    (multi-axis interpolation, PT/PVT streaming, gearing/cam, filters,
    hardware IO, Recorder, ...) exist as same-named methods that raise
    RsiNotImplementedError, so ported code fails loudly at the call site.

Dependency: pyserial.
"""

__version__ = "0.1.0"

from .constants import (COUNTS_PER_MM, COUNTS_PER_REV, SCREW_PITCH_MM,
                        AxisStatus, CiaState, RSIAction, RSIEventType,
                        RSIHomeMethod, RSIOperationMode, RSIState,
                        decode_cia_state)
from .exceptions import (AxisNotPresentError, CommunicationError, FaultError,
                         MotionError, MotionTimeoutError, NetworkError,
                         NotEnabledError, RsiError, RsiNotImplementedError,
                         SoftLimitError)
from .controller import MotionController
from .axis import Axis
from .multiaxis import MultiAxis
from .node import NetworkNode

__all__ = [
    "__version__",
    # classes
    "MotionController", "Axis", "MultiAxis", "NetworkNode", "AxisStatus",
    # enums
    "CiaState", "RSIState", "RSIAction", "RSIOperationMode", "RSIHomeMethod",
    "RSIEventType",
    # errors
    "RsiError", "MotionError", "CommunicationError", "NetworkError",
    "SoftLimitError", "FaultError", "MotionTimeoutError", "NotEnabledError",
    "AxisNotPresentError", "RsiNotImplementedError",
    # helpers / constants
    "decode_cia_state", "COUNTS_PER_REV", "SCREW_PITCH_MM", "COUNTS_PER_MM",
]
