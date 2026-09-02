"""AUBO i7 control adapter.

Importing this package does not import the optional :mod:`pyaubo_sdk` module.
The vendor SDK is loaded only when :class:`AuboControl` is instantiated without
an explicitly supplied ``sdk_module``.
"""

from .aubo_control import (
    AUBOI7,
    AUBO_i7,
    AuboControl,
    AuboControlError,
    AuboError,
    AuboI7,
    AuboModelError,
)

__all__ = (
    "AUBOI7",
    "AUBO_i7",
    "AuboControl",
    "AuboControlError",
    "AuboError",
    "AuboI7",
    "AuboModelError",
)
