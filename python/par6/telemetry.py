"""Telemetry consumer: the daemon's recipe-selected field stream.

The reader lives in the engine (``par6-client``); each received frame is
a dict — ``recipe``, ``seq``, ``mono_time_ns``, and ``fields`` keyed by
field name. The stream is silent until a recipe is active (config
``[protocol] initial_recipe`` or a client's ``set_recipe``).
"""

from par6._par6 import TelemetryReader

__all__ = ["TelemetryReader"]
