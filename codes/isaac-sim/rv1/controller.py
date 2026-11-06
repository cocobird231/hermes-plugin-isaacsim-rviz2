"""Compatibility import for the canonical Nav2-only RV1 actuator.

The implementation lives in robots/rv1_tune_control.py, matching the requested
robot control source. No manual agent command or standalone keyboard loop lives
here. Existing runtime and feedback integrations retain the RV1Controller API.
"""
from robots.rv1_tune_control import PARAMS, RV1Controller

__all__ = ["PARAMS", "RV1Controller"]
