"""
SOA robot arm custom converters for Rosetta.

Handles the unit mismatch between:
  - feetech_ros2_driver: publishes/accepts radians (tick = rad*4096/(2π) + 2048)
  - LeRobot SO101FollowerConfig: normalizes to [-100,100] (arm) / [0,100] (gripper)

Calibration data from gix-follower1.json (must match the physical robot).
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np
from rosidl_runtime_py.utilities import get_message

_TWO_PI = 2.0 * math.pi

# Calibration from gix-follower1.json
_CALIB: dict[str, dict] = {
    "shoulder_pan":  {"range_min": 759,  "range_max": 3363, "type": "arm"},
    "shoulder_lift": {"range_min": 816,  "range_max": 3317, "type": "arm"},
    "elbow_flex":    {"range_min": 887,  "range_max": 3108, "type": "arm"},
    "wrist_flex":    {"range_min": 826,  "range_max": 3163, "type": "arm"},
    "wrist_roll":    {"range_min": 156,  "range_max": 3967, "type": "arm"},
    "gripper":       {"range_min": 1885, "range_max": 3343, "type": "gripper"},
}


def _rad_to_tick(rad: float) -> float:
    return rad * 4096.0 / _TWO_PI + 2048.0


def _tick_to_norm(tick: float, joint_name: str) -> float:
    c = _CALIB[joint_name]
    ratio = (tick - c["range_min"]) / (c["range_max"] - c["range_min"])
    return ratio * 200.0 - 100.0 if c["type"] == "arm" else ratio * 100.0


def _norm_to_tick(norm: float, joint_name: str) -> float:
    c = _CALIB[joint_name]
    span = c["range_max"] - c["range_min"]
    return (
        (norm + 100.0) / 200.0 * span + c["range_min"]
        if c["type"] == "arm" else
        norm / 100.0 * span + c["range_min"]
    )


def _tick_to_rad(tick: float) -> float:
    return (tick - 2048.0) * _TWO_PI / 4096.0


def decode_soa_joint_state(msg: Any, spec: Any) -> np.ndarray:
    """Decode JointState radians -> LeRobot-normalized observation.

    Signature: (msg: JointState, spec: ObservationStreamSpec) -> np.ndarray
    Replaces the default JointState decoder for the observation.state key.
    Selector names like "position.shoulder_pan" are parsed the same way as
    the default _dec_joint_state decoder in rosetta/common/decoders.py.
    """
    name_to_idx = {name: i for i, name in enumerate(msg.name)}
    out = []

    for selector in spec.names:
        field, joint_name = selector.split(".", 1) if "." in selector else ("position", selector)

        if joint_name not in name_to_idx:
            raise ValueError(f"Joint '{joint_name}' not in JointState. Available: {list(msg.name)}")

        idx = name_to_idx[joint_name]
        arr = getattr(msg, field)
        if idx >= len(arr):
            raise ValueError(f"Index {idx} out of range for '{field}' (len={len(arr)})")

        rad = float(arr[idx])
        out.append(_tick_to_norm(_rad_to_tick(rad), joint_name))

    return np.asarray(out, dtype=np.float64)


def encode_soa_action(action_vec: Any, spec: Any, stamp_ns: int | None = None) -> Any:
    """Encode LeRobot-normalized actions -> Float64MultiArray of radians.

    Signature: (action_vec: np.ndarray, spec: ActionStreamSpec, stamp_ns) -> ROS msg
    Replaces the default Float64MultiArray encoder for the action key.
    spec.clamp is applied to normalized values before conversion (same as default encoder).
    The safety_behavior=hold path stores normalized arrays and re-calls this encoder;
    the conversion always produces correct radians output.
    """
    Float64MultiArray = get_message("std_msgs/msg/Float64MultiArray")
    msg = Float64MultiArray()

    arr = np.asarray(action_vec, dtype=np.float64).flatten()
    if spec.clamp:
        arr = np.clip(arr, spec.clamp[0], spec.clamp[1])

    joint_names = list(spec.names)
    if len(arr) != len(joint_names):
        raise ValueError(f"Action length {len(arr)} != selector names length {len(joint_names)}")

    msg.data = [_tick_to_rad(_norm_to_tick(float(n), j)) for n, j in zip(arr, joint_names)]
    return msg
