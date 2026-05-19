"""Rosetta custom converters that mirror lerobot's per-joint range normalization.

Use these in `soa_act_contract.yaml` to make the SO-101 follower's Rosetta
pipeline numerically equivalent to running a policy through
`lerobot.SO101Follower.send_action` / `get_observation`:

    observations:
      - key: observation.state
        topic: /follower/joint_states
        type: sensor_msgs/msg/JointState
        decoder: soa_rosetta.lerobot_norm_converters:decode_joint_state_norm
        selector: { names: [position.shoulder_pan, ..., position.gripper] }

    actions:
      - key: action
        publish: { topic: /follower/arm_fwd_controller/commands,
                   type: std_msgs/msg/Float64MultiArray }
        encoder: soa_rosetta.lerobot_norm_converters:encode_arm_fwd_norm
        selector: { names: [position.shoulder_pan, ..., position.wrist_roll] }
      - key: action
        publish: { topic: /follower/gripper_fwd_controller/commands,
                   type: std_msgs/msg/Float64MultiArray }
        encoder: soa_rosetta.lerobot_norm_converters:encode_gripper_fwd_norm
        selector: { names: [position.gripper] }

The decoder reads the wire-format radians on `/follower/joint_states`,
maps them back to raw servo ticks (the inverse of the feetech_ros2_driver
read path), then applies lerobot's per-joint normalization:
  - 5 arm joints use `RANGE_M100_100` (output in [-100, +100])
  - gripper uses `RANGE_0_100`         (output in [0, 100])
matching `lerobot/robots/so101_follower/so101_follower.py:48-58` and
`lerobot/motors/motors_bus.py:_normalize` (lines 770-797).

The encoders apply the inverse (matching `_unnormalize`, lines 799-827)
and convert back to radians so the existing forward_command_controller +
feetech_ros2_driver write path consumes them unchanged.

The per-joint `range_min` / `range_max` / `drive_mode` are read at first
use from the lerobot calibration JSON whose path is derived from
`soa_bringup/config/soa_params.yaml` (the same source the soa_bringup
launch uses) -- `{follower.calibration_dir}/{follower.id}.json`.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray


# Joint set and norm modes mirror lerobot/robots/so101_follower/so101_follower.py:48-58.
_ARM_JOINTS = (
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
)
_GRIPPER = 'gripper'
_ALL_JOINTS = _ARM_JOINTS + (_GRIPPER,)

# Feetech tick<->radian constants (match feetech_ros2_driver/common.hpp:36
# and calibration_loader.py's offset=2048).
_TICKS_PER_REV = 4096
_CENTER_TICK = 2048
_RAD_PER_TICK = 2.0 * math.pi / _TICKS_PER_REV  # = TWO_PI / 4096


# Cache: joint -> (range_min, range_max, drive_mode).
# Populated on first decoder/encoder call so importing this module never
# fails just because the calibration file isn't installed yet.
_RANGES_CACHE: dict[str, tuple[int, int, int]] | None = None


def _load_ranges() -> dict[str, tuple[int, int, int]]:
    """Resolve soa_params.yaml -> calibration JSON -> per-joint ranges.

    Mirrors the discovery the soa_bringup launch already does
    (soa_bringup.launch.py:115-131): read soa_params.yaml from the package
    share, then look up `{follower.calibration_dir}/{follower.id}.json`.
    """
    bringup_share = get_package_share_directory('soa_bringup')
    params_path = os.path.join(bringup_share, 'config', 'soa_params.yaml')
    with open(params_path) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    follower = params['follower']
    cal_path = os.path.join(follower['calibration_dir'], f"{follower['id']}.json")
    with open(cal_path) as f:
        cal = json.load(f)

    ranges: dict[str, tuple[int, int, int]] = {}
    for j in _ALL_JOINTS:
        if j not in cal:
            raise KeyError(
                f"Calibration file {cal_path} is missing joint '{j}'. "
                f"Found joints: {sorted(cal.keys())}"
            )
        entry = cal[j]
        for field in ('range_min', 'range_max'):
            if field not in entry:
                raise KeyError(
                    f"Calibration file {cal_path} joint '{j}' is missing "
                    f"required field '{field}' (got: {sorted(entry.keys())})"
                )
        ranges[j] = (
            int(entry['range_min']),
            int(entry['range_max']),
            int(entry.get('drive_mode', 0)),
        )
    return ranges


def _ranges() -> dict[str, tuple[int, int, int]]:
    global _RANGES_CACHE
    if _RANGES_CACHE is None:
        _RANGES_CACHE = _load_ranges()
    return _RANGES_CACHE


# ---------------------------------------------------------------------------
# Math: lerobot _normalize / _unnormalize equivalents, around the rad<->tick
# transform that the feetech_ros2_driver applies on read/write.
# ---------------------------------------------------------------------------


def _rad_to_tick(rad: float) -> float:
    return rad / _RAD_PER_TICK + _CENTER_TICK


def _tick_to_rad(tick: float) -> float:
    return (tick - _CENTER_TICK) * _RAD_PER_TICK


def _tick_to_norm(joint: str, tick: float) -> float:
    """Mirror lerobot motors_bus._normalize (motors_bus.py:770-797).

    Arm joints use RANGE_M100_100; gripper uses RANGE_0_100.
    """
    lo, hi, drive = _ranges()[joint]
    bounded = min(hi, max(lo, tick))
    if joint == _GRIPPER:
        norm = (bounded - lo) / (hi - lo) * 100.0
        return 100.0 - norm if drive else norm
    norm = (bounded - lo) / (hi - lo) * 200.0 - 100.0
    return -norm if drive else norm


def _norm_to_tick(joint: str, val: float) -> float:
    """Mirror lerobot motors_bus._unnormalize (motors_bus.py:799-827).

    Returns float ticks (no int() round). The downstream feetech driver's
    from_radians() does the int conversion when writing to the servo.
    """
    lo, hi, drive = _ranges()[joint]
    if joint == _GRIPPER:
        if drive:
            val = 100.0 - val
        bounded = min(100.0, max(0.0, val))
        return bounded / 100.0 * (hi - lo) + lo
    if drive:
        val = -val
    bounded = min(100.0, max(-100.0, val))
    return (bounded + 100.0) / 200.0 * (hi - lo) + lo


# ---------------------------------------------------------------------------
# Public converters consumed by Rosetta.
# ---------------------------------------------------------------------------


def decode_joint_state_norm(msg: Any, spec: Any) -> np.ndarray:
    """Decode `sensor_msgs/JointState` into lerobot-normalized values.

    For each `position.<joint>` selector in `spec.names`, read the radian
    position from the message, convert to raw ticks (inverse of
    feetech_ros2_driver.cpp:135-137), then apply lerobot's per-joint
    normalization. Non-position selectors are not supported -- the SO-101
    contract only uses positions.
    """
    if not spec.names:
        raise ValueError(
            'decode_joint_state_norm requires selector.names; got empty list.'
        )

    name_to_idx = {n: i for i, n in enumerate(msg.name)}
    out = np.empty(len(spec.names), dtype=np.float64)
    for i, selector in enumerate(spec.names):
        field, joint = (selector.split('.', 1) if '.' in selector
                        else ('position', selector))
        if field != 'position':
            raise ValueError(
                f"decode_joint_state_norm only supports 'position.*' "
                f"selectors; got '{selector}'."
            )
        if joint not in _ALL_JOINTS:
            raise ValueError(
                f"Unknown joint '{joint}' in selector '{selector}'. "
                f"Expected one of: {list(_ALL_JOINTS)}"
            )
        if joint not in name_to_idx:
            raise ValueError(
                f"Joint '{joint}' not in JointState.name (got: {list(msg.name)})."
            )
        rad = float(msg.position[name_to_idx[joint]])
        out[i] = _tick_to_norm(joint, _rad_to_tick(rad))
    return out


def _encode_norm_to_radians(action_vec: Any, spec: Any) -> Float64MultiArray:
    """Shared body for arm/gripper encoders.

    Maps each normalized action value through lerobot's _unnormalize and
    back to radians (so the forward_command_controller + feetech driver
    write path consumes the same wire format as today).
    """
    arr = np.asarray(action_vec, dtype=np.float64).flatten()
    if len(arr) != len(spec.names):
        raise ValueError(
            f"Action vector length {len(arr)} != selector.names length "
            f"{len(spec.names)} (names: {list(spec.names)})."
        )
    out = np.empty(len(arr), dtype=np.float64)
    for i, selector in enumerate(spec.names):
        field, joint = (selector.split('.', 1) if '.' in selector
                        else ('position', selector))
        if field != 'position':
            raise ValueError(
                f"lerobot-norm encoders only support 'position.*' "
                f"selectors; got '{selector}'."
            )
        if joint not in _ALL_JOINTS:
            raise ValueError(
                f"Unknown joint '{joint}' in selector '{selector}'. "
                f"Expected one of: {list(_ALL_JOINTS)}"
            )
        out[i] = _tick_to_rad(_norm_to_tick(joint, float(arr[i])))

    msg = Float64MultiArray()
    msg.data = out.tolist()
    return msg


def encode_arm_fwd_norm(action_vec: Any, spec: Any, stamp_ns: int | None = None
                        ) -> Float64MultiArray:
    """Encode normalized arm actions to a radian Float64MultiArray.

    Each value in `action_vec` is interpreted in lerobot's RANGE_M100_100
    convention and translated to the radian the forward_command_controller
    expects on `/follower/arm_fwd_controller/commands`.
    """
    _ = stamp_ns  # Float64MultiArray has no header.
    return _encode_norm_to_radians(action_vec, spec)


def encode_gripper_fwd_norm(action_vec: Any, spec: Any, stamp_ns: int | None = None
                            ) -> Float64MultiArray:
    """Encode the gripper action (RANGE_0_100) to a radian Float64MultiArray."""
    _ = stamp_ns
    return _encode_norm_to_radians(action_vec, spec)
