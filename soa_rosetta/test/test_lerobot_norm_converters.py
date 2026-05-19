"""Numerical tests for `soa_rosetta.lerobot_norm_converters`.

Verifies the decoder/encoder pair is byte-equivalent to lerobot's
`motors_bus._normalize` / `_unnormalize` (parameterized by the SO-101
follower's per-joint norm modes) against captured calibration samples.

The samples below were recorded by `calibrate_policy_offsets.py` with arm
`soa-follower2`; each pose pair is (ROS-radians implied by stored
`degrees`, lerobot percent). The calibration JSON in use at recording time
is reproduced inline as ``RANGES_SOA_FOLLOWER2``.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from soa_rosetta import lerobot_norm_converters as conv


# soa-follower2.json: (range_min, range_max, drive_mode) per joint.
RANGES_SOA_FOLLOWER2 = {
    'shoulder_pan':  (722, 3436, 0),
    'shoulder_lift': (778, 3285, 0),
    'elbow_flex':    (875, 3065, 0),
    'wrist_flex':    (870, 3209, 0),
    'wrist_roll':    (147, 3936, 0),
    'gripper':       (1888, 3322, 0),
}


# Three pose pairs from /tmp/calib_samples.json: same physical pose, read
# through ROS (degrees, after rad->deg) and through lerobot (percent).
SAMPLES = [
    {
        'ros_deg': {
            'shoulder_pan':   1.9335937499999998,
            'shoulder_lift': -103.00781249999977,
            'elbow_flex':     88.94531249999991,
            'wrist_flex':     50.888671875,
            'wrist_roll':     -4.658203124999994,
            'gripper':        -9.492187499999988,
        },
        'lerobot': {
            'shoulder_pan':   -0.6632277081798037,
            'shoulder_lift': -92.18189070602315,
            'elbow_flex':     99.5433789954338,
            'wrist_flex':     50.235143223599835,
            'wrist_roll':     -2.4544734758511453,
            'gripper':         3.6262203626220364,
        },
    },
    {
        'ros_deg': {
            'shoulder_pan':   40.95703124999996,
            'shoulder_lift':  45.08789062499997,
            'elbow_flex':     -5.009765625000004,
            'wrist_flex':     50.185546875000064,
            'wrist_roll':     -4.746093749999994,
            'gripper':        -9.580078125000018,
        },
        'lerobot': {
            'shoulder_pan':   32.05600589535741,
            'shoulder_lift':  42.24172317510971,
            'elbow_flex':     1.9178082191780845,
            'wrist_flex':     49.551090209491235,
            'wrist_roll':     -2.5072578516759023,
            'gripper':         3.556485355648536,
        },
    },
    {
        'ros_deg': {
            'shoulder_pan':   -9.404296875000012,
            'shoulder_lift':  48.60351562499994,
            'elbow_flex':     -56.68945312500007,
            'wrist_flex':     12.832031250000012,
            'wrist_roll':     -4.658203124999994,
            'gripper':        -9.580078125000018,
        },
        'lerobot': {
            'shoulder_pan':   -10.169491525423723,
            'shoulder_lift':  45.43278819305942,
            'elbow_flex':     -51.78082191780821,
            'wrist_flex':     13.210773834972201,
            'wrist_roll':     -2.4544734758511453,
            'gripper':         3.556485355648536,
        },
    },
]


JOINT_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper']
ARM_JOINTS = JOINT_ORDER[:5]
SELECTORS = [f'position.{j}' for j in JOINT_ORDER]
ARM_SELECTORS = [f'position.{j}' for j in ARM_JOINTS]
GRIPPER_SELECTORS = ['position.gripper']


# 1 servo tick ~= 2*pi/4096 rad. Round-trip tolerance.
ONE_TICK_RAD = 2.0 * math.pi / 4096


@pytest.fixture(autouse=True)
def stub_ranges(monkeypatch):
    """Replace the lazy calibration loader with the known soa-follower2 ranges."""
    monkeypatch.setattr(conv, '_RANGES_CACHE', dict(RANGES_SOA_FOLLOWER2))
    yield


def _fake_joint_state(positions_deg):
    """Build a stand-in for sensor_msgs/JointState from a {joint: degrees} map."""
    names = list(positions_deg.keys())
    return SimpleNamespace(
        name=names,
        position=[math.radians(positions_deg[n]) for n in names],
    )


def _spec(names):
    return SimpleNamespace(names=names)


# ---------------------------------------------------------------------------
# Decode: ROS radians -> lerobot normalized percent
# ---------------------------------------------------------------------------


@pytest.mark.parametrize('sample_idx', range(len(SAMPLES)))
def test_decode_matches_recorded_lerobot(sample_idx):
    """For each captured pose, the decoder reproduces lerobot's percent value."""
    sample = SAMPLES[sample_idx]
    msg = _fake_joint_state(sample['ros_deg'])
    out = conv.decode_joint_state_norm(msg, _spec(SELECTORS))
    assert out.dtype == np.float64
    for i, joint in enumerate(JOINT_ORDER):
        expected = sample['lerobot'][joint]
        # The recorded lerobot values come from float division in motors_bus
        # and the ROS values stored in the JSON have ~6 decimal digits of
        # precision. 1e-2 is a generous bound on the round-trip noise.
        assert math.isclose(float(out[i]), expected, abs_tol=1e-2), (
            f"pose {sample_idx} joint {joint}: decoded {out[i]:.6f} "
            f"vs recorded lerobot {expected:.6f}"
        )


def test_decode_rejects_unknown_joint():
    msg = _fake_joint_state({'wrist_roll': 0.0})
    with pytest.raises(ValueError, match="Unknown joint"):
        conv.decode_joint_state_norm(msg, _spec(['position.nonsense']))


def test_decode_rejects_non_position_selector():
    msg = _fake_joint_state({'wrist_roll': 0.0})
    with pytest.raises(ValueError, match="position.\\*"):
        conv.decode_joint_state_norm(msg, _spec(['velocity.wrist_roll']))


# ---------------------------------------------------------------------------
# Encode: lerobot normalized percent -> ROS radians
# ---------------------------------------------------------------------------


@pytest.mark.parametrize('sample_idx', range(len(SAMPLES)))
def test_encode_arm_reproduces_radians(sample_idx):
    """Feeding lerobot percent through encode_arm_fwd_norm yields the
    radians the same pose started from."""
    sample = SAMPLES[sample_idx]
    action_vec = np.array(
        [sample['lerobot'][j] for j in ARM_JOINTS], dtype=np.float64)
    msg = conv.encode_arm_fwd_norm(action_vec, _spec(ARM_SELECTORS))
    expected_rad = [math.radians(sample['ros_deg'][j]) for j in ARM_JOINTS]
    assert len(msg.data) == 5
    for i, joint in enumerate(ARM_JOINTS):
        # Round-trip through float ticks -- accept 1 tick of slop.
        assert math.isclose(msg.data[i], expected_rad[i], abs_tol=ONE_TICK_RAD), (
            f"pose {sample_idx} joint {joint}: encoded "
            f"{msg.data[i]:.6f} rad vs expected {expected_rad[i]:.6f} rad"
        )


@pytest.mark.parametrize('sample_idx', range(len(SAMPLES)))
def test_encode_gripper_reproduces_radians(sample_idx):
    """Gripper uses RANGE_0_100 instead of RANGE_M100_100."""
    sample = SAMPLES[sample_idx]
    action_vec = np.array([sample['lerobot']['gripper']], dtype=np.float64)
    msg = conv.encode_gripper_fwd_norm(action_vec, _spec(GRIPPER_SELECTORS))
    expected_rad = math.radians(sample['ros_deg']['gripper'])
    assert len(msg.data) == 1
    assert math.isclose(msg.data[0], expected_rad, abs_tol=ONE_TICK_RAD)


def test_encode_rejects_length_mismatch():
    with pytest.raises(ValueError, match="length"):
        conv.encode_arm_fwd_norm(np.zeros(3), _spec(ARM_SELECTORS))


# ---------------------------------------------------------------------------
# Round trip: decode then encode should reproduce the input radians.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize('sample_idx', range(len(SAMPLES)))
def test_decode_then_encode_round_trip(sample_idx):
    """decode(rad) -> norm -> encode(norm) -> rad' should preserve rad."""
    sample = SAMPLES[sample_idx]
    msg_in = _fake_joint_state(sample['ros_deg'])
    arm_norm = conv.decode_joint_state_norm(msg_in, _spec(ARM_SELECTORS))
    grip_norm = conv.decode_joint_state_norm(msg_in, _spec(GRIPPER_SELECTORS))
    msg_arm = conv.encode_arm_fwd_norm(arm_norm, _spec(ARM_SELECTORS))
    msg_grip = conv.encode_gripper_fwd_norm(grip_norm, _spec(GRIPPER_SELECTORS))

    for i, joint in enumerate(ARM_JOINTS):
        expected = math.radians(sample['ros_deg'][joint])
        # Pure float math (no int truncation); should round-trip very tight.
        assert math.isclose(msg_arm.data[i], expected, abs_tol=1e-9), (
            f"pose {sample_idx} arm {joint}: {msg_arm.data[i]} vs {expected}"
        )
    expected_grip = math.radians(sample['ros_deg']['gripper'])
    assert math.isclose(msg_grip.data[0], expected_grip, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# drive_mode = 1 inversion path. The default soa-follower2 calibration has
# drive_mode = 0 everywhere, so synthesize a fake range table to exercise the
# negation branches.
# ---------------------------------------------------------------------------


def test_drive_mode_one_inverts_arm(monkeypatch):
    inverted = {**RANGES_SOA_FOLLOWER2,
                'shoulder_pan': (722, 3436, 1)}
    monkeypatch.setattr(conv, '_RANGES_CACHE', inverted)
    # Centered (tick 2079, near 2048) -> normally ~-0.66; inverted -> +0.66.
    msg = _fake_joint_state({'shoulder_pan': 1.9335937499999998})
    out = conv.decode_joint_state_norm(msg, _spec(['position.shoulder_pan']))
    assert math.isclose(float(out[0]), 0.6632277081798037, abs_tol=1e-2)

    # Encode the inverted percent and check it round-trips to the same rad.
    enc = conv.encode_arm_fwd_norm(np.array([0.6632277081798037]),
                                   _spec(['position.shoulder_pan']))
    expected = math.radians(1.9335937499999998)
    assert math.isclose(enc.data[0], expected, abs_tol=ONE_TICK_RAD)


def test_drive_mode_one_inverts_gripper(monkeypatch):
    inverted = {**RANGES_SOA_FOLLOWER2,
                'gripper': (1888, 3322, 1)}
    monkeypatch.setattr(conv, '_RANGES_CACHE', inverted)
    # Normal gripper at the calibration pose returns 3.626; inverted -> 96.374.
    msg = _fake_joint_state({'gripper': -9.492187499999988})
    out = conv.decode_joint_state_norm(msg, _spec(['position.gripper']))
    assert math.isclose(float(out[0]), 100.0 - 3.6262203626220364, abs_tol=1e-2)
