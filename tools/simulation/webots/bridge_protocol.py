"""Device-like Webots -> production bridge encoding for Single.

This module is deliberately independent of the Webots controller API so the
axis/sign/unit contract can be regression-tested in ordinary host CI.

The bridge boundary carries MPU6050 register-domain values and 16-bit encoder
counter snapshots. It never carries Webots body truth, Euler angles, or a
reduced-model state vector into the production estimator/control path.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

STANDARD_GRAVITY_MPS2 = 9.80665
ACCEL_LSB_PER_G = 8192.0  # MPU6050 ±4 g synthetic bridge fixture
GYRO_LSB_PER_DPS = 32.8  # MPU6050 ±1000 deg/s synthetic bridge fixture
ENCODER_COUNTS_PER_REVOLUTION = 65_536
WIRE_SCHEMA = 1
MAPPING_ID = "webots-body-identity-v1"


class BridgeContractError(ValueError):
    pass


@dataclass(frozen=True)
class AxisMapping:
    accel_axes: tuple[int, int, int] = (0, 1, 2)
    accel_signs: tuple[int, int, int] = (1, 1, 1)
    gyro_axes: tuple[int, int, int] = (0, 1, 2)
    gyro_signs: tuple[int, int, int] = (1, 1, 1)
    drive_encoder_sign: int = 1
    reaction_encoder_sign: int = 1


CANONICAL_MAPPING = AxisMapping()


def validate_mapping(mapping: AxisMapping) -> None:
    """Reject simulator-only sign/axis compensation.

    The Webots sensor nodes are installed in the canonical body frame:
    +X forward, +Y left, +Z up. Drive joint +Y is positive forward rolling;
    reaction joint +X is positive by the right-hand rule. Any alternate mapping
    is a model/fixture defect to investigate, not something to tune away here.
    """

    if mapping != CANONICAL_MAPPING:
        raise BridgeContractError(
            "Webots device mapping must remain canonical; hidden axis/sign compensation is forbidden"
        )


def _finite_vector(values, name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise BridgeContractError(f"{name} must contain exactly three values")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise BridgeContractError(f"{name} contains a non-finite value")
    return converted


def _quantize_i16(value: float) -> int:
    if not math.isfinite(value):
        raise BridgeContractError("cannot quantize a non-finite sensor value")
    rounded = round(value)
    return max(-32768, min(32767, rounded))


def _map_vector(values, axes, signs) -> tuple[float, float, float]:
    return tuple(float(signs[index]) * values[axes[index]] for index in range(3))


def _encoder_count(angle_rad: float, sign: int) -> int:
    if not math.isfinite(angle_rad):
        raise BridgeContractError("encoder angle must be finite")
    counts = sign * angle_rad / math.tau * ENCODER_COUNTS_PER_REVOLUTION
    return int(round(counts)) % (1 << 16)


def encode_raw_sample(
    *,
    sample_index: int,
    timestamp_us: int,
    accel_m_per_s2,
    gyro_rad_per_s,
    drive_encoder_rad: float,
    reaction_encoder_rad: float,
    mapping: AxisMapping = CANONICAL_MAPPING,
) -> dict:
    """Encode one Webots device sample as the raw bridge wire contract."""

    validate_mapping(mapping)
    if not 0 <= int(sample_index) <= 0xFFFF_FFFF:
        raise BridgeContractError("sample_index must fit u32")
    if int(timestamp_us) < 0:
        raise BridgeContractError("timestamp_us must be non-negative")

    accel = _map_vector(
        _finite_vector(accel_m_per_s2, "accel_m_per_s2"),
        mapping.accel_axes,
        mapping.accel_signs,
    )
    gyro = _map_vector(
        _finite_vector(gyro_rad_per_s, "gyro_rad_per_s"),
        mapping.gyro_axes,
        mapping.gyro_signs,
    )

    accel_scale = ACCEL_LSB_PER_G / STANDARD_GRAVITY_MPS2
    gyro_scale = GYRO_LSB_PER_DPS * 180.0 / math.pi

    return {
        "schema": WIRE_SCHEMA,
        "mapping_id": MAPPING_ID,
        "sample_index": int(sample_index),
        "timestamp_us": int(timestamp_us),
        "accel_raw": [_quantize_i16(value * accel_scale) for value in accel],
        # 0 code is exactly 36.53 C under the nominal MPU6050 transfer function.
        "temperature_raw": 0,
        "gyro_raw": [_quantize_i16(value * gyro_scale) for value in gyro],
        "drive_encoder_count": _encoder_count(
            float(drive_encoder_rad), mapping.drive_encoder_sign
        ),
        "reaction_encoder_count": _encoder_count(
            float(reaction_encoder_rad), mapping.reaction_encoder_sign
        ),
    }
