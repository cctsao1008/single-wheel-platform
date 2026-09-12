"""Webots trace controller for simulator-neutral Single open-loop experiments.

This is host-only evidence tooling. It consumes the same v2 experiment used by
the analytical and Rust backends, applies ideal joint torques, and emits only
common physical observables. Backend-native drive-joint angle is intentionally
not used as the reduced model's forward coordinate.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

from controller import Supervisor


TRACE_PATH = os.environ.get("SWP_WEBOTS_TRACE")
EXPERIMENT_PATH = os.environ.get("SWP_EXPERIMENT")


def quantity(mapping, name):
    return float(mapping[name]["value"])


def axis_angle_from_roll_pitch(roll, pitch):
    """Return Webots axis-angle for R = Ry(pitch) * Rx(roll), yaw=0."""
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    qw = cr * cp
    qx = sr * cp
    qy = cr * sp
    qz = -sr * sp
    norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if norm < 1.0e-15:
        return [1.0, 0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(norm, qw)
    return [qx / norm, qy / norm, qz / norm, angle]


def load_experiment(step_s):
    if EXPERIMENT_PATH is None:
        duration = float(os.environ.get("SWP_WEBOTS_DURATION_S", "0.25"))
        drive = float(os.environ.get("SWP_WEBOTS_DRIVE_TORQUE_NM", "0.0"))
        reaction = float(os.environ.get("SWP_WEBOTS_REACTION_TORQUE_NM", "0.0"))
        return {
            "name": "legacy-webots-smoke",
            "timing": {"duration_s": duration, "step_s": step_s},
            "initial_state": {
                "forward_position_m": {"value": 0.0},
                "forward_velocity_m_per_s": {"value": 0.0},
                "body_pitch_rad": {"value": 0.0},
                "body_pitch_rate_rad_per_s": {"value": 0.0},
                "body_roll_rad": {"value": 0.0},
                "body_roll_rate_rad_per_s": {"value": 0.0},
                "reaction_position_rad": {"value": 0.0},
                "reaction_rate_rad_per_s": {"value": 0.0},
            },
            "input_profile": [
                {
                    "time_s": {"value": 0.0},
                    "drive_torque_nm": {"value": drive},
                    "reaction_torque_nm": {"value": reaction},
                    "external_force_body_n": {"value": [0.0, 0.0, 0.0]},
                }
            ],
        }

    document = json.loads(Path(EXPERIMENT_PATH).read_text(encoding="utf-8"))
    requested_step = float(document["timing"]["step_s"])
    if not math.isclose(requested_step, step_s, rel_tol=0.0, abs_tol=1.0e-12):
        raise RuntimeError(
            f"experiment step {requested_step} s does not match Webots basicTimeStep {step_s} s"
        )
    for item in document["input_profile"]:
        if any(abs(float(value)) > 0.0 for value in item["external_force_body_n"]["value"]):
            raise RuntimeError("Webots correlation controller does not yet apply external body force")
    return document


def configure_initial_state(robot, experiment):
    state = experiment["initial_state"]
    unsupported = (
        "forward_velocity_m_per_s",
        "body_pitch_rate_rad_per_s",
        "body_roll_rate_rad_per_s",
        "reaction_position_rad",
        "reaction_rate_rad_per_s",
    )
    for name in unsupported:
        if abs(quantity(state, name)) > 1.0e-12:
            raise RuntimeError(f"Webots bootstrap runner currently requires {name}=0")

    node = robot.getSelf()
    translation = node.getField("translation")
    rotation = node.getField("rotation")
    translation.setSFVec3f([quantity(state, "forward_position_m"), 0.0, 0.15])
    rotation.setSFRotation(
        axis_angle_from_roll_pitch(
            quantity(state, "body_roll_rad"), quantity(state, "body_pitch_rad")
        )
    )
    node.resetPhysics()
    return node


def main() -> None:
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    step_s = step_ms / 1000.0
    experiment = load_experiment(step_s)
    duration_s = float(experiment["timing"]["duration_s"])
    body_node = configure_initial_state(robot, experiment)

    imu = robot.getDevice("body_imu")
    gyro = robot.getDevice("body_gyro")
    reaction_encoder = robot.getDevice("reaction_encoder")
    drive_motor = robot.getDevice("drive_motor")
    reaction_motor = robot.getDevice("reaction_motor")

    imu.enable(step_ms)
    gyro.enable(step_ms)
    reaction_encoder.enable(step_ms)

    profile = experiment["input_profile"]
    profile_index = 0
    drive_torque = 0.0
    reaction_torque = 0.0

    def apply_events(time_s):
        nonlocal profile_index, drive_torque, reaction_torque
        while profile_index < len(profile):
            event_time = quantity(profile[profile_index], "time_s")
            if event_time > time_s + 0.5 * step_s:
                break
            drive_torque = quantity(profile[profile_index], "drive_torque_nm")
            reaction_torque = quantity(profile[profile_index], "reaction_torque_nm")
            # setTorque() is direct torque control. Positive drive torque acts
            # about body +Y; positive reaction torque acts about body +X.
            drive_motor.setTorque(drive_torque)
            reaction_motor.setTorque(reaction_torque)
            profile_index += 1

    apply_events(0.0)
    output = open(Path(TRACE_PATH), "w", encoding="utf-8") if TRACE_PATH else None
    previous_reaction = quantity(experiment["initial_state"], "reaction_position_rad")

    try:
        while robot.step(step_ms) != -1:
            time_s = robot.getTime()
            if time_s > duration_s + 0.5 * step_s:
                break

            apply_events(time_s)
            roll, pitch, _yaw = imu.getRollPitchYaw()
            wx, wy, _wz = gyro.getValues()
            reaction_position = reaction_encoder.getValue()
            reaction_rate = (reaction_position - previous_reaction) / step_s
            previous_reaction = reaction_position
            position = body_node.getPosition()
            velocity = body_node.getVelocity()

            record = {
                "time_s": time_s,
                "forward_position_m": position[0],
                "forward_velocity_m_per_s": velocity[0],
                "body_pitch_rad": pitch,
                "body_pitch_rate_rad_per_s": wy,
                "body_roll_rad": roll,
                "body_roll_rate_rad_per_s": wx,
                "reaction_position_rad": reaction_position,
                "reaction_rate_rad_per_s": reaction_rate,
                "drive_torque_nm": drive_torque,
                "reaction_torque_nm": reaction_torque,
            }
            line = json.dumps(record, sort_keys=True, separators=(",", ":"))
            if output:
                output.write(line + "\n")
            else:
                print(line)
    finally:
        if output:
            output.close()
        robot.simulationQuit(0)


if __name__ == "__main__":
    main()
