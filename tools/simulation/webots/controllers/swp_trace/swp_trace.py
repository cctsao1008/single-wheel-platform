"""Webots evidence controller for Single open- and closed-loop experiments.

Open-loop mode consumes the simulator-neutral v2 experiment contract.
Closed-loop mode is deliberately different: Webots exposes only device-like
accelerometer, gyro, and encoder evidence to the Rust production bridge. Webots
pose/orientation truth is recorded for evidence only and never enters the
production estimator or Control.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import sys

from controller import Supervisor


WEBOTS_ROOT = Path(__file__).resolve().parents[2]
if str(WEBOTS_ROOT) not in sys.path:
    sys.path.insert(0, str(WEBOTS_ROOT))

from bridge_protocol import encode_raw_sample  # noqa: E402


TRACE_PATH = os.environ.get("SWP_WEBOTS_TRACE")
EXPERIMENT_PATH = os.environ.get("SWP_EXPERIMENT")
CLOSED_LOOP = os.environ.get("SWP_WEBOTS_CLOSED_LOOP") == "1"
CONTROL_BRIDGE_PATH = os.environ.get("SWP_WEBOTS_CONTROL_BRIDGE")


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


def closed_loop_experiment(step_s):
    return {
        "name": "synthetic-production-path-closed-loop",
        "timing": {
            "duration_s": float(os.environ.get("SWP_WEBOTS_DURATION_S", "0.40")),
            "step_s": step_s,
        },
        "initial_state": {
            "forward_position_m": {
                "value": float(os.environ.get("SWP_WEBOTS_INITIAL_FORWARD_M", "0.0"))
            },
            "forward_velocity_m_per_s": {"value": 0.0},
            "body_pitch_rad": {
                "value": float(os.environ.get("SWP_WEBOTS_INITIAL_PITCH_RAD", "0.005"))
            },
            "body_pitch_rate_rad_per_s": {"value": 0.0},
            "body_roll_rad": {
                "value": float(os.environ.get("SWP_WEBOTS_INITIAL_ROLL_RAD", "-0.005"))
            },
            "body_roll_rate_rad_per_s": {"value": 0.0},
            "reaction_position_rad": {"value": 0.0},
            "reaction_rate_rad_per_s": {"value": 0.0},
        },
        "input_profile": [],
    }


def load_experiment(step_s):
    if CLOSED_LOOP:
        return closed_loop_experiment(step_s)

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


def start_production_bridge():
    if not CLOSED_LOOP:
        return None
    if not CONTROL_BRIDGE_PATH:
        raise RuntimeError("SWP_WEBOTS_CONTROL_BRIDGE is required in closed-loop mode")
    path = Path(CONTROL_BRIDGE_PATH)
    if not path.is_file():
        raise RuntimeError(f"production bridge executable does not exist: {path}")
    return subprocess.Popen(
        [str(path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )


def bridge_step(process, raw_sample):
    if process.poll() is not None:
        stderr = process.stderr.read() if process.stderr else ""
        raise RuntimeError(
            f"production bridge exited before sample {raw_sample['sample_index']}: {stderr.strip()}"
        )
    process.stdin.write(json.dumps(raw_sample, separators=(",", ":")) + "\n")
    process.stdin.flush()
    line = process.stdout.readline()
    if not line:
        stderr = process.stderr.read() if process.stderr else ""
        raise RuntimeError(
            f"production bridge returned no response for sample {raw_sample['sample_index']}: "
            f"{stderr.strip()}"
        )
    return json.loads(line)


def truth_record(body_node, imu, gyro, reaction_encoder):
    roll, pitch, yaw = imu.getRollPitchYaw()
    wx, wy, wz = gyro.getValues()
    position = body_node.getPosition()
    velocity = body_node.getVelocity()
    return {
        "forward_position_m": position[0],
        "forward_velocity_m_per_s": velocity[0],
        "body_roll_rad": roll,
        "body_pitch_rad": pitch,
        "body_yaw_rad": yaw,
        "body_roll_rate_rad_per_s": wx,
        "body_pitch_rate_rad_per_s": wy,
        "body_yaw_rate_rad_per_s": wz,
        "reaction_position_rad": reaction_encoder.getValue(),
    }


def run_closed_loop(
    robot,
    body_node,
    experiment,
    output,
    imu,
    accel,
    gyro,
    drive_encoder,
    reaction_encoder,
    drive_motor,
    reaction_motor,
    bridge,
):
    step_ms = int(robot.getBasicTimeStep())
    step_s = step_ms / 1000.0
    duration_s = float(experiment["timing"]["duration_s"])
    control_period_ms = 2
    if control_period_ms % step_ms != 0:
        raise RuntimeError("Webots basicTimeStep must divide the 2 ms production period")
    control_period_steps = control_period_ms // step_ms

    accel.enable(control_period_ms)
    gyro.enable(control_period_ms)
    drive_encoder.enable(control_period_ms)
    reaction_encoder.enable(control_period_ms)
    imu.enable(step_ms)

    drive_motor.setTorque(0.0)
    reaction_motor.setTorque(0.0)
    drive_torque = 0.0
    reaction_torque = 0.0
    sample_index = 0
    simulation_step = 0

    while robot.step(step_ms) != -1:
        simulation_step += 1
        time_s = robot.getTime()
        if time_s > duration_s + 0.5 * step_s:
            break
        if simulation_step % control_period_steps != 0:
            continue

        timestamp_us = int(round(time_s * 1_000_000.0))
        raw_sample = encode_raw_sample(
            sample_index=sample_index,
            timestamp_us=timestamp_us,
            accel_m_per_s2=accel.getValues(),
            gyro_rad_per_s=gyro.getValues(),
            drive_encoder_rad=drive_encoder.getValue(),
            reaction_encoder_rad=reaction_encoder.getValue(),
        )
        response = bridge_step(bridge, raw_sample)

        if response["sample_index"] != sample_index:
            raise RuntimeError(
                f"production bridge response sample mismatch: {response['sample_index']} != {sample_index}"
            )
        if response["actuation"] == "apply":
            drive_torque = float(response["drive_torque_nm"])
            reaction_torque = float(response["reaction_torque_nm"])
        elif response["actuation"] == "revoke":
            drive_torque = 0.0
            reaction_torque = 0.0
        else:
            raise RuntimeError(f"unknown production bridge actuation {response['actuation']!r}")

        # Only the production bridge response can change Webots actuator torque.
        drive_motor.setTorque(drive_torque)
        reaction_motor.setTorque(reaction_torque)

        record = {
            "mode": "closed_loop_production_path",
            "time_s": time_s,
            "raw_device_observation": raw_sample,
            "production": response,
            "authorized_drive_torque_nm": drive_torque,
            "authorized_reaction_torque_nm": reaction_torque,
            # Evidence-only simulator truth. This object is never sent to Rust.
            "webots_evidence_truth": truth_record(
                body_node, imu, gyro, reaction_encoder
            ),
        }
        line = json.dumps(record, sort_keys=True, separators=(",", ":"))
        if output:
            output.write(line + "\n")
            output.flush()
        else:
            print(line)
        sample_index += 1


def run_open_loop(
    robot,
    body_node,
    experiment,
    output,
    imu,
    gyro,
    reaction_encoder,
    drive_motor,
    reaction_motor,
):
    step_ms = int(robot.getBasicTimeStep())
    step_s = step_ms / 1000.0
    duration_s = float(experiment["timing"]["duration_s"])
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
    previous_reaction = quantity(experiment["initial_state"], "reaction_position_rad")

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


def main() -> None:
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    step_s = step_ms / 1000.0
    experiment = load_experiment(step_s)
    body_node = configure_initial_state(robot, experiment)

    imu = robot.getDevice("body_imu")
    accel = robot.getDevice("body_accel")
    gyro = robot.getDevice("body_gyro")
    drive_encoder = robot.getDevice("drive_encoder")
    reaction_encoder = robot.getDevice("reaction_encoder")
    drive_motor = robot.getDevice("drive_motor")
    reaction_motor = robot.getDevice("reaction_motor")

    bridge = start_production_bridge()
    output = open(Path(TRACE_PATH), "w", encoding="utf-8") if TRACE_PATH else None

    try:
        if CLOSED_LOOP:
            run_closed_loop(
                robot,
                body_node,
                experiment,
                output,
                imu,
                accel,
                gyro,
                drive_encoder,
                reaction_encoder,
                drive_motor,
                reaction_motor,
                bridge,
            )
        else:
            run_open_loop(
                robot,
                body_node,
                experiment,
                output,
                imu,
                gyro,
                reaction_encoder,
                drive_motor,
                reaction_motor,
            )
    finally:
        drive_motor.setTorque(0.0)
        reaction_motor.setTorque(0.0)
        if output:
            output.close()
        if bridge:
            if bridge.stdin:
                bridge.stdin.close()
            try:
                bridge.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                bridge.terminate()
                bridge.wait(timeout=2.0)
            if bridge.returncode not in (0, None):
                stderr = bridge.stderr.read() if bridge.stderr else ""
                raise RuntimeError(
                    f"production bridge exited with {bridge.returncode}: {stderr.strip()}"
                )
        robot.simulationQuit(0)


if __name__ == "__main__":
    main()
