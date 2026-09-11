"""Minimal Webots trace controller for the synthetic Single rigid-body plant.

This is host-only simulation tooling. It applies explicit ideal joint torques and
records simulator observations in project coordinate semantics. It does not
implement the production estimator/controller stack.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from controller import Robot


DURATION_S = float(os.environ.get("SWP_WEBOTS_DURATION_S", "0.25"))
DRIVE_TORQUE_NM = float(os.environ.get("SWP_WEBOTS_DRIVE_TORQUE_NM", "0.0"))
REACTION_TORQUE_NM = float(os.environ.get("SWP_WEBOTS_REACTION_TORQUE_NM", "0.0"))
TRACE_PATH = os.environ.get("SWP_WEBOTS_TRACE")


def main() -> None:
    robot = Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_s = step_ms / 1000.0

    imu = robot.getDevice("body_imu")
    gyro = robot.getDevice("body_gyro")
    drive_encoder = robot.getDevice("drive_encoder")
    reaction_encoder = robot.getDevice("reaction_encoder")
    drive_motor = robot.getDevice("drive_motor")
    reaction_motor = robot.getDevice("reaction_motor")

    imu.enable(step_ms)
    gyro.enable(step_ms)
    drive_encoder.enable(step_ms)
    reaction_encoder.enable(step_ms)

    # setTorque() switches Webots Motor into direct torque control.
    # +drive torque acts about body +Y; +reaction torque acts about body +X.
    drive_motor.setTorque(DRIVE_TORQUE_NM)
    reaction_motor.setTorque(REACTION_TORQUE_NM)

    output = open(Path(TRACE_PATH), "w", encoding="utf-8") if TRACE_PATH else None
    previous_drive = None
    previous_reaction = None

    try:
        while robot.step(step_ms) != -1:
            time_s = robot.getTime()
            if time_s > DURATION_S + 0.5 * step_s:
                break

            roll, pitch, _yaw = imu.getRollPitchYaw()
            wx, wy, _wz = gyro.getValues()
            drive_position = drive_encoder.getValue()
            reaction_position = reaction_encoder.getValue()

            drive_rate = 0.0 if previous_drive is None else (drive_position - previous_drive) / step_s
            reaction_rate = 0.0 if previous_reaction is None else (reaction_position - previous_reaction) / step_s
            previous_drive = drive_position
            previous_reaction = reaction_position

            record = {
                "time_s": time_s,
                "body_roll_rad": roll,
                "body_roll_rate_rad_s": wx,
                "body_pitch_rad": pitch,
                "body_pitch_rate_rad_s": wy,
                "drive_position_rad": drive_position,
                "drive_rate_rad_s": drive_rate,
                "reaction_position_rad": reaction_position,
                "reaction_rate_rad_s": reaction_rate,
                "drive_torque_nm": DRIVE_TORQUE_NM,
                "reaction_torque_nm": REACTION_TORQUE_NM,
            }
            line = json.dumps(record, sort_keys=True, separators=(",", ":"))
            if output:
                output.write(line + "\n")
            else:
                print(line)
    finally:
        if output:
            output.close()


if __name__ == "__main__":
    main()
