"""Load SCOUT MINI and send a constant linear/angular velocity command."""

import argparse
import math
from pathlib import Path
import time

import mujoco


MODEL_PATH = Path(__file__).resolve().parents[1] / "model" / "scout_mini.xml"


def finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("must be finite")
    return number


def set_velocity(model, data, linear, angular):
    """Ideal differential-drive mixing; actual skid-steer yaw depends on slip."""
    radius = model.geom("front_left_tire").size[0]
    track = model.numeric("drive_track").data[0]
    left = (linear - angular * track / 2) / radius
    right = (linear + angular * track / 2) / radius
    # Scale both sides together to preserve the requested curvature.
    limit = model.actuator_ctrlrange[0, 1]
    scale = max(1.0, abs(left) / limit, abs(right) / limit)
    for name, speed in (("front_left_motor", left), ("rear_left_motor", left),
                        ("front_right_motor", right), ("rear_right_motor", right)):
        data.ctrl[model.actuator(name).id] = speed / scale


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("gazebo", "primitive"), default="gazebo",
                        help="Gazebo meshes (default) or the lightweight primitive model")
    parser.add_argument("--linear", type=finite_float, default=0.4, help="forward speed in m/s")
    parser.add_argument("--angular", type=finite_float, default=0.0, help="yaw rate in rad/s (+left)")
    parser.add_argument("--duration", type=finite_float, default=15.0, help="simulation seconds")
    parser.add_argument("--headless", action="store_true", help="run without a viewer")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    model_path = MODEL_PATH if args.model == "gazebo" else MODEL_PATH.with_name("scout_mini_primitive.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def step():
        # Let ground contacts settle before driving.
        if data.time >= 0.5:
            set_velocity(model, data, args.linear, args.angular)
        mujoco.mj_step(model, data)

    if args.headless:
        while data.time < args.duration:
            step()
    else:
        from mujoco import viewer as mujoco_viewer

        with mujoco_viewer.launch_passive(model, data) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = model.body("base_link").id
            viewer.cam.distance = 1.8
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            while viewer.is_running() and data.time < args.duration:
                start = time.perf_counter()
                # Sync the viewer at approximately 60 Hz.
                for _ in range(max(1, round(1 / (60 * model.opt.timestep)))):
                    if data.time >= args.duration:
                        break
                    step()
                viewer.sync()
                time.sleep(max(0, 1 / 60 - (time.perf_counter() - start)))

    if any(data.warning.number):
        raise RuntimeError("MuJoCo reported a simulation warning")
    x, y, z = data.qpos[:3]
    print(f"Simulated {data.time:.2f} s; base position: x={x:.3f}, y={y:.3f}, z={z:.3f} m")


if __name__ == "__main__":
    main()
