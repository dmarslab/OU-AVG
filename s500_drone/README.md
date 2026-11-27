# S500 Drone Sim

MuJoCo simulation of the Holybro S500 V2 quadcopter with a simple cascaded PD flight controller (height, position, tilt and yaw). It runs without ROS, PX4 or Gazebo.

## Run it

```
python -m pip install mujoco
python fly.py
```

Run the command from inside this folder. A viewer window opens, and the drone flies to its target and hovers there.

To push the drone in the viewer, double-click it, then hold Ctrl and drag with the right mouse button (push) or the left mouse button (twist).

## Files

- `drone.xml`: the drone model. An invisible 1.4 kg box handles physics, and 4 motors sit at the real S500 motor positions (480 mm wheelbase).
- `fly.py`: the flight controller.
- `s500.stl`: the S500 frame mesh, used for looks only.

## Settings (top of `fly.py`)

- `target_x`, `target_y`, `target_z`: where the drone should hover (meters).
- `target_yaw`: which way it faces (radians, 0 = +x).
- `max_tilt`: the most the drone is allowed to lean.
- `kp_*`, `kd_*`: controller gains.

## Notes

- The sim reads perfect position and tilt from MuJoCo. The real drone has to estimate these from sensors (IMU, rangefinder, optical flow).
- The S500 frame CAD comes from Holybro's public S500 frame kit download (docs.holybro.com).
