import mujoco
import mujoco.viewer
import time
import math

model = mujoco.MjModel.from_xml_path("drone.xml")
data = mujoco.MjData(model)

# targets
target_x = 1.5
target_y = 1.5
target_z = 1.5
target_yaw = 0

# height
hover = 3.4335
kp_z = 2
kd_z = 2
ki_z = 1
integral_z = 0

# position
kp_x = 0.5
kd_x = 0.4
kp_y = 0.5
kd_y = 0.4
max_tilt = 0.5

# tilt
kp_pitch = 2
kd_pitch = 0.3
kp_roll = 2
kd_roll = 0.3

# yaw
kp_yaw = 3
kd_yaw = 1.2

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # read sensors
        x = data.qpos[0]
        y = data.qpos[1]
        height = data.qpos[2]
        vx = data.qvel[0]
        vy = data.qvel[1]
        velocity = data.qvel[2]
        tilt = math.atan2(-data.xmat[1][6], data.xmat[1][8])
        tilt_speed = data.qvel[4]
        tilt_y = math.atan2(-data.xmat[1][7], data.xmat[1][8])
        tilt_y_speed = data.qvel[3]
        yaw = math.atan2(data.xmat[1][3], data.xmat[1][0])
        yaw_speed = data.qvel[5]

        # height
        error_z = target_z - height
        integral_z = integral_z + error_z * model.opt.timestep
        integral_z = max(-2, min(2, integral_z))
        thrust = hover + kp_z * error_z + ki_z * integral_z - kd_z * velocity

        # forward/back
        target_tilt = kp_x * (target_x - x) - kd_x * vx
        target_tilt = max(-max_tilt, min(max_tilt, target_tilt))

        # left/right
        target_tilt_y = kp_y * (target_y - y) - kd_y * vy
        target_tilt_y = max(-max_tilt, min(max_tilt, target_tilt_y))

        # lean in the drone's own direction
        tt_x = math.cos(yaw) * target_tilt + math.sin(yaw) * target_tilt_y
        tt_y = -math.sin(yaw) * target_tilt + math.cos(yaw) * target_tilt_y
        pitch_fix = kp_pitch * (tilt - tt_x) + kd_pitch * tilt_speed
        roll_fix = kp_roll * (tilt_y - tt_y) - kd_roll * tilt_y_speed

        # yaw
        error_yaw = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        yaw_fix = kp_yaw * error_yaw - kd_yaw * yaw_speed
        yaw_fix = max(-1, min(1, yaw_fix))

        # motors
        data.ctrl[0] = thrust + pitch_fix + roll_fix + yaw_fix
        data.ctrl[1] = thrust + pitch_fix - roll_fix - yaw_fix
        data.ctrl[2] = thrust - pitch_fix - roll_fix + yaw_fix
        data.ctrl[3] = thrust - pitch_fix + roll_fix - yaw_fix

        print(x, y, height, yaw)

        # step physics
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)