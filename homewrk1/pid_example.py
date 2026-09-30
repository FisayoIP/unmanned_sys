import pandas as pd
import numpy as np
from unmanned_systems_basics.quadcopter import Quadcopter

"""
Recipe for PID
- Set desired pos
- Calculate error
- Feed error to pid
- send command to quadcopter
- Repeat
"""

def compute_error(actual: float, desired: float) -> float:
    return desired - actual


def pid_calculate(current_error, prev_error: float, dt: float, kp: float, min_val: float, max_val: float) -> float:
    ki: float = 0.0
    kd: float = 0.0

    prop_error: float = kp * current_error
    int_error: float = ki * ((prev_error + current_error) / 2) * dt
    der_error: float = kd * ((current_error - prev_error) / 2) * dt

    command = prop_error + int_error + der_error
    command = np.clip(command, min_val, max_val)

    return command


# NED
vehicle = Quadcopter("udp:127.0.0.1:14551")

# throw away the first few reads -- state can come back all zeros before
# telemetry actually starts arriving, and we don't want that as our first error
for _ in range(20):
    state = vehicle.update()

vehicle.takeoff()

desired_n = 10.0
desired_e = 5.0
desired_d = -5.0   # NED: negative down = climbing 5m

kp_n = 0.5
kp_e = 0.5
kp_d = 0.5

v_limit_horiz = 5.0   # matches command_velocity_ned's own clamp
v_limit_vert = 2.0    # matches command_velocity_ned's own clamp

prev_error_n = 0.0
prev_error_e = 0.0
prev_error_d = 0.0

log = []

t_prev = state.time_s

while True:
    state = vehicle.update()
    dt = state.time_s - t_prev
    t_prev = state.time_s
    if dt <= 0:
        continue   # duplicate/stale telemetry read, skip this iteration

    error_n = compute_error(actual=state.north_m, desired=desired_n)
    error_e = compute_error(actual=state.east_m, desired=desired_e)
    error_d = compute_error(actual=state.down_m, desired=desired_d)

    command_vel_n = pid_calculate(error_n, prev_error_n, dt, kp_n, -v_limit_horiz, v_limit_horiz)
    command_vel_e = pid_calculate(error_e, prev_error_e, dt, kp_e, -v_limit_horiz, v_limit_horiz)
    command_vel_d = pid_calculate(error_d, prev_error_d, dt, kp_d, -v_limit_vert, v_limit_vert)

    # command_velocity_ned clips N/E together as one vector, not per-axis --
    # match that here so the command we LOG is the command that actually gets APPLIED
    horiz_mag = np.hypot(command_vel_n, command_vel_e)
    if horiz_mag > v_limit_horiz:
        command_vel_n *= v_limit_horiz / horiz_mag
        command_vel_e *= v_limit_horiz / horiz_mag

    vehicle.command_velocity_ned(
        vn_m_s=command_vel_n,
        ve_m_s=command_vel_e,
        vd_m_s=command_vel_d,
    )

    log.append({
        "t_s": state.time_s,
        "pN": state.north_m, "pE": state.east_m, "pD": state.down_m,
        "VN": state.vn_m_s, "VE": state.ve_m_s, "VD": state.vd_m_s,
        "VN_cmd": command_vel_n, "VE_cmd": command_vel_e, "VD_cmd": command_vel_d,
        "eN": error_n, "eE": error_e, "eD": error_d,
    })

    print(f"t={state.time_s:.2f}  pos=({state.north_m:.2f},{state.east_m:.2f},{state.down_m:.2f})  "
          f"cmd=({command_vel_n:.2f},{command_vel_e:.2f},{command_vel_d:.2f})")

    prev_error_n = error_n
    prev_error_e = error_e
    prev_error_d = error_d

    if abs(error_n) < 0.3 and abs(error_e) < 0.3 and abs(error_d) < 0.3:
        print("reached target")
        break

vehicle.stop()

df = pd.DataFrame(log)
df.to_csv("problem4_run.csv", index=False)