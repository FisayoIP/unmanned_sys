import pandas as pd
import numpy as np
from unmanned_systems_basics.quadcopter import Quadcopter


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


waypoints = [
    (15.0, 20.0, -30.0),
    (10.0, 5.0, -15.0),
    (-3.0, -5.0, -25.0),
    (25.0, 30.0, -25.0),
]

vehicle = Quadcopter("udp:192.168.64.1:14551")

for _ in range(20):
    state = vehicle.update()

vehicle.takeoff()

print("waiting for liftoff...")
while True:
    state = vehicle.update()
    if state.down_m < -1.0:
        print(f"airborne, down_m={state.down_m:.2f}")
        break

kp_n = 0.5
kp_e = 0.5
kp_d = 0.5

v_limit_horiz = 7
v_limit_vert = 7

prev_error_n = 0.0
prev_error_e = 0.0
prev_error_d = 0.0

log = []

t_prev = state.time_s

for waypoint_idx, (desired_n, desired_e, desired_d) in enumerate(waypoints):
    print(f"heading to waypoint {waypoint_idx}: N={desired_n} E={desired_e} D={desired_d}")

    while True:
        state = vehicle.update()
        dt = state.time_s - t_prev
        t_prev = state.time_s
        if dt <= 0:
            continue

        error_n = compute_error(actual=state.north_m, desired=desired_n)
        error_e = compute_error(actual=state.east_m, desired=desired_e)
        error_d = compute_error(actual=state.down_m, desired=desired_d)

        command_vel_n = pid_calculate(error_n, prev_error_n, dt, kp_n, -v_limit_horiz, v_limit_horiz)
        command_vel_e = pid_calculate(error_e, prev_error_e, dt, kp_e, -v_limit_horiz, v_limit_horiz)
        command_vel_d = pid_calculate(error_d, prev_error_d, dt, kp_d, -v_limit_vert, v_limit_vert)

        vehicle.command_velocity_ned(
            vn_m_s=command_vel_n,
            ve_m_s=command_vel_e,
            vd_m_s=command_vel_d,
        )

        log.append({
            "t_s": state.time_s,
            "waypoint": waypoint_idx,
            "pN": state.north_m, "pE": state.east_m, "pD": state.down_m,
            "VN": state.vn_m_s, "VE": state.ve_m_s, "VD": state.vd_m_s,
            "VN_cmd": command_vel_n, "VE_cmd": command_vel_e, "VD_cmd": command_vel_d,
            "eN": error_n, "eE": error_e, "eD": error_d,
        })

        print(f"t={state.time_s:.2f}  wp={waypoint_idx}  pos=({state.north_m:.2f},{state.east_m:.2f},{state.down_m:.2f})  "
              f"cmd=({command_vel_n:.2f},{command_vel_e:.2f},{command_vel_d:.2f})")

        prev_error_n = error_n
        prev_error_e = error_e
        prev_error_d = error_d

        if abs(error_n) < 0.3 and abs(error_e) < 0.3 and abs(error_d) < 0.3:
            print(f"reached waypoint {waypoint_idx}")
            break

vehicle.stop()

df = pd.DataFrame(log)
df.to_csv("waypoints.csv", index=False)
