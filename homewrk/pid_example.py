#from homewrk.compute.py import compute_error
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

def compute_error(actual: float, desired:float) -> float:
    return desired - actual

def pid_calculate(current_error, prev_error: float, dt:float, min_val:float, max_val:float) -> float:
    """_summary_

    Args:
        current_error (_type_): _description_
        prev_error (float): _description_

    Returns:
        float: _description_
        
    # Position Error
    # Integral Error
    # Derivative Error
    """
    
    kp: float = 0.5 
    ki: float = 0.0
    kd: float = 0.0

    prop_error:float = kp * current_error
    int_error:float =  ki * ((prev_error + current_error)/2) * dt
    der_error:float = kd * ((current_error - prev_error)/2) * dt
    
    # Computes the new command with the cumulative error, then clips the value to account for physical constraints
    command = prop_error + int_error + der_error
    command = np.clip(command, min_val, max_val)
    
    return command


# NED
vehicle = Quadcopter("udp:127.0.0.1:14551")
state = vehicle.update()

desired_x_ned = 10.0
current_x = ... # need to get
prev_error = 0.0
error = compute_error(actual=current_x, desired=desired_x_ned)



while True:
    error = compute_error()
    command_vel_x = pid_calculate(current_error=error,
                            prev_error=prev_error,
                            dt = .05,
                            min_val = -10.0,
                            max_val = 10.0
                            )
    print(f"Command, {command_vel_x}")
    
    #command_drone()
    vehicle.command_velocity_ned(
                vn_m_s=command_vel_x,
                ve_m_s=0, 
                vd_m_s=0,
            )
    
    state = vehicle.update()
    current_x = state.north_m
    print("current_x", current_x)

