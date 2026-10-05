import pandas as pd
import numpy as np

def func_name() -> None:
    ...
    
def using_arctan_for_theta(inputs) -> np.array:
    theta = np.arctan2()
    #theta = atan2(-ax, sqrt(ay² + az²))
    # can do math on full array
    # dataset = pd.read_csv("/Users/fisayopopoola/unmanned_sys/toy_data/toy_data_complementary_filter.csv")
    # print(f"dataset is, {dataset}")
    # time_d = dataset["t_s"]
    # acceleration = dataset["accel_y_m_s2"]
    # print(acceleration)

def gyro_only_integration(rate, dt, theta0=0.0):
    theta = np.zeros(len(rate))
    theta[0] = theta0
    for k in range(1, len(rate)):
        theta[k] = theta[k-1] + rate[k] * dt[k]
    return theta

def accel_roll_pitch(ax, ay, az):
    roll_acc = np.arctan2(-ay, -az)
    pitch_acc = np.arctan2(ax, np.sqrt(ay**2 + az**2))
    return roll_acc, pitch_acc

def complementary_filter(rate, dt, acc_angle, alpha, theta0):
    theta = np.zeros(len(rate))
    theta[0] = theta0
    for k in range(1, len(rate)):
        gyro_step = theta[k - 1] + rate[k] * dt[k]
        theta[k] = alpha * gyro_step + (1 - alpha) * acc_angle[k]
    return theta

def rmse(estimate, truth):
    return np.sqrt(np.mean((estimate - truth) ** 2))