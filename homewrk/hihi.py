import pandas as pd
import numpy as np

def func_name() -> None:
    ...
    
def using_arctan_for_theta(inputs) -> np.array:
    theta = np.atan2()
    # can do math on full array
dataset = pd.read_csv("/Users/fisayopopoola/unmanned_sys/toy_data/toy_data_complementary_filter.csv")
print(f"dataset is, {dataset}")
time_d = dataset["t_s"]
acceleration = dataset["accel_y"]
