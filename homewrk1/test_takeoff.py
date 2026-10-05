import time
from unmanned_systems_basics.quadcopter import Quadcopter

vehicle = Quadcopter("udp:192.168.64.1:14551")

vehicle.takeoff()

for i in range(10):
    state = vehicle.update()
    print(f"t={state.time_s:.2f}  down_m={state.down_m:.2f}")
    time.sleep(0.5)