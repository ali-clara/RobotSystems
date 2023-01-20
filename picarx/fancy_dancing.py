import time
from picarx_improved import Picarx

px = Picarx()

def steering_calibration(speed):
    px.forward(speed)
    time.sleep(3)
    px.stop()

steering_calibration(0.5)
