import time
from picarx_improved import Picarx
import sys
sys.path.append('/home/ali_pi/robot-hat')

px = Picarx()

def steering_calibration():
    speed = int(input("Enter a speed (??): "))
    t = int(input("Enter a time (s): "))
    px.forward(speed)
    time.sleep(t)
    px.stop()

steering_calibration()
