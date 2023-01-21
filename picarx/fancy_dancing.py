import time
from picarx_improved import Picarx
import sys
sys.path.append('/home/ali_pi/robot-hat')

px = Picarx()

def steering_calibration():
    speed = int(input("Enter a speed: "))
    px.forward(speed)
    time.sleep(3)
    px.stop()

steering_calibration()
print(sys.path)
