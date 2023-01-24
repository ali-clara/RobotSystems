import time
import os
import logging
from logdecorator import log_on_start , log_on_end , log_on_error
import atexit
import numpy as np
import sys
sys.path.append('/home/ali_pi/robot-hat/robot_hat')

from motors import Motors
from sensor_control import Sensor, Interpretor, Controller
try:
    from robot_hat import *
    from robot_hat import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions")
    from sim_robot_hat import *

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt ="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

# user and User home directory
User = os.popen('echo ${SUDO_USER:-$LOGNAME}').readline().strip()
UserHome = os.popen('getent passwd %s | cut -d: -f 6'%User).readline().strip()
config_file = '%s/.config/picar-x/picar-x.conf'%UserHome

class Picarx(object):
    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: direction_servo, camera_servo_1, camera_servo_2 
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: tring, echo
    # config: path of config file
    @log_on_start(logging.DEBUG , "Intitializing picarx")
    def __init__(self):

        # stop motors upon shutdown
        atexit.register(self.stop)  
                  
if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()
