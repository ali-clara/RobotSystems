import time
import numpy as np
import cv2
import logging
from logdecorator import log_on_start , log_on_end , log_on_error
from motors import Motors
from sensor import Sensor
from interpretor import Interpretor

try:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    time.sleep(0.01)
except ImportError:
    print("PiCamera not present, not a picar-x system")
    from sim_robot_hat import *

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt ="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

mtrs = Motors()
snsr = Sensor()
intr = Interpretor()

class Controller(object):
    def __init__(self, steering_offset=30):
        self.steering_offset = steering_offset

    def line_following(self):
        mtrs.forward(20)
        gm_val_list = snsr.sense_line()
        line_offset, state = intr.grayscale_processing(gm_val_list)
        steering_angle = line_offset*self.steering_offset
        mtrs.set_dir_servo_angle(steering_angle)
        
        print(gm_val_list, state, steering_angle)
       # if intr.state == "off":
       #     mtrs.stop()
       #     time.sleep(1)

if __name__== "__main__":
   # snsr.stream_camera()
    ctrl = Controller()
    while True:
        ctrl.line_following()

    
    
