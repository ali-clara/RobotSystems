import time
import numpy as np
import cv2
import logging
from logdecorator import log_on_start , log_on_end , log_on_error
from motors import Motors

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt ="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


class Controller(object):
    def __init__(self, steering_offset=30):
        self.steering_offset = steering_offset
        self.line_offset = 0
        self.mtrs = Motors()

    def controller_consumer(self, line_interp_bus, sonar_bus, delay):
        """Reads data from line_interp_bus and sonar_bus,
            uses those values to set motor and servo commands"""
        # while True:
        sonar_message = sonar_bus.read()
        self.line_offset = line_interp_bus.read()
        self.line_following()
        time.sleep(delay)

    def line_following(self):
        steering_angle = self.line_offset*self.steering_offset
        self.mtrs.set_dir_servo_angle(steering_angle)
        self.mtrs.forward(20)
        
        # print(gm_val_list, state, steering_angle)
       # if intr.state == "off":
       #     mtrs.stop()
       #     time.sleep(1)

if __name__== "__main__":
   # snsr.stream_camera()
    ctrl = Controller()
    while True:
        ctrl.line_following()

    
    
