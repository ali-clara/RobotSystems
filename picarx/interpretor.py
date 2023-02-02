import numpy as np
import cv2
import time

class Interpretor(object):
    def __init__(self, 
                sensitivity = 0.5,
                polarity = 1):
        ''' inputs: sensitivity (???), polarity (0- light line, 1- dark line)'''

        self.calibration_param = None
        self.polarity = polarity
        # position of the robot relative to the line ('left', 'right', or 'centered')
        self.state = None
        self.rel_position = 0
        self.gs_val_list = None

    def line_consumer_producer(self, gs_bus, camera_bus, line_interp_bus, delay):
        """Reads raw data from gs_ and camera_bus, writes interpreted values to 
            line_interp_bus
            Written message: [-1,1] relative numeric position of the robot relative to the line, 
                where 0 is centered and -1 is off the line to the right"""
        # while True:
        camera_message = camera_bus.read()
        self.gs_val_list = gs_bus.read()
        self.grayscale_processing()
        line_interp_bus.write(self.rel_position)
        time.sleep(delay)
    
    def calibrate_grayscale(self):
        self.calibration_param = np.mean(self.gs_val_list)
        
    def dark_line(self):

        self.calibrate_grayscale()

        similar_threshold = 0.3
        different_threshold = 0.8
        dark_threshold = 1

        left_val = self.gs_val_list[0]
        middle_val = self.gs_val_list[1]
        right_val = self.gs_val_list[2]

        left_val /= self.calibration_param
        right_val /= self.calibration_param
        middle_val /= self.calibration_param

        # print(left_val, middle_val, right_val)

        # if left and middle have similar readings and are OFF the line (needs to turn hard right)
        if np.isclose(left_val, middle_val, atol=similar_threshold) and (left_val - right_val) > different_threshold:
        # if abs(left - middle) < similar_threshold and abs(right - left) > different_threshold:
            self.state = "left"
            self.rel_position = 2/3
        # if right and middle have similar readings and are ON the line (needs to turn slight right)
        elif np.isclose(right_val, middle_val, atol=similar_threshold) and (left_val - right_val) > different_threshold:
            self.state = "left"
            self.rel_position = 1/3
        # if left and middle have similar readings and are ON the line (needs to turn slight left)
        elif np.isclose(left_val, middle_val, atol=similar_threshold) and (right_val - left_val) > different_threshold:
            self.state = "right"
            self.rel_position = -1/3
        # if right and middle have similar readings and are OFF the line (needs to turn hard left)
        elif np.isclose(right_val, middle_val, atol=similar_threshold) and (right_val - left_val) > different_threshold:
        # elif abs(right - middle) < similar_threshold and abs(left - right) > different_threshold:
            self.state = "right"
            self.rel_position = -2/3
        # if centered
        elif np.isclose(right_val, left_val, atol=similar_threshold) and right_val < dark_threshold:
            self.state = "middle"
            self.rel_position = 0
        elif np.isclose(right_val, left_val, atol=similar_threshold) and right_val > dark_threshold:
            self.state = "off"
            self.rel_position = 0
        
    def light_line(self):
        pass
    
    def grayscale_processing(self):

        if self.polarity == 1:
            self.dark_line()
        elif self.polarity == 0:
            self.light_line()

        # return(self.rel_position, self.state)
   

if __name__ == "__main__":
    from sensor import Sensor
    snsr = Sensor()
    intr = Interpretor()

    # line following data test
    # gm_val_list = snsr.sense_line()
    # print(gm_val_list)
    # intr.grayscale_processing(gm_val_list)

    snsr.stream_camera()