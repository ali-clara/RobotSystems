import time
import numpy as np
from motors import Motors

try:
    from robot_hat import *
    from robot_hat import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions")
    from sim_robot_hat import *

class Sensor(object):
    def __init__(self,
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3']):

        # grayscale module init
        adc0, adc1, adc2 = grayscale_pins
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=1000)
        # ultrasonic init
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        self.get_grayscale_reference = value
        
    def get_grayscale_data(self):
        return list.copy(self.grayscale.get_grayscale_data())

    def get_line_status(self,gm_val_list):
        return str(self.grayscale.get_line_status(gm_val_list))

    def sense_line(self):
        gm_val_list = self.get_grayscale_data()
        gm_state = self.get_line_status(gm_val_list)
        #print("gm_val_list: %s, %s"%(gm_val_list, gm_state))
        return(gm_val_list)

class Interpretor(object):
    def __init__(self, 
                sensitivity = 0.5,
                polarity = 1):
        ''' inputs: sensitivity (???), polarity (0- light line, 1- dark line)'''

        self.polarity = polarity
        # position of the robot relative to the line ('left', 'right', or 'centered')
        self.state = None
        # [-1,1] relative numeric position of the robot relative to the line, where 0 is centered and -1 is off the line to the right
        self.rel_position = 0

    def dark_line(self, left_val, middle_val, right_val):
        similar_threshold = 50
        different_threshold = 80
        dark_threshold = 200

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
        
    def light_line(self, left_val, middle_val, right_val):
        pass
    
    def processing(self, gm_val_list):
        left_val = gm_val_list[0]
        middle_val = gm_val_list[1]
        right_val = gm_val_list[2]

        if self.polarity == 1:
            self.dark_line(left_val, middle_val, right_val)
        elif self.polarity == 0:
            self.light_line(left_val, middle_val, right_val)

        return(self.rel_position, self.state)

class Controller(object):
    def __init__(self, steering_offset=30):
        self.steering_offset = steering_offset

    def line_following(self, line_offset):
        steering_angle = line_offset*self.steering_offset
        mtrs.set_dir_servo_angle(steering_angle)
        return steering_angle

if __name__== "__main__":
    mtrs = Motors()
    snsr = Sensor()
    intr = Interpretor()
    ctrl = Controller()

    while True:
        mtrs.forward(20)
        gm_val_list = snsr.sense_line()
        line_offset, state = intr.processing(gm_val_list)
        steering_angle = ctrl.line_following(line_offset)
        print(gm_val_list, state, steering_angle)
        #if intr.state == "off":
           #mtrs.stop()
           #time.sleep(1)
    
