import os
import time
import atexit
import numpy as np
import logging
from logdecorator import log_on_start , log_on_end , log_on_error

try:
    from robot_hat import *
    from robot_hat import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions")
    from sim_robot_hat import *

# user and User home directory
User = os.popen('echo ${SUDO_USER:-$LOGNAME}').readline().strip()
UserHome = os.popen('getent passwd %s | cut -d: -f 6'%User).readline().strip()
config_file = '%s/.config/picar-x/picar-x.conf'%UserHome

# logging setup
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt ="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

class Motors(object):
    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    def __init__(self,
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                config:str=config_file,
                ):

        # config_flie
        self.config_flie = fileDB(config, 774, User)
        
        # initialize servos
        self.camera_servo_pin1 = Servo(PWM(servo_pins[0]))
        self.camera_servo_pin2 = Servo(PWM(servo_pins[1]))   
        self.dir_servo_pin = Servo(PWM(servo_pins[2])) 
        self.dir_cal_value = int(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_cal_value_1 = int(self.config_flie.get("picarx_cam_servo1", default_value=0))
        self.cam_cal_value_2 = int(self.config_flie.get("picarx_cam_servo2", default_value=0))
        self.dir_servo_pin.angle(self.dir_cal_value)
        self.camera_servo_pin1.angle(self.cam_cal_value_1)
        self.camera_servo_pin2.angle(self.cam_cal_value_2)
        
        # initialize motors
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1,1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)
        
        # shutdown properly
        atexit.register(self.cleanup)
    
    def motor_speed_calibration(self,value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibration(self, motor, value):
        # 1: positive direction
        # -1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_angle_calibration(self,value):
        self.dir_cal_value = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)
    
    def camera_servo1_angle_calibration(self,value):
        self.cam_cal_value_1 = value
        self.config_flie.set("picarx_cam_servo1", "%s"%value)
        self.camera_servo_pin1.angle(value)

    def camera_servo2_angle_calibration(self,value):
        self.cam_cal_value_2 = value
        self.config_flie.set("picarx_cam_servo2", "%s"%value)
        self.camera_servo_pin2.angle(value)
    
    @log_on_end(logging.DEBUG , "Set motor {motor} speed: {speed}")
    def set_motor_speed(self, motor,speed):
        ''' Inputs: motor (int, right = 1, left = 2), speed (int)'''
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    @log_on_end(logging.DEBUG , "Set steering angle: {value}")
    def set_dir_servo_angle(self,value):
        '''Inputs: steering_angle (deg)'''
        self.dir_current_angle = value
        angle_value  = value + self.dir_cal_value
        self.dir_servo_pin.angle(angle_value)

    def set_camera_servo1_angle(self,value):
        self.camera_servo_pin1.angle(-1*(value + -1*self.cam_cal_value_1))

    def set_camera_servo2_angle(self,value):
        self.camera_servo_pin2.angle(-1*(value + -1*self.cam_cal_value_2))

    def calc_speed_differential(self, steering_angle):
        '''Inputs: steering_angle (deg)
        Calculates factor by which to slow down the inner wheel on a turn'''
        # car dimensions
        length = 11.6
        height = 9.5
        # distance to instantaneous center of rotation, calculated with angles
        icr_dist = np.tan(90 - steering_angle)*height + length/2
        wheel_velocity_scale = (icr_dist - length/2) / icr_dist
        return abs(wheel_velocity_scale)

    def forward(self,speed):
        ''' Inputs: speed. Drives forward'''
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # cap the angle
            if abs_current_angle > 40:
                abs_current_angle = 40
            wheel_speed_adjust = self.calc_speed_differential(abs_current_angle)
            # if the car is pointed right, slow down the right wheel
            if current_angle > 0:
                self.set_motor_speed(1, 1*speed*wheel_speed_adjust)
                self.set_motor_speed(2, -speed) 
            # if the car is pointed left, slow down the left wheel
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed*wheel_speed_adjust)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)

    def backward(self,speed):
        ''' Inputs: speed. Drives backward'''
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # cap the angle
            if abs_current_angle > 40:
                abs_current_angle = 40
            wheel_speed_adjust = self.calc_speed_differential(abs_current_angle)
            # if the car is pointed right, slow down the right wheel
            if current_angle > 0:
                self.set_motor_speed(1, -1*speed*wheel_speed_adjust)
                self.set_motor_speed(2, speed)
            # if the car is pointed left, slow down the left wheel
            else:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed*wheel_speed_adjust)
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)

    @log_on_start(logging.DEBUG, "Stopping motors")
    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def cleanup(self):
        self.stop()
        self.set_dir_servo_angle(0)

if __name__ == "__main__":
    mtrs = Motors()
    mtrs.forward(50)
    time.sleep(2)
    # mtrs.stop()


    
