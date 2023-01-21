import time
import os
import logging
from logdecorator import log_on_start , log_on_end , log_on_error
import atexit
import numpy as np
import sys
sys.path.append('/home/ali_pi/robot-hat/robot_hat')

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
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=config_file,
                ):

        # config_flie
        self.config_flie = fileDB(config, 774, User)
        # servos init 
        self.camera_servo_pin1 = Servo(PWM(servo_pins[0]))
        self.camera_servo_pin2 = Servo(PWM(servo_pins[1]))   
        self.dir_servo_pin = Servo(PWM(servo_pins[2])) 
        self.dir_cal_value = int(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_cal_value_1 = int(self.config_flie.get("picarx_cam_servo1", default_value=0))
        self.cam_cal_value_2 = int(self.config_flie.get("picarx_cam_servo2", default_value=0))
        self.dir_servo_pin.angle(self.dir_cal_value)
        self.camera_servo_pin1.angle(self.cam_cal_value_1)
        self.camera_servo_pin2.angle(self.cam_cal_value_2)
        # motors init
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
        # grayscale module init
        # usage: self.grayscale.get_grayscale_data()
        adc0, adc1, adc2 = grayscale_pins
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=1000)
        # ultrasonic init
        # usage: distance = self.ultrasonic.read()
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))

        # stop motors upon shutdown
        atexit.register(self.stop)
        
    @log_on_start(logging.DEBUG , "Setting motor {motor} speed: {speed}")
    @log_on_end(logging.DEBUG , "Set motor speed ")
    def set_motor_speed(self,motor,speed):
        ''' Inputs: motor (int, left = 1, right = 2), speed '''
        # global cali_speed_value,cali_dir_value
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

    def motor_speed_calibration(self,value):
        # global cali_speed_value,cali_dir_value
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibration(self,motor, value):
        # 1: positive direction
        # -1:negative direction
        motor -= 1
        # if value == 1:
        #     self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]
        # self.config_flie.set("picarx_dir_motor", self.cali_dir_value)
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_angle_calibration(self,value):
        self.dir_cal_value = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self,value):
        self.dir_current_angle = value
        angle_value  = value + self.dir_cal_value
        self.dir_servo_pin.angle(angle_value)

    def camera_servo1_angle_calibration(self,value):
        self.cam_cal_value_1 = value
        self.config_flie.set("picarx_cam_servo1", "%s"%value)
        self.camera_servo_pin1.angle(value)

    def camera_servo2_angle_calibration(self,value):
        self.cam_cal_value_2 = value
        self.config_flie.set("picarx_cam_servo2", "%s"%value)
        self.camera_servo_pin2.angle(value)

    def set_camera_servo1_angle(self,value):
        self.camera_servo_pin1.angle(-1*(value + -1*self.cam_cal_value_1))

    def set_camera_servo2_angle(self,value):
        self.camera_servo_pin2.angle(-1*(value + -1*self.cam_cal_value_2))

    def set_power(self,speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed) 

    def adjust_speed(self, steering_angle):
        # car dimensions
        length = 11.6
        height = 9.5
        # distance to instantaneous center of rotation, calculated with angles
        icr_dist = np.tan(90 - steering_angle)*height + length/2
        wheel_velocity_scale = (icr_dist - length/2) / length
        return abs(wheel_velocity_scale)

    def backward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            wheel_speed_adjust = self.adjust_speed(current_angle)
            print(wheel_speed_adjust)
            # if the car is pointed right, slow down the right wheel
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed*wheel_speed_adjust)
            # if the car is pointed left, slow down the left wheel
            else:
                self.set_motor_speed(1, -1*speed*wheel_speed_adjust)
                self.set_motor_speed(2, speed)
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            wheel_speed_adjust = self.adjust_speed(current_angle)
            print(wheel_speed_adjust)
            # if the car is pointed right, slow down the right wheel
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed)
                self.set_motor_speed(2, -speed*wheel_speed_adjust) 
                # print("current_speed: %s %s"%(1*speed * power_scale, -speed))
            # if the car is pointed left, slow down the left wheel
            else:
                self.set_motor_speed(1, speed*wheel_speed_adjust)
                self.set_motor_speed(2, -1*speed)
                # print("current_speed: %s %s"%(speed, -1*speed * power_scale))
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    @log_on_start(logging.DEBUG, "Stopping motors")
    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        self.get_grayscale_reference = value
        
    def get_grayscale_data(self):
        return list.copy(self.grayscale.get_grayscale_data())

    def get_line_status(self,gm_val_list):
        return str(self.grayscale.get_line_status(gm_val_list))

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()
