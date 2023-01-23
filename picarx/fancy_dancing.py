import time
from picarx_improved import Picarx
import sys
sys.path.append('/home/ali_pi/robot-hat')

px = Picarx()

class FancyDancing(object):

    def __init__(self):
        self.exit_var = 0
        self.wait_between_commands = time.sleep(0.01)

    def steering_calibration(self):
        speed = int(input("Enter a speed (??): "))
        t = int(input("Enter a time (s): "))
        px.forward(speed)
        time.sleep(t)
        px.stop()
    
    def forward_and_reverse(self):
        forward_angle = int(input("Set forward steering angle: "))
        backward_angle = int(input("Set reverse steering angle: "))

        px.set_dir_servo_angle(forward_angle)
        self.wait_between_commands
        px.forward(50)
        time.sleep(5)
        px.stop()
        self.wait_between_commands
        px.set_dir_servo_angle(0)
        time.sleep(0.25)

        px.set_dir_servo_angle(backward_angle)
        self.wait_between_commands
        px.backward(50)
        time.sleep(5)
        px.stop()
        self.wait_between_commands
        px.set_dir_servo_angle(0)

    def parallel_park_general(self, steering_angle, speed):
        ''' Inputs: steering_angle (deg), speed (pwm?? 40 is a moderate speed)
        Parallel parks in one direction given a steering angle and speed. Is called by parallel_park()'''
        px.set_dir_servo_angle(steering_angle)
        self.wait_between_commands
        px.backward(speed)
        time.sleep(2)
        px.stop()
        self.wait_between_commands
        px.set_dir_servo_angle(-steering_angle)
        self.wait_between_commands
        px.backward(speed)
        time.sleep(2)
        px.stop()
    
    def parallel_park(self):
        direction = input("Enter direction ('right' or 'left'): ")
        if direction == "right":
            self.parallel_park_general(20, 40)
        else:
            self.parallel_park_general(-20, 40)

    def k_turn_general(self, steering_angle, speed):
        ''' Inputs: steering_angle (deg), speed (pwm?? 40 is a moderate speed)
        Performs a k-turn in one direction given a steering angle and speed. Is called by k_turn()'''
        px.set_dir_servo_angle(steering_angle)
        self.wait_between_commands
        px.forward(speed)
        time.sleep(2.95)
        px.stop()
        self.wait_between_commands
        px.set_dir_servo_angle(-steering_angle)
        self.wait_between_commands
        px.backward(speed)
        time.sleep(2.95)
        px.set_dir_servo_angle(steering_angle)
        self.wait_between_commands
        px.forward(speed)
        time.sleep(2.95)
        px.stop()
        px.set_dir_servo_angle(0)
        self.wait_between_commands
        px.forward(speed)
        time.sleep(3.5)
        px.stop()
        
    def k_turn(self):
        direction = input("Enter initial turn direction ('right' or 'left'): ")
        if direction == "right":
            self.k_turn_general(20, 40)
        else:
            self.k_turn_general(-20, 40)

    def exit_program(self):
        self.exit_var = 1

    def user_interface(self):
        print("Options:")
        print("1 - Forward and Reverse")
        print("2 - Parallel Parking")
        print("3 - K Turning")
        print("4 - Check steering calibration")
        print("q - Exit the program")

        task = input("Enter the number of the task to execute, or type 'q' to exit: ")
        if task == "q":
            self.exit_program()
        elif task == "1":
            self.forward_and_reverse()
        elif task == "2":
            self.parallel_park()
        elif task == "3": 
            self.k_turn()
        elif task == "4":
           self.steering_calibration()
        else:
            print("Invalid entry. Please enter the number that corresponds to the task you'd like to execute, or type 'q' to exit the program.")

if __name__ == "__main__":
    fancy_dancing = FancyDancing()

    while fancy_dancing.exit_var == 0:
        fancy_dancing.user_interface()
