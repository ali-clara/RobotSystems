import time
<<<<<<< HEAD
from motors import Motors
import atexit
=======
from picarx_improved import Picarx
>>>>>>> parent of f2889ed (updated fancy_dancing to call motor script)
import sys
sys.path.append('/home/ali_pi/robot-hat')

px = Picarx()

class FancyDancing(object):

    def __init__(self):
        self.exit_var = 0
        self.wait_between_commands = time.sleep(0.01)
        atexit.register(px.cleanup) # do I need this here if it's also in motors

    def steering_calibration(self):
        speed = int(input("Enter a speed (??): "))
        t = int(input("Enter a time (s): "))
        px.forward(speed)
        time.sleep(t)
        px.stop()
    
    def drive_forward(self, speed, steering_angle, t):
        px.set_dir_servo_angle(steering_angle)
        self.wait_between_commands
        px.forward(speed)
        time.sleep(t)
        px.stop()
        self.wait_between_commands
        px.set_dir_servo_angle(0)
        time.sleep(0.25)

    def drive_reverse(self, speed, steering_angle, t):
        px.set_dir_servo_angle(steering_angle)
        self.wait_between_commands
        px.backward(speed)
        time.sleep(t)
        px.stop()
        self.wait_between_commands
        px.set_dir_servo_angle(0)
    
    def forward_and_reverse(self):
        '''No inputs. Asks user for steering angles and drives forward and backward at those angles'''
        # get user input
        try:
            forward_angle = int(input("Enter forward steering angle (-30 to 30 deg): "))
            backward_angle = int(input("Enter reverse steering angle (-30 to 30 deg): "))
        except:
            forward_angle = 0
            backward_angle = 0
            print("Error in input, defaulted to 0 degrees")
        
        # drive forward and back at set angle and speed for time
        t = 5
        speed = 50
        self.drive_forward(speed, forward_angle, t)
        self.drive_reverse(speed, backward_angle, t)

    def parallel_park_general(self, speed, steering_angle):
        ''' Inputs: speed (pwm?? 40 is a moderate speed), steering_angle (deg)
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
            self.parallel_park_general(40, 20)
        elif direction == "left":
            self.parallel_park_general(40, -20)
        else:
            print("Please enter a direction")

    def k_turn_general(self, steering_angle, speed, t=2.95):
        ''' Inputs: steering_angle (deg), speed (pwm?? 40 is a moderate speed), t (time to execute each maneuver)
        Performs a k-turn in one direction given a steering angle and speed. Is called by k_turn()'''
        px.set_dir_servo_angle(steering_angle)
        self.wait_between_commands
        px.forward(speed)
        time.sleep(t)
        px.stop()
        self.wait_between_commands
        
    def k_turn(self):
        direction = input("Enter initial turn direction ('right' or 'left'): ")
        if direction == "right":
            self.k_turn_general(20, 40)
            self.k_turn_general(-20, 40)
            self.k_turn_general(20, 40)
            self.k_turn_general(0, 40)
        elif direction == "left":
            self.k_turn_general(-20, 40)
            self.k_turn_general(20, 40)
            self.k_turn_general(-20, 40)
            self.k_turn_general(0, 40)
        else:
            print("Please enter an initial direction")

    def exit_program(self):
        # exits the user interface
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
