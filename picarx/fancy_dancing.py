import time
from picarx_improved import Picarx
import sys
sys.path.append('/home/ali_pi/robot-hat')

px = Picarx()

class FancyDancing(object):

    def __init__(self):
        self.exit_var = 0

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
        time.sleep(0.01)
        px.forward(30)
        time.sleep(0.01)
        px.set_dir_servo_angle(-backward_angle)
        time.sleep(0.01)
        px.backward(30)

    def parallel_park(self):
        print("parallel park")
    def k_turn(self):
        print("k turn")

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
