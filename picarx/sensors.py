import time

try:
    from robot_hat import *
    from robot_hat import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions")
    from sim_robot_hat import *

class Sensors(object):
    
    def __init__(self,
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3']):

        # grayscale module init
        adc0, adc1, adc2 = grayscale_pins
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=1000)
        # ultrasonic init
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))