import time
import numpy as np
import cv2
import logging
from logdecorator import log_on_start , log_on_end , log_on_error

try:
    from robot_hat import *
    from robot_hat import reset_mcu
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions")
    from sim_robot_hat import *

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt ="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

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

    def grayscale_producer(self, gs_bus, delay):
        """Writes data from grayscale sensors to gs_bus.
            Args - gs_bus (Bus() object), delay (time delay, sec).
            Message: [raw_value_left, raw_value_center, raw_value_right]"""
        gs_message = self.sense_line()
        gs_bus.write(gs_message)
        time.sleep(delay)

    def sonar_producer(self, sonar_bus, delay):
        """Writes sonar data to sonar_bus
            Args - sonar_bus (Bus() object), delay (time delay, sec).
            Message: raw_value (float?)"""
        sonar_message = self.get_distance()
        sonar_bus.write(sonar_message)
        time.sleep(delay)
    
    def camera_producer(self, camera_bus, delay):
        """Writes camera data to camera_bus
            Args - camera_bus (Bus() object), delay (time delay, sec)
            Message: not sure yet. Either raw camera frame or line coordinates"""
        time.sleep(delay)

    ######## SONAR ########
    def get_distance(self):
        """SONAR, returns data from ultrasonic sensors.
            Returns - distance (unit ??)"""
        dist = self.ultrasonic.read()
        return dist

    ######## GRAYSCALE ########
    def set_grayscale_reference(self, value):
        """GS"""
        self.get_grayscale_reference = value
        
    def get_grayscale_data(self):
        """GS, returns raw data from grayscale sensors"""
        return list.copy(self.grayscale.get_grayscale_data())

    def get_line_status(self,gm_val_list):
        """GS"""
        return str(self.grayscale.get_line_status(gm_val_list))

    def sense_line(self):
        """GS"""
        gm_val_list = self.get_grayscale_data()
        gm_state = self.get_line_status(gm_val_list)
        # print("gm_val_list: %s, %s"%(gm_val_list, gm_state))
        return gm_val_list

    ######## CAMERA ########
    @log_on_start(logging.DEBUG, "Starting camera ('esc' to quit)")
    @log_on_end(logging.DEBUG, "Quitting camera")
    def stream_camera(self):
        with PiCamera() as camera:
            camera.resolution = (640,480)
            camera.framerate = 24
            rawCapture = PiRGBArray(camera, size=camera.resolution)  
            time.sleep(2)

            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                img = frame.array
                img_2 = self.camera_processing(img)
                cv2.imshow("raw", img)
                cv2.imshow("mask", img_2)
                rawCapture.truncate(0)   # Release cache
            
                k = cv2.waitKey(1) & 0xFF
                # 27 is the ESC key
                if k == 27:
                    break

            cv2.destroyAllWindows()
            camera.close()  

    def region_of_interest(self, edges):
        """Crops image to bottom half of screen"""
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height / 2),
            (width, height / 2),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges
    
    def camera_processing(self, img):
        """Takes in frame from Pi camera and applies masking and edge detection"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        edges = cv2.Canny(mask, 200, 400)
        cropped_edges = self.region_of_interest(edges)
        return cropped_edges

if __name__ == "__main__":
    snsr = Sensor()
    # snsr.stream_camera()
    snsr.get_distance()