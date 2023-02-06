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
        self.gs_val_list = None
        self.gs_state = None
        # ultrasonic init
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))
        self.sonar_distance = None

    def grayscale_producer(self, gs_bus, delay):
        """Writes data from grayscale sensors to gs_bus.
            Args - gs_bus (Bus() object), delay (time delay, sec).
            Message: [raw_value_left, raw_value_center, raw_value_right]"""
        self.sense_line()
        gs_bus.write(self.gs_val_list)
        time.sleep(delay)

    def sonar_producer(self, sonar_bus, delay):
        """Writes sonar data to sonar_bus
            Args - sonar_bus (Bus() object), delay (time delay, sec).
            Message: raw_value (float?)"""
        self.get_distance()
        sonar_bus.write(self.sonar_distance)
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
        self.sonar_distance = self.ultrasonic.read()

    ######## GRAYSCALE ########
    def set_grayscale_reference(self, value):
        """GS"""
        self.get_grayscale_reference = value
        
    def get_grayscale_data(self):
        """GS, returns raw data from grayscale sensors"""
        return list.copy(self.grayscale.get_grayscale_data())

    def get_line_status(self):
        """GS"""
        return str(self.grayscale.get_line_status(self.gs_val_list))

    def sense_line(self):
        """GS"""
        self.gs_val_list = self.get_grayscale_data()
        self.gs_state = self.get_line_status()

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

    def detect_line_segments(self, cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixels
        angle = np.deg2rad(1)  # angular precision in radians, i.e. 1 degree
        min_threshold = 10 
        min_line_length = 8
        max_line_gap = 4
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), min_line_length, max_line_gap)

        logging.info('line_segments: %s' %line_segments)
        return line_segments

    def make_points(self, frame, line):
        """Takes a slope and intercept, returns the endpoints of the line segment"""
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]
    
    def fit_line(self, frame, line_segments):
        """Combines line segments into lane lines"""
        line_fit = []
        lane_lines = []
        if line_segments is None:
            logging.info('No line_segment segments detected')
            return lane_lines
    
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                line_fit.append((slope, intercept))

        line_fit_average = np.average(line_fit, axis=0)
        if len(line_fit) > 0:
            lane_lines.append(self.make_points(frame, line_fit_average))

        logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
        return lane_lines
    
    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=2):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    
    def camera_processing(self, img):
        """Takes in frame from Pi camera and applies masking and edge detection"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        edges = cv2.Canny(mask, 200, 400)
        cropped_edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.fit_line(img, line_segments)
        lane_lines_image = self.display_lines(img, lane_lines)
        return lane_lines_image


if __name__ == "__main__":
    snsr = Sensor()
    # snsr.stream_camera()
    snsr.get_distance()
    snsr.sense_line()
    print(snsr.gs_val_list, snsr.sonar_distance)