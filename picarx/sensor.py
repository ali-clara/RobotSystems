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
    """Class to read and publish sensor data. Includes ultrasonic, grayscale (light), and camera"""
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
        # camera init
        self.cam_val = None
        self.cv = ComputerVis()

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
        self.stream_camera()
        camera_bus.write(self.cam_val)
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
                # capture and process image
                img = frame.array
                self.cv.raw_frame = img
                line_coords, line_img = self.cv.camera_processing
                self.cam_val = line_coords
                # display camera feed with line overlay
                cv2.imshow("lines", line_img)
                rawCapture.truncate(0)   # Release cache
            
                k = cv2.waitKey(1) & 0xFF
                # 27 is the ESC key
                if k == 27:
                    break

            cv2.destroyAllWindows()
            camera.close()  

class ComputerVis():
    """Helper class that processes camera data"""
    def __init__(self):
        self.raw_frame = None

    def create_mask(self):
        """Creates mask in desired color range (blue masking tape)"""
        hsv = cv2.cvtColor(self.raw_frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        return mask
    
    def detect_edges(self, frame):
        """Applies canny edge detection to isolate edges of frame"""
        edges = cv2.Canny(frame, 200, 400)
        return edges

    def region_of_interest(self, frame):
        """Crops frame to bottom half only to remove non-line camera noise"""
        height, width = frame.shape
        mask = np.zeros_like(frame)

        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height / 2),
            (width, height / 2),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_frame = cv2.bitwise_and(frame, mask)
        return cropped_frame

    def detect_line_segments(frame):
        """Uses probabalistic Hough Transform to detect lines in image
            Returns - List of [x1, y1, x2, y2] line segment values"""
        # set params for hough transform (hand-tuned)
        rho = 1  # distance precision in pixels
        angle = np.deg2rad(1)  # angular precision in radians, i.e. 1 degree
        min_threshold = 15 
        min_line_length = 80
        max_line_gap = 10
        line_segments = cv2.HoughLinesP(frame, rho, angle, min_threshold, 
                                    np.array([]), min_line_length, max_line_gap)
        return line_segments

    def make_points(self, line):
        """Takes a slope and intercept, returns the endpoints of the line segment"""
        height, width, _ = self.raw_frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    @log_on_end(logging.DEBUG, "Lane line: {lane_line}")
    def fit_line(self, line_segments):
        """Combines line segments into lane lines
            Returns - List of [x1, y1, x2, y2] lane line values"""
        line_fit = []
        lane_line = []
        if line_segments is None:
            logging.info('No line_segment segments detected')
            return lane_line

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
            lane_line.append(self.make_points(line_fit_average))

        return lane_line

    def display_lines(self, lines, line_color=(0, 255, 0), line_width=2):
        """Adds the detected line on top of the raw video"""
        line_image = np.zeros_like(self.raw_frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(self.raw_frame, 0.8, line_image, 1, 1)
        return line_image

    def camera_processing(self):
        """Takes in frame from Pi camera and applies transformations to
            detect a painters-tape blue line
            Returns - [x1, y1, x2, y2] coordinates of line, original frame with
                detected line overlayed"""
        mask = self.create_mask()
        edges = self.detect_edges(mask)
        cropped_edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(cropped_edges)
        lane_line = self.fit_line(line_segments)
        lane_line_image = self.display_lines(lane_line)
        return lane_line, lane_line_image

if __name__ == "__main__":
    snsr = Sensor()
    snsr.stream_camera()
    #snsr.get_distance()
    #snsr.sense_line()
    #print(snsr.gs_val_list, snsr.sonar_distance)


