import cv2
import numpy as np

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

    def detect_line_segments(self, frame):
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

    #@log_on_end(logging.DEBUG, "Lane line: {lane_line}")
    def fit_line(self, line_segments):
        """Combines line segments into lane lines
            Returns - List of [x1, y1, x2, y2] lane line values"""
        line_fit = []
        lane_line = []
        if line_segments is None:
            # logging.info('No line_segment segments detected')
            return lane_line

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    # logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
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