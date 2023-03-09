#!/usr/bin/python3

import sys
import cv2
import math
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from object_tracking.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

lock = RLock()

class Sense():
    def __init__(self, target_color):
        # target initialization
        # get LAB range from ros param server
        self.color_range = rospy.get_param('/lab_config_manager/color_range_list', {}) 
        self.__target_color = target_color
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
            }
        self.frame_size = (320, 240)

        # initialize services, subscribers, and publishers
        self.rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)

    def reset_vars(self):
        """Resets the Sense() variables"""
        self.reset_rgb()
        # self.__target_color = ''
    
    # resets the RGB value, sets it to black
    def reset_rgb(self):
        led = Led()
        led.index = 0
        led.rgb.r = 0
        led.rgb.g = 0
        led.rgb.b = 0
        self.rgb_pub.publish(led)
        led.index = 1
        self.rgb_pub.publish(led)

    def get_max_contour(self, contours):
        """Finds the contour with the largest area.
            Inputs - contours (list of contours to compare)
            Returns - largest contour, area of largest contour"""
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # comptute contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 10:  # not sure where the 10 comes from
                    area_max_contour = c

        return area_max_contour, contour_area_max
    
    def find_object_outline(self, frame_lab):
        """If we've detected an object of the target color, 
            get the maximum contour surrounding it"""
        # __target_color is set by the user in set_target()
        if self.__target_color in self.color_range:
            target_color_range = self.color_range[self.__target_color]
            # Perform bitwise operations on original image and mask
            frame_mask = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(target_color_range['max']))
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            # find the outline
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            area_max_contour, area_max = self.get_max_contour(contours)
        else:
            area_max_contour = 0
            area_max = 0

        return area_max_contour, area_max
    
    def draw_crosshairs(self, img, img_h, img_w):
        cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 0, 255), 2)
        cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 0), 2)

    def draw_circle(self, area_max_contour, img, img_h, img_w):
        """Gets and draws the smallest circumscribed circle around
            the detected object"""
        (center_x, center_y), radius = cv2.minEnclosingCircle(area_max_contour)
        center_x = int(Misc.map(center_x, 0, self.frame_size[0], 0, img_w))
        center_y = int(Misc.map(center_y, 0, self.frame_size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, self.frame_size[0], 0, img_w))
        if radius <= 100:
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), self.range_rgb[self.__target_color], 2)

        return center_x, center_y, radius

    def resize_frame(self, img):
        """Resizes frame to desired shape and converts to LAB space"""
        frame_resize = cv2.resize(img, self.frame_size, interpolation=cv2.INTER_NEAREST)
        frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB) 
        return frame_lab

class Act():
    def __init__(self, lock, is_running, sense):
        # status initialization
        self.start_move = True
        self.is_running = is_running
    
        # movement distance initialization
        self.x_dis = 500
        self.y_dis = 0.167
        self.z_dis = 0.2
        self.ik = ik_transform.ArmIK()

        # PID initialization
        self.x_pid = PID.PID(P=0.06, I=0.005, D=0)
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.00003, I=0, D=0)

        self.sense = sense
        self.lock = lock
        
        # initialize ROS things
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.image_pub = rospy.Publisher('/object_tracking/image_result', Image, queue_size=1)  # register result image publisher

        # initialize arm
        rospy.loginfo("Initialize object tracking")
        self.reset_vars()
        self.init_move()
    
    def reset_vars(self):
        """Resets Act() and Sense() variables"""
        with self.lock:
            self.x_dis = 500
            self.y_dis = 0.167
            self.z_dis = 0.2
            self.x_pid.clear()
            self.y_pid.clear()
            self.z_pid.clear()
            self.sense.reset_vars()
    
    def init_move(self, delay=True):
        """Moves the joints to their initial position"""
        with self.lock:
            target = self.ik.setPitchRanges((0, self.y_dis, self.z_dis), -90, -92, -88)
            if target:
                servo_data = target[1]
                bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']),(6, servo_data['servo6'])))
        if delay:
            rospy.sleep(2)
    
    def set_x_distance(self, img_w, center_x):
        self.x_pid.SetPoint = img_w / 2.0  # set up
        self.x_pid.update(center_x)  # current
        dx = self.x_pid.output
        self.x_dis += int(dx)  # output
        # cap x between 200 and 800
        self.x_dis = 200 if self.x_dis < 200 else self.x_dis
        self.x_dis = 800 if self.x_dis > 800 else self.x_dis

    def set_y_distance(self, area_max):
        self.y_pid.SetPoint = 900  # set up
        if abs(area_max - 900) < 50:
            area_max = 900
        self.y_pid.update(area_max)  # current
        dy = self.y_pid.output
        self.y_dis += dy  # output
        # cap y between 0.12 and 25
        self.y_dis = 0.12 if self.y_dis < 0.12 else self.y_dis
        self.y_dis = 0.25 if self.y_dis > 0.25 else self.y_dis

    def set_z_distance(self, img_h, center_z):
        self.z_pid.SetPoint = img_h / 2.0
        self.z_pid.update(center_z)
        dy = self.z_pid.output
        self.z_dis += dy
        self.z_dis = 0.22 if self.z_dis > 0.22 else self.z_dis
        self.z_dis = 0.17 if self.z_dis < 0.17 else self.z_dis

    def run(self, img):
        """Primary image processing and arm controller. Called by image_callback()"""
        # set up image
        img_h, img_w = img.shape[:2]
        self.sense.draw_crosshairs(img, img_h, img_w)
        img_copy = img.copy()
        frame_lab = self.sense.resize_frame(img_copy)
        # find contours around detected color
        area_max_contour, area_max = self.sense.find_object_outline(frame_lab)
        
        # if we've found the largest area, move to it
        if area_max > 100:  
            # draw a circle around the detected object
            center_x, center_z, radius = self.sense.draw_circle(area_max_contour, img, img_h, img_w)
            
            # if we've detected something too large, disregard
            if radius > 100:
                return img
            
            # if we're initialized, move to the target
            if self.start_move:
                self.set_x_distance(img_w, center_x)
                self.set_y_distance(area_max)
                self.set_z_distance(img_h, center_z)
                
                target = self.ik.setPitchRanges((0, round(self.y_dis, 4), round(self.z_dis, 4)), -90, -85, -95)
                if target:
                    servo_data = target[1]
                    bus_servo_control.set_servos(self.joints_pub, 20, (
                        (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, self.x_dis)))
        return img
    
    def image_callback(self, ros_image):
        """Callback function for image_pub. Calls run()"""
        # Convert custom image messages to images
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                        buffer=ros_image.data)
        
        bgr_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        frame = bgr_img.copy()
        frame_result = frame
        with self.lock:
            if self.is_running:
                frame_result = self.run(frame)

        rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
        ros_image.data = rgb_image
        self.image_pub.publish(ros_image)

class Interface():
    def __init__(self, target_color):
        self.lock = RLock()

        self.image_sub = None
        self.is_running = False
        self.org_image_sub_ed = False
        self.heartbeat_timer = None
        
        self.target_color = target_color

        self.sense = Sense(target_color)
        self.act = Act(self.lock, self.is_running, self.sense)
        
        # ROS init
        #self.enter_srv = rospy.Service('/object_tracking/enter', Trigger, self.enter_func)
        #self.exit_srv = rospy.Service('/object_tracking/exit', Trigger, self.exit_func)
        #self.running_srv = rospy.Service('/object_tracking/set_running', SetBool, self.set_running)
        #self.set_target_srv = rospy.Service('/object_tracking/set_target', SetTarget, self.set_target)
        #self.heartbeat_srv = rospy.Service('/object_tracking/heartbeat', SetBool, self.heartbeat_srv_cb)

    def enter_func(self, msg):
        rospy.loginfo("Enter object tracking")
        self.act.init_move()
        with self.lock:
            if not self.org_image_sub_ed:
                self.org_image_sub_ed = True
                self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.act.image_callback)
                
        return [True, 'enter']

    def exit_func(self, msg):
        rospy.loginfo("Exit object tracking")
        with self.lock:
            self.is_running = False
            self.act.is_running = False
            self.act.reset_vars()

            try:
                if self.org_image_sub_ed:
                    self.org_image_sub_ed = False
                    self.heartbeat_timer.cancel()
                    self.image_sub.unregister()
            except BaseException as e:
                rospy.loginfo('%s', e)
            
        return [True, 'exit']
    
    def start_running(self):        
        rospy.loginfo("Start running object tracking")
        with self.lock:
            self.is_running = True
            self.act.is_running = True

    def stop_running(self):
        rospy.loginfo("Stop running object tracking")
        with self.lock:
            self.is_running = False
            self.act.is_running = False
            self.act.reset_vars()
            self.act.init_move(delay=False)

    def set_running(self, msg):
        if msg.data:
            self.start_running()
        else:
            self.stop_running()
            
        return [True, 'set_running']
    
    def set_target(self, msg):
        """gets user input for color and sets it"""        
        rospy.loginfo("Set target %s", msg.data)
        with self.lock:
            self.target_color = msg.data
            self.sense.__target_color = self.target_color
            led = Led()
            led.index = 0
            # in bgr
            led.rgb.r = self.sense.range_rgb[self.target_color][2]
            led.rgb.g = self.sense.range_rgb[self.target_color][1]
            led.rgb.b = self.sense.range_rgb[self.target_color][0]
            self.sense.rgb_pub.publish(led)
            led.index = 1
            self.sense.rgb_pub.publish(led)
            rospy.sleep(0.1)
            
        return [True, 'set_target']
    
    def heartbeat_srv_cb(self, msg):
        if isinstance(self.heartbeat_timer, Timer):
            heartbeat_timer.cancel()
        if msg.data:
            heartbeat_timer = Timer(5, rospy.ServiceProxy('/object_tracking/exit', Trigger))
            heartbeat_timer.start()
        rsp = SetBoolResponse()
        rsp.success = msg.data

        return rsp

if __name__ == "__main__":
    rospy.init_node('object_tracking', anonymous=True)
    
    ## set the target color
    color = 'blue'
        
    interface = Interface(color)
    
    # ROS init
    enter_srv = rospy.Service('/object_tracking/enter', Trigger, interface.enter_func)
    exit_srv = rospy.Service('/object_tracking/exit', Trigger, interface.exit_func)
    running_srv = rospy.Service('/object_tracking/set_running', SetBool, interface.set_running)
    set_target_srv = rospy.Service('/object_tracking/set_target', SetTarget, interface.set_target)
    heartbeat_srv = rospy.Service('/object_tracking/heartbeat', SetBool, interface.heartbeat_srv_cb)
    
    debug = True
    if debug:
        rospy.sleep(0.2)
        interface.enter_func(1)
        
        msg = SetTarget()
        msg.data = color
        
        interface.set_target(msg)
        interface.start_running()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()
