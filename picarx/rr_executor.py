#!/usr/bin/python3
import sys
sys.path.append("C:\\Users\\alicl\\Documents\\GitHub\\RobotSystems\\RossROS")
sys.path.append("/home/ali_pi/RobotSystems/RossROS")

import rossros as rr
from sensor import Sensor
from interpretor import Interpretor
from controller import Controller

# functions to create buses for:
    # sensor.sense_line()
    # sensor.get_distance()
    # sensor.stream_camera()
    # interpretor.grayscale_processing()
    # controller.line_following()
sensor = Sensor()
interpretor = Interpretor()
controller = Controller()

"""Create buses for passing data"""
grayscale_bus = rr.Bus(name = "Grayscale sensor bus")
sonar_bus = rr.Bus(name = "Sonar sensor bus")
camera_bus = rr.Bus(name = "Camera bus")
interp_bus = rr.Bus(name = "Interpretor bus")
control_bus = rr.Bus(name = "Controller bus")
termination_bus = rr.Bus(0, "Termination bus")

"""Wrap signal generation and processing into RossROS objects"""
default_delay = 0.05

read_grayscale = rr.Producer(
    sensor.sense_line,  # function that will generate data
    grayscale_bus,  # output data bus
    default_delay,  # delay between data generation cycles
    termination_bus,  # bus to watch for termination signal
    "Read grayscale sensor data"
)

read_sonar = rr.Producer(
    sensor.get_distance,
    sonar_bus,
    default_delay,
    termination_bus,
    "Read ultrasonic sensor data"
)

read_camera = rr.Producer(
    sensor.stream_camera,
    camera_bus,
    0.4,
    termination_bus,
    "Read camera data"
)

process_data = rr.ConsumerProducer(
    interpretor.grayscale_processing,
    (grayscale_bus, sonar_bus, camera_bus),
    interp_bus,
    default_delay,
    termination_bus,
    "Interpret sensor data"
)

control_car = rr.Consumer(
    controller.line_following,
    interp_bus,
    default_delay,
    termination_bus,
    "Use interpreted sensor data for control"
)

print_buses = rr.Printer(
    (grayscale_bus, sonar_bus, camera_bus, interp_bus, control_bus),
    0.25, # delay between printing cycles
    termination_bus,
    "Print raw and derived data", # name of printer
    "Data bus readings: " # prefix for output
)

termination_timer = rr.Timer(
    termination_bus, # output data bus
    3, # duration
    0.01, # delay between checking for termination time
    termination_bus, # bus to check for termination signal
    "Termination timer"
)

"""Concurrent execution"""

# create a list of producer-consumers to execute concurrently
producer_consumer_list = [read_grayscale, 
                            read_sonar, 
                            read_camera,
                            process_data,
                            control_car,
                            print_buses,
                            termination_timer]

# execute them
rr.runConcurrently(producer_consumer_list)
