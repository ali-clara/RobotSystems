import concurrent.futures
from readerwriterlock import rwlock
from sensor import Sensor
from interpretor import Interpretor
from controller import Controller


class Bus():
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWriteD()

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message

class Executor():
    def __init__(self):
        # initialize all the busses
        self.gs_bus = Bus()
        self.sonar_bus = Bus()
        self.camera_bus = Bus()
        self.line_interp_bus = Bus()

        self.sensor = Sensor()
        self.interpretor = Interpretor()
        self.controller = Controller()

    def execute(self):
        gs_delay = 0.1
        interp_delay = 0.1
        control_delay = 0.1

        while True:
            with concurrent.futures.ThreadPoolExecutor() as executor:
                eGrayscale = executor.submit(self.sensor.grayscale_producer, self.gs_bus, gs_delay)
                eSonar = executor.submit(self.sensor.sonar_producer, self.sonar_bus, gs_delay)
                eCamera = executor.submit(self.sensor.camera_producer, self.camera_bus, gs_delay)
                eInterpreter = executor.submit(self.interpretor.line_consumer_producer, self.gs_bus, 
                                                self.camera_bus, self.line_interp_bus, interp_delay)
                eController = executor.submit(self.controller.controller_consumer, self.line_interp_bus, 
                                                self.sonar_bus, control_delay)

            eGrayscale.result()
            eSonar.result()
            eCamera.result()
            eInterpreter.result()
            eController.result()

if __name__ == "__main__":
    # bus = Bus()
    # bus.write("Hi")
    # print(bus.read())

    exec = Executor()
    exec.execute()