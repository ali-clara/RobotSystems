#!/usr/bin/env python3

class fileDB(object):
    def __init__(self, db:str, mode:str=None, owner:str=None):
        '''Init the db_file is a file to save the datas.'''
        pass

    def file_check_create(self, file_path:str, mode:str=None, owner:str=None):
        pass

    def get(self, name, default_value=None):
        """Get value by data's name. Default value is for the arguemants do not exist"""
        return default_value

    def set(self, name, value):
        """Set value by data's name. Or create one if the arguement does not exist"""
        pass

class _Basic_class(object):
    def __init__(self):
        pass

    @property
    def debug(self):
        pass

    @debug.setter
    def debug(self, debug):
        pass

    def run_command(self, cmd):
        pass

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Servo(_Basic_class):
    def __init__(self, pwm):
        pass
    
    def angle(self, angle):
        pass

    def set_pwm(self, pwm_value):
        pass

class I2C(_Basic_class):
    def __init__(self, *args, **kargs):
        pass

    # @_retry_wrapper
    def _i2c_write_byte(self, addr, data):
        pass

    # @_retry_wrapper
    def _i2c_write_byte_data(self, addr, reg, data):
        pass

    # @_retry_wrapper
    def _i2c_write_word_data(self, addr, reg, data):
        pass

    # @_retry_wrapper
    def _i2c_write_i2c_block_data(self, addr, reg, data):
        pass

    # @_retry_wrapper
    def _i2c_read_byte(self, addr):
        pass

    # @_retry_wrapper
    def _i2c_read_i2c_block_data(self, addr, reg, num):
        pass

    # @_retry_wrapper
    def is_ready(self, addr):
        pass

    def scan(self):
        pass

    def send(self, send, addr, timeout=0):
        pass

    def recv(self, recv, addr=0x00, timeout=0):
        pass

    def mem_write(self, data, addr, memaddr, timeout=5000, addr_size=8):
        pass

    # @_retry_wrapper 
    def mem_read(self, data, addr, memaddr, timeout=5000, addr_size=8):
        pass

    def readfrom_mem_into(self, addr, memaddr, buf):
        pass

    def writeto_mem(self, addr, memaddr, data):
        pass

class PWM(I2C):
    def __init__(self, channel, debug="critical"):
        pass

    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
        pass

    def prescaler(self, *prescaler):
        pass

    def period(self, *arr):
        pass

    def pulse_width(self, *pulse_width):
        pass

    def pulse_width_percent(self, *pulse_width_percent):
        pass

class Pin(_Basic_class):
    PULL_NONE = None

    def __init__(self, *value):
        pass

    def check_board_type(self):
        pass

    def init(self, mode, pull=PULL_NONE):
        pass

    def dict(self, *_dict):
        pass

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        return value

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        pass

    def pull(self, *value):
        pass

    def irq(self, handler=None, trigger=None, bouncetime=200):
        pass

    def name(self):
        pass

    def names(self):
        return [self.name]

    class cpu(object):
        def __init__(self):
            pass

class Ultrasonic():
    def __init__(self, trig, echo, timeout=0.02):
        pass

    def _read(self):
        return -1

    def read(self, times=10):
        return -1

class Grayscale_Module(object):
    def __init__(self, pin0, pin1, pin2, reference=1000):
        pass

    def get_line_status(self,fl_list):
        return 'stop'

    def get_grayscale_data(self):
        return [80, 72, 30]

    





