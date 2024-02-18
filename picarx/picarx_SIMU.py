#!/usr/bin/python3

import time
import concurrent.futures
from threading import Lock
from picarx_classes import Motors, Sensors, Interpreters, Controllers
from rossros import Bus, ConsumerProducer, Producer, Consumer, Timer, Printer, runConcurrently

try :
    from robot_hat import __reset_mcu__
    from robot_hat import *
    
    __reset_mcu__()
    time.sleep(0.01)
except ImportError :
    print ("Simulator")
    from sim_robot_hat import *

import logging
logging_format = "%(asctime) s : %(message) s " 
logging.basicConfig(level = logging.INFO)

from logdecorator import log_on_start , log_on_end , log_on_error 


class DataBus:
    def __init__(self):
        self.message = 0
        
    def read(self):
        return self.message
    
    def write(self, msg):
        self.message = msg
    
    def test(self):
        print(self.read())
        self.write('Testing read/write')
        print(self.read())


def sensor_producer(s, in_bus, delay):
    lock = Lock()
    while True:
        with lock:
            adcs = s.get_adc_value()
        in_bus.write(adcs)
        time.sleep(delay)
        
    
    
def interpreter_cp(i, in_bus, out_bus, delay):
    while True:
        if in_bus.read() != None:
            position = i.get_grayscale_value(in_bus.read())
            out_bus.write(position)
            time.sleep(delay)
        else:
            time.sleep(delay)
    
def controller_consumer(c, out_bus, delay, v):
    while True:
        if out_bus.read() != None:
            c.line_following(out_bus.read(), v)
            time.sleep(delay)
        else:
            time.sleep(delay)
            
def simultaneity(m,s,i,c, v):

    sensor_delay = 0.2
    interpreter_delay = 0.2
    controller_delay = 0.2
    in_bus = DataBus()
    out_bus = DataBus()
    
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        bus_sensor = executor.submit(sensor_producer, s, in_bus, sensor_delay)
        bus_interpreter = executor.submit(interpreter_cp, i, in_bus, out_bus, interpreter_delay)
        bus_controller = executor.submit(controller_consumer, c, out_bus, controller_delay, v)
    
    logging.info("made it here")
    logging.info(bus_controller)
    bus_sensor.result()
    bus_interpreter.result()
    bus_controller.result()


if __name__ == "__main__":
    b = DataBus()
    b.test()