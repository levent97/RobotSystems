#!/usr/bin/python3

import time
from math import tan, pi
import numpy as np
import cv2
import concurrent.futures
from threading import Lock

try :
    from robot_hat import __reset_mcu__
    from robot_hat import *
    __reset_mcu__()
    time.sleep(0.01)
except ImportError :
    from sim_robot_hat import *


import logging
logging_format = "%(asctime) s : %(message) s " 
logging.basicConfig(level = logging.INFO) #format = logging_format, level = logging.INFO, )#datefmt ="% H :% M :% S ")

class Motors:
    steering_dir_val = 0
    
    def __init__(self):
        self.PERIOD = 4095
        self.PRESCALER = 10
        self.TIMEOUT = 0.02
        
        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")
    
        self.Servo_dir_flag = 1
        self.dir_cal_value = -18
        self.cam_cal_value_1 = 5
        self.cam_cal_value_2 = 10
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]

        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)
        
        import atexit
        atexit.register(self.stop)

    def set_motor_speed(self, motor, v):
        motor -= 1
        if v >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif v < 0:
            direction = -1 * self.cali_dir_value[motor]
        v = abs(v)
        v = v - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(v)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(v)
            
            
    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0
    
    def motor_direction_calibration(self, motor, value):
        # 0: positive direction
        # 1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1*self.cali_dir_value[motor]
    
    def dir_servo_angle_calibration(self, value):
        self.dir_cal_value = value
        self.set_dir_servo_angle(self.dir_cal_value)
    
    def set_dir_servo_angle(self, value):
        
        Motors.steering_dir_val = value
        self.dir_servo_pin.angle(value+self.dir_cal_value)
    
    def camera_servo1_angle_calibration(self, value):
        self.cam_cal_value_1 = value
        self.set_camera_servo1_angle(self.cam_cal_value_1)

    
    def camera_servo2_angle_calibration(self, value):
        self.cam_cal_value_2 = value
        self.set_camera_servo2_angle(self.cam_cal_value_2)
    
    def set_camera_servo1_angle(self, value):
        self.camera_servo_pin1.angle(-1 *(value+self.cam_cal_value_1))
    
    def set_camera_servo2_angle(self, value):
        self.camera_servo_pin2.angle(-1 * (value+self.cam_cal_value_2))
    
    def set_power(self, v):
        self.set_motor_speed(1, v)
        self.set_motor_speed(2, v) 
    
    def backward(self, v):
        if Motors.steering_dir_val != 0:
            turn_radius = 9.5/tan((Motors.steering_dir_val* pi/ 180))
            angle_vel = v/turn_radius
            motor_speed = [angle_vel*(turn_radius-5.85), angle_vel*(turn_radius+5.85)]
            motor_speed = [motor_speed[0]/max(motor_speed)*v, motor_speed[1]/max(motor_speed)*v]
    
        else:
            motor_speed = [v,v]
        
        
        self.set_motor_speed(1, motor_speed[0])
        self.set_motor_speed(2, motor_speed[1])
    
    def forward(self, v):
        if Motors.steering_dir_val != 0 and v != 0:
            
            turn_radius = 9.5/tan(Motors.steering_dir_val* pi/ 180)
            angle_vel = v/turn_radius
            motor_speed = [angle_vel*(turn_radius+5.85), angle_vel*(turn_radius-5.85)]
            motor_speed = [motor_speed[0]/max(motor_speed)*v, motor_speed[1]/max(motor_speed)*v]
        else:
            motor_speed = [v,v]
        
        
        self.set_motor_speed(1, -1*motor_speed[0])
        self.set_motor_speed(2, -1*motor_speed[1])
    
    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def test(self):
        self.camera_servo1_angle_calibration(5)
        self.camera_servo2_angle_calibration(10)
        self.dir_servo_angle_calibration(-10) 
        self.set_dir_servo_angle(-60)
        time.sleep(1)
        self.set_dir_servo_angle(0)
        time.sleep(1)
        self.set_motor_speed(1, 1)
        self.set_motor_speed(2, 1)
        self.camera_servo_pin1.angle(0)
        self.camera_servo_pin2.angle(0)
        time.sleep(1)
        self.camera_servo_pin1.angle(-60)
        self.camera_servo_pin2.angle(-60)
    
    def manual_motor_shutdown(self):
        self.stop()
        
    import atexit
    atexit.register(self.stop)


class Sensors:
    def __init__(self):
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')
        self.trig = Pin('D8')
        self.echo = Pin('D9')

    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        logging.info("adcs: {0}".format(adc_value_list))
        return adc_value_list  

    def get_distance(self):
        timeout=0.05
    
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.000015)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()

        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        logging.info("distance(cm): {0}".format(cm))
        return cm
        
class Interpreters:
    def __init__(self):
        self.sensitivity = 200
        self.polarity = 1

    
    def get_grayscale_value(self, adcs):
        if abs(adcs[0] - adcs[2]) > self.sensitivity:
            if adcs[0] < adcs[2]:
                if adcs[0] + abs((adcs[2]-adcs[0])/4) > adcs[1]:
                    rob_pos = .5 * self.polarity   
                else:
                    rob_pos = 1* self.polarity
            else:
                if adcs[2]+abs((adcs[2]-adcs[0])/4) < adcs[1]:
                    rob_pos = -1 * self.polarity   
                else:
                    rob_pos = -.5* self.polarity
        else:
            rob_pos = 0
                
        logging.info("robot pos: {0}".format(rob_pos))
        return rob_pos
          

class Controllers:
    
    def __init__(self,m):
       self.line_steering = -30 
       self.m = m
    def line_following(self, rob_pos, v):
        logging.info("steering angle: {0}, v: {1}".format(rob_pos*self.line_steering,v))
        self.m.set_dir_servo_angle(rob_pos*self.line_steering)
        self.m.forward(v)
        return rob_pos*self.line_steering
    
    def wall_checking(self, cm):
        if 0 < cm < 5:
            logging.info("About to hit an obstacle @ {0}".format(cm))
            self.m.forward(0)

class CVSteering:
    
    def __init__(self):
        pass
    
    def start_cv(self): 
        camera = cv2.VideoCapture(-1)
        camera.set(3, 640)
        camera.set(4, 480)
        _, image = camera.read()
        return image        

            
    cv2.destroyAllWindows()
    
    def look_for_color(self, frame):
        upper_black = np.array([40,40,40])
        lower_black = np.array([0,0,0])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,lower_black,upper_black)
        edges = cv2.Canny(mask, 200, 400)
        return edges
    
    def crop_video(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
            ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges
        
    def detect_line_segments(self, cropped_edges):    

        rho = 1  # distance precision in pixels
        angle = np.pi / 180  # angular precision in radians
        min_threshold = 10
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
            np.array([]), minLineLength=8, maxLineGap=4)
        return line_segments
    
    def average_slope_intercept(self, cvs, frame, line_segments): 
        middle_lines = []
        lane_lines = []
        if line_segments is None:
            return []
        
        for line_segment in line_segments:
            logging.info('line segment: {0}'.format(line_segment))
            for x1, y1, x2, y2 in line_segment:
                if x1 != x2:
                    fit = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = fit[0]
                    intercept = fit[1]
                    middle_lines.append((slope, intercept)) 
                else:
                    logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                    
        if middle_lines is None:
            logging.info('All lines were vertical')
            return []
        lane_lines.append(cvs.make_points(frame, np.average(middle_lines,axis = 0)))
        return lane_lines


    def make_points(self, frame, line):
        if line == []:
            return []
        else:
            height, width, _ = frame.shape
            slope, intercept = line
            y1 = height
            y2 = int(y1 * 1 / 2)

            x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
            x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
            return [x1, y1, x2, y2]
    
    def steering_angle(self, path):
        if path !=[]:
            x1, y1, x2, y2 = path[0]
            x_offset = x2 - x1
            y_offset = y2 - y1
            drive_angle = math.atan(x_offset / y_offset * pi/ 180)
        else:
            drive_angle = 0
        return drive_angle
        
    def steering_angle_adjustment(self, new_angle, turn_limit):
        angle_diff = new_angle - Motors.steering_dir_val
        if abs(angle_diff) > turn_limit:
            adjusted_angle = Motors.steering_dir_val + turn_limit * angle_diff / abs(angle_diff)
        else:
            adjusted_angle = new_angle
        logging.info('Angle from camera: {0}'.format(adjusted_angle))
        return adjusted_angle
    
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
    
    bus_sensor.result()
    bus_interpreter.result()
    bus_controller.result()
        
if __name__ == "__main__":
    pass 