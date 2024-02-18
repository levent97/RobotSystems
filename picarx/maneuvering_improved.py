#!/usr/bin/python3

from math import tan, pi

try :
    from robot_hat import __reset_mcu__
    __reset_mcu__()
    time.sleep(0.01)
    from robot_hat import *
except ImportError:
    from sim_robot_hat import *

from picarx_classes import Motors, Sensors, Interpreters, Controllers, CVSteering
from picarx_SIMU import simultaneity
from picarx_MM import multimodal
import time
import cv2


import logging
logging_format = "%(asctime) s : %(message) s " 
logging.basicConfig(level = logging.INFO) #format = logging_format, level = logging.INFO, )#datefmt ="% H :% M :% S ")


def move(self,v,len,ang):
    self.set_dir_servo_angle(ang)
    self.forward(v,ang)
    time.sleep(len)
    self.stop()

def p_park(self,v,len, dir=-1):
    self.set_dir_servo_angle(dir*40)
    self.backward(v,dir*40)
    time.sleep(0.5*len)
    self.set_dir_servo_angle(-dir*40)
    self.backward(v,-dir*40)
    time.sleep(0.5*len)

def k_turn(self,v,len, dir=-1):
    self.set_dir_servo_angle(dir*40)
    self.forward(v,-dir*40)
    time.sleep(0.5*len)
    self.set_dir_servo_angle(-dir*40)
    self.backward(v,dir*40)
    time.sleep(0.5*len)
    self.set_dir_servo_angle(dir*40)
    self.forward(v,-dir*40)
    time.sleep(0.5*len)  

def gray_follow_line(m,s,i,c,v):    
    while True:        
        position = i.get_grayscale_value(s.get_adc_value())
        c.line_following(position,v)
        
def wall_stop(s,c):
    while True: 
        distance = s.get_distance()
        logging.info("distance reading: {0}".format(distance))
        c.wall_checking(distance)

def cv_follow_line(cvs,c,v):
    while True:
        frame = cvs.start_cv()
        edges = cvs.look_for_color(frame)
        cropped_edges = cvs.crop_video(edges)
        line_segments = cvs.detect_line_segments(cropped_edges)
        path = cvs.average_slope_intercept(cvs, frame, line_segments)
        new_angle = cvs.steering_angle(path)
        adjusted_angle = cvs.steering_angle_adjustment(new_angle, turn_limit = 30)
        c.line_following(adjusted_angle/-30,v)


if __name__ == "__main__":
    m = Motors()
    s = Sensors()
    i = Interpreters()
    c = Controllers(m)
    cvs = CVSteering()
    choice = input('Choose an action to take: (park, forward, kturn, \
                   grayfollow, camerafollow, grayfollow2, grayfollow3, wallstop)')
    if choice == 'forward':
        print('moving forward...')
        move(m,50,2,0)
    elif choice == 'park':
        print('parking...')
        p_park(m,75, 1.75,-1)
    elif choice == 'kturn':
        print('turning around...')
        k_turn(m,75,2.25,-1)
    elif choice == 'grayfollow':
        print('Following a line using the ADC grayscale sensor')
        gray_follow_line(m,s,i,c,0)
    elif choice == 'camerafollow':
        print('Following a line using the Camera and OpenCV')
        cv_follow_line(cvs, c, 0)
    elif choice == 'grayfollow2':
        print('Following a line using bus & grayscale sensor')
        simultaneity(m,s,i,c,0)
    elif choice == 'grayfollow3':
        print('Following a line using RossROS & grayscale sensor')
        multimodal(m,s,i,c,0)
    elif choice == 'wallstop':
        print('Following a line using RossROS & grayscale sensor')
        wall_stop(s,c)
    else:
        print('did nothing...')
        pass
    