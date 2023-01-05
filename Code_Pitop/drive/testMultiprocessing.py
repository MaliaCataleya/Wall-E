from time import sleep
from pitop import Camera
import cv2
import numpy as np
import math

from pitop import BrakingType, EncoderMotor, ForwardDirection

from multiprocessing import Pool

import image

motor_left = EncoderMotor("M3", ForwardDirection.COUNTER_CLOCKWISE)
motor_right = EncoderMotor("M0", ForwardDirection.CLOCKWISE)

motor_left.braking_type = BrakingType.COAST
motor_right.braking_type = BrakingType.COAST

cam = Camera(format="OpenCV")

img_counter = 127

def turn(steering_angle):                                
    default_speed = 0.2
    delta_degree = abs(steering_angle-90)

    if delta_degree > 10:
        speed = round((((delta_degree*(0.44-default_speed))/45)+default_speed), 4)
    
        print("speed: ", speed)

        if speed > 0.447:
            speed = 0.447

        if steering_angle >= 90:
            motor_right.forward(speed, 0.08) 
            motor_left.forward(default_speed, 0.08)
        else:            
            motor_left.forward(speed, 0.08)
            motor_right.forward(default_speed, 0.08)
    else:
        motor_left.forward(default_speed, 0.2)
        motor_right.forward(default_speed, 0.2)

#cam.on_frame = image.run #idk if this should be commented, uncomment if buggy

def processFrame(cam):
    frame = cam.get_frame()
    steering_angle = image.run(frame)
    
    return steering_angle

def loopDrive(steering_angle, cam):
    turn(steering_angle)
    
    #thread this
    steering_angle = processFrame(cam)
    
    #start of thread - uncomment 60 and 61 or comment 57
    #with Pool() as mp_pool:
    #    steering_angle = mp_pool.map(processFrame, cam)
    #end of thread -- Alternative: processFrame in turn Funktion
    
    loopDrive(steering_angle)
    
# run program   
   
steering_angle = processFrame(cam)
loopDrive(steering_angle, cam)
