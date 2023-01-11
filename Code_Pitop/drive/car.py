from time import sleep
from pitop import Camera
import cv2
import numpy as np
import math
import time

from pitop import BrakingType, EncoderMotor, ForwardDirection

import image

motor_left = EncoderMotor("M3", ForwardDirection.COUNTER_CLOCKWISE)
motor_right = EncoderMotor("M0", ForwardDirection.CLOCKWISE)

motor_left.braking_type = BrakingType.COAST
motor_right.braking_type = BrakingType.COAST

#robot = Pitop()

#robot.add_component(DriveController(left_motor_port="M3", right_motor_port="M0"))
#robot.add_component(Camera(resolution=(640, 640), format="PIL", rotate_angle=0, flip_top_bottom=False))

cam = Camera(format="OpenCV")

img_counter = 50

def turn(steering_angle):  
    default_speed = 0.22
    delta_degree = abs(steering_angle-90)

    if delta_degree > 15:
        speed = round((((delta_degree*(0.44-default_speed))/45)+default_speed), 4)
    
        print("speed: ", speed)

        if speed > 0.447:
            speed = 0.447
        
        curve = 0.08

        if steering_angle >= 90:
            motor_right.forward(speed, curve) 
            motor_left.forward(default_speed, curve)
        else:            
            motor_left.forward(speed, curve)
            motor_right.forward(default_speed, curve)
    else:
        motor_left.forward(default_speed, 0.2)
        motor_right.forward(default_speed, 0.2)


try:
    start = time.time()
    while True:
        if (time.time() - start >= 0.8):
            frame = cam.get_frame()

            #TODO: python check for keyboard input
       
            cv2.imwrite("/home/pi/Documents/Bilder/train/frame_{}.png".format(img_counter), frame)
            img_counter += 1

            steering_angle = image.run(frame)

            turn(steering_angle)
            start = time.time()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
