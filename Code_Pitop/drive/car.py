from time import sleep
from pitop import Camera
import cv2
import numpy as np
import math

from pitop import BrakingType, EncoderMotor, ForwardDirection

import image

motor_left = EncoderMotor("M3", ForwardDirection.COUNTER_CLOCKWISE)
motor_right = EncoderMotor("M0", ForwardDirection.CLOCKWISE)

motor_left.braking_type = BrakingType.COAST
motor_right.braking_type = BrakingType.COAST

cam = Camera(format="OpenCV")

img_counter = 0

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

cam.on_frame = image.run

try:
    while True:
        sleep(0.7)
        frame = cam.get_frame()

        #TODO: python check for keyboard input
        k = cv2.waitKey(1)
        if k%256 == 32:
            cv2.imwrite("/home/pi/Documents/train_data/frame_{}.png".format(img_counter), frame)
            img_counter += 1

        steering_angle = image.run(frame)

        turn(steering_angle)
except KeyboardInterrupt:
    cv2.destroyAllWindows()
