from time import sleep
from pitop import Camera
import cv2
import numpy as np
import math
import time

from pitop import BrakingType, EncoderMotor, ForwardDirection

import image
import detect 

from PIL import Image

motor_left = EncoderMotor("M3", ForwardDirection.COUNTER_CLOCKWISE)
motor_right = EncoderMotor("M0", ForwardDirection.CLOCKWISE)

motor_left.braking_type = BrakingType.COAST
motor_right.braking_type = BrakingType.COAST

#robot = Pitop()

#robot.add_component(DriveController(left_motor_port="M3", right_motor_port="M0"))
#robot.add_component(Camera(resolution=(640, 640), format="PIL", rotate_angle=0, flip_top_bottom=False))

cam = Camera(format="OpenCV")

img_counter = 857

def turn(steering_angle):  
    default_speed = 0.2
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
        if (time.time() - start >= 0.7):
            frame = cam.get_frame()
       
            cv2.imwrite("/home/pi/Documents/repo/Wall-E/Bilder/train/frame_{}.jpg".format(img_counter), frame)
            img_counter += 1

            #detection_result = detect.run("/home/pi/Documents/repo/Wall-E/Code_Pitop/drive/schilder.tflite", frame, 640, 480, 3, False)
            #print(detection_result)

            steering_angle = image.run(frame)

            turn(steering_angle)
            start = time.time()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
