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

default_speed = 0.2

detect_stop = True
stop_timer = None


def turn(steering_angle):  
    delta_degree = abs(steering_angle-90)

    if delta_degree > 15:
        speed = round((((delta_degree*(0.44-default_speed))/45)+default_speed), 4)

        if speed > 0.447:
            speed = 0.447
        
        print("speed: ", speed)
        
        curve = 0.08

        if default_speed == 0.1:
            speed = speed - 0.1

        if steering_angle >= 90:
            motor_right.forward(speed, curve) 
            motor_left.forward(default_speed, curve)
        else:            
            motor_left.forward(speed, curve)
            motor_right.forward(default_speed, curve)
    else:
        motor_left.forward(default_speed, 0.2)
        motor_right.forward(default_speed, 0.2)

def handleDetection(bounding_box, category, speed, detect_stop, stop_timer):
    print(category)
    x, y, w, h = bounding_box.origin_x, bounding_box.origin_y, bounding_box.width, bounding_box.height
    print("x: ", x, " y: ", y, " w: ", w, " h: ", h)
    if x < 130 and h > 50:
        if category == "stop" and detect_stop is True:

            if stop_timer:
                if (time.time() - stop_timer) > 3:
                    detect_signs = False
                    motor_left.stop()
                    motor_right.stop()
                    speed = default_speed
                    sleep(3)
                    stop_timer = time.time()
            else:
                detect_signs = False
                motor_left.stop()
                motor_right.stop()
                speed = default_speed
                sleep(3)
                stop_timer = time.time()
        elif category == "no30":
            detect_stop = True
            speed = 0.2
        elif category == "30":
            detect_stop = True
            speed = 0.1
    
    return detect_stop, speed, stop_timer


try:
    start = time.time()
    while True:
        if (time.time() - start >= 0.7):
            frame = cam.get_frame()
       
            #cv2.imwrite("/home/pi/Documents/repo/Wall-E/Bilder/train/frame_{}.jpg".format(img_counter), frame)
            #img_counter += 1

            detection_result = detect.run("/home/pi/Documents/repo/Wall-E/Code_Pitop/drive/schilder.tflite", frame, 640, 480, 3, False)
            if len(detection_result.detections) > 0:
                for detection in detection_result.detections:
                    for category in detection.categories:
                            stop, speed, stop_timer = handleDetection(detection.bounding_box, category.category_name, default_speed, detect_stop, stop_timer)

                            default_speed = speed
                            detect_stop = stop

            steering_angle = image.run(frame)

            turn(steering_angle)
            start = time.time()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
