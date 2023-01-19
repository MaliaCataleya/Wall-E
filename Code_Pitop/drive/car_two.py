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

default_speed = 0.1

speed_l, speed_r = 0.1, 0.1
distance = 0

stop_timer = None


def speed(steering_angle, speed_l, speed_r, distance):  
    delta_degree = abs(steering_angle-90)

    if delta_degree > 15:
        speed = round((((delta_degree*(0.44-default_speed))/45)+default_speed), 4)

        if speed > 0.447:
            speed = 0.447

        if steering_angle >= 90:
            speed_r = speed
            distance = 0.3
        else:            
            speed_l = speed
            distance = 0.3
    distance = 0

    return speed_l, speed_r, distance

def handleDetection(bounding_box, category, stop_timer):
    print(category)
    x, y, w, h = bounding_box.origin_x, bounding_box.origin_y, bounding_box.width, bounding_box.height
    print("x: ", x, " y: ", y, " w: ", w, " h: ", h)
    if x < 150:
        if category == "stop":
            if stop_timer:
                if (time.time() - stop_timer) > 3:
                    motor_left.stop()
                    motor_right.stop()
                    sleep(3)
                    stop_timer = time.time()
        elif category == "no30":
            speed_l = 0.1
            speed_r = 0.1
        elif category == "30":
            speed_l = 0.05
            speed_r = 0.05
    
    return speed_l, speed_r, stop_timer

try:
    while True:
        frame = cam.get_frame()
        steering_angle = image.run(frame)

        speed_l, speed_r, distance = speed(steering_angle, speed_l, speed_r, distance)
        motor_left.forward(speed_l, distance)
        motor_right.forward(speed_r, distance)
        
        detection_result = detect.run("/home/pi/Documents/repo/Wall-E/Code_Pitop/drive/schilder.tflite", frame, 640, 480, 3, False)
        if len(detection_result.detections) > 0:
            for detection in detection_result.detections:
                for category in detection.categories:
                        speed_l, speed_r, stop_timer = handleDetection(detection.bounding_box, category.category_name, stop_timer)
            
except KeyboardInterrupt:
    cv2.destroyAllWindows()
