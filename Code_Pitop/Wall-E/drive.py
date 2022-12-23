from threading import Thread
from time import sleep
import cv2

from pitop import Camera, DriveController, Pitop

import lane_detection 

# Setup the motors for the rover configuration

robot = Pitop()
robot.add_component(DriveController(left_motor_port="M0", right_motor_port="M3"))
#robot.add_component(Camera())

cam = cv2.VideoCapture(0)
cv2.namedWindow("test")

# Set up logic based on line detection
def drive_based_on_frame(frame, cam):
    robot.drive.forward(0.1, hold=False)
    steering_angle = lane_detection.run(frame)
    #if orig_steering_angle is not steering_angle:
    print("steering angle: ", steering_angle)
    turn(steering_angle)

def turn(steering_angle):
    if steering_angle < 90:
        robot.drive.left(0.25, steering_angle)
    elif steering_angle > 90:
        robot.drive.right(0.25, steering_angle)
    elif steering_angle == 90:
        pass
    #robot.drive.target_lock_drive_angle(steering_angle)
    
        

# Go!

while(True):
    ret, frame = cam.read()
    if ret:
        #cv2.imshow("test1", frame)
        img_name = "/home/pi/Documents/Pictures/opencv_frame_captured.png"
        cv2.imwrite(img_name, frame)
        img = cv2.imread(img_name)
        #cv2.imshow("test", img)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        else:
            #orig_steering_angle = lane_detection.run(frame)
            drive_based_on_frame(img, cam)
       
