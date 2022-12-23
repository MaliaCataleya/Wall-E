import cv2

import lane_detection 
from pitop import BrakingType, EncoderMotor, ForwardDirection

motor_left = EncoderMotor("M3", ForwardDirection.COUNTER_CLOCKWISE)
motor_right = EncoderMotor("M0", ForwardDirection.CLOCKWISE)

motor_left.braking_type = BrakingType.COAST
motor_right.braking_type = BrakingType.COAST

#robot.add_component(Camera())

cam = cv2.VideoCapture(0)

# Set up logic based on line detection
def drive_based_on_frame(frame):
    #cam.on_frame = lane_detection.run
    steering_angle = lane_detection.run(frame, motor_left, motor_right)
    #if orig_steering_angle is not steering_angle:
    print("steering angle: ", steering_angle)
    turn(steering_angle)

def turn(steering_angle):
    default_speed = 0.2
    delta_degree = abs(steering_angle-90)

    if delta_degree > 10:
        speed = ((delta_degree*(0.44-default_speed))/45)+default_speed
    
        print("speed: ", speed)

        if speed > 0.447:
            speed = 0.447

        if steering_angle >= 90:
            motor_right.forward(speed, 0.05)
            motor_left.forward(default_speed, 0.05)
        else:
            motor_left.forward(speed, 0.05)
            motor_right.forward(default_speed, 0.05)
    else:
        motor_left.forward(default_speed, 0.2)
        motor_right.forward(default_speed, 0.2)


def calcSteering(steering_angle):
    pass


def stop():
    motor_right.stop()
    motor_left.stop()
    exit()
        

# Go!

while(True):
    ret, frame = cam.read()
    #cv2.imshow("test1", frame)
    img_name = "/home/pi/Documents/Pictures/opencv_frame_captured.png"
    cv2.imwrite(img_name, frame)
    img = cv2.imread(img_name)
    #cv2.imshow("test", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    else:
        #orig_steering_angle = lane_detection.run(frame)
        drive_based_on_frame(img)
       
