from time import sleep
from pitop import Camera
import cv2
import numpy as np
import math

from pitop import BrakingType, EncoderMotor, ForwardDirection

motor_left = EncoderMotor("M3", ForwardDirection.COUNTER_CLOCKWISE)
motor_right = EncoderMotor("M0", ForwardDirection.CLOCKWISE)

motor_left.braking_type = BrakingType.COAST
motor_right.braking_type = BrakingType.COAST

cam = Camera(format="OpenCV")

def canny(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    canny = cv2.Canny(blurred, 260, 300)
    return canny

def region_of_interest(image, width, height):
    polygon = np.array([[(0, height), (width, height), (width, 250), (0, 250)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image
    
def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)

        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    if left_fit:
        left_fit_average = np.average(left_fit, axis=0)
        left_line = make_coordinates(image, left_fit_average)
    else:
        left_line = np.array([0, 0, 0, 0])
    if right_fit:
        right_fit_average = np.average(right_fit, axis=0)
        right_line = make_coordinates(image, right_fit_average)
    else:
        right_line = np.array([0, 0, 0, 0])
    return np.array([left_line, right_line])

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

def calculate_offset(lines, width, height):
    if not lines[0].any() and not lines[1].any():
        print("ERROR: NO LINES DETECTED ---------------------------------")
    elif not lines[0].any() or not lines[1].any():
        print("INFO: ONE LINE DETECTED")
        x_offset, y_offset = one_line_detected(lines, height)
    else:
        x_offset, y_offset = two_lines_detected(lines, width, height)
    return x_offset, y_offset

def one_line_detected(lines, height):
    if lines[0].any():
        x1, _, x2, _ = lines[0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    else:
        x1, _, x2, _ = lines[1]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    return x_offset, y_offset

def two_lines_detected(lines, width, height):
    _, _, left_x2, _ = lines[0]
    _, _, right_x2, _ = lines[1]
    mid = int(width / 2)
    x_offset = ((left_x2 + right_x2) / 2) - mid
    y_offset = int(height / 2)
    return x_offset, y_offset

def calculate_steering_angle(x_offset, y_offset):
    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
    return steering_angle

def display_lines(image, lines):
    line_img = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_img

def display_heading_line(image, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(image)
    height, width, _ = image.shape
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(image, 0.8, heading_image, 1, 1)
    return heading_image

def turn(steering_angle):
    default_speed = 0.2
    delta_degree = abs(steering_angle-90)

    if delta_degree > 10:
        speed = ((delta_degree*(0.44-default_speed))/45)+default_speed

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

cam.on_frame = canny

try:
    while True:
        sleep(1)
        frame = cam.get_frame()
        height = frame.shape[0]
        width = frame.shape[1]
        lane_img = np.copy(frame)

        canny_img = canny(lane_img)
        cropped_img = region_of_interest(canny_img, width, height)
        
        lines = cv2.HoughLinesP(cropped_img, 1, np.pi/180, threshold=50, minLineLength=10, maxLineGap=10) #113

        line_img1 = display_lines(lane_img, lines)

        blank_img = np.zeros((cropped_img.shape[0], cropped_img.shape[1]), np.uint8)
        for line in lines:
            x = (line[0][0], line[0][1])
            y = (line[0][2], line[0][3])
            cv2.line(blank_img, x, y, (255, 255, 255), 1)

        averaged_lines = average_slope_intercept(lane_img, lines)

        x_offset, y_offset = calculate_offset(averaged_lines, width, height)
        steering_angle = calculate_steering_angle(x_offset, y_offset)

        line_img = display_lines(lane_img, averaged_lines)
        combo_img = cv2.addWeighted(lane_img, 0.8, line_img, 1, 1)
        heading_img = display_heading_line(combo_img, steering_angle, (0, 0, 255), 10)

        print("steering angle: ", steering_angle)

        turn(steering_angle)

        """ cv2.imshow("frame", heading_img)
        cv2.waitKey(1) """
except KeyboardInterrupt:
    cv2.destroyAllWindows()
