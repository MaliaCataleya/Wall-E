import math
import cv2
import numpy as np
import matplotlib.pyplot as plt


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

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
    left_fit_average = np.average(left_fit, axis=0)
    """ print("right: ", right_fit)
    print("left: ", left_fit) """
    right_fit_average = np.average(right_fit, axis=0)
    if left_fit:
        left_line = make_coordinates(image, left_fit_average)
    else:
        left_line = np.array([0, 0, 0, 0])
    if right_fit:
        right_line = make_coordinates(image, right_fit_average)
    else:
        right_line = np.array([0, 0, 0, 0])
    for p in left_line:
        if p < 0:
            left_line = np.array([0, 0, 0, 0])
            break
    for p in right_line:
        if p < 0:
            right_line = np.array([0, 0, 0, 0])
            break
    return np.array([left_line, right_line])

def canny(image):
    #verwandelt in Graustufen
    """ gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = ([30, 255, 255])
    mask = cv2.inRange(hsv, np.float32(lower_yellow), np.float32(upper_yellow))
    result = cv2.bitwise_and(image, image, mask=mask)
    #reduziert Rauschen über 5x5 Kernel
    blur = cv2.GaussianBlur(result, (5, 5), 0)
    #zeige Ecken mithilfe vom Gradienten
    canny = cv2.Canny(blur, 50, 150)
    return canny

def region_of_interest(image, width, height):
    #definiert Region of Interest nach:
    #plt.imshow(canny)
    #plt.show() statt cv2.imshow() & cv2.waitKey()
    polygon = np.array([[(0, 400), (width, 400), (900, 70), (350, 70)]]) #[(200, 550), (1200, 550), (650, 250)]
    #füge Polygon in schwarzes Bild ein, um Bit-Vergleich vorzubereiten
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygon, 255)
    #Bit-Vergleich
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def two_lines_detected(lines, width, height):
    _, _, left_x2, _ = lines[0]
    _, _, right_x2, _ = lines[1]
    mid = int(width / 2)
    x_offset = ((left_x2 + right_x2) / 2) - mid
    y_offset = int(height / 2)
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

def calculate_offset(lines, width, height, motor_left, motor_right):
    if not lines[0].any() and not lines[1].any():
        print("ERROR: NO LINES DETECTED ---------------------------------")
        #cv2.destroyAllWindows()
        motor_left.stop()
        motor_right.stop()
        exit()
    elif not lines[0].any() or not lines[1].any():
        print("INFO: ONE LINE DETECTED")
        x_offset, y_offset = one_line_detected(lines, height)
    else:
        x_offset, y_offset = two_lines_detected(lines, width, height)
    return x_offset, y_offset

def calculate_steering_angle(x_offset, y_offset):
    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
    return steering_angle

def run(image, motor_left, motor_right):
    height = image.shape[0]
    width = image.shape[1]

    lane_img = np.copy(image)
    canny_img = canny(lane_img)

    cropped_img = region_of_interest(canny_img, width, height)
    lines = cv2.HoughLinesP(cropped_img, 2, np.pi/180, 5, np.array([]), minLineLength=4, maxLineGap=100) #113
    """ print("lines: ", lines) """
    if lines is None:
        return 90

    averaged_lines = average_slope_intercept(lane_img, lines)

    x_offset, y_offset = calculate_offset(averaged_lines, width, height, motor_left, motor_right)
    steering_angle = calculate_steering_angle(x_offset, y_offset)

    return steering_angle
