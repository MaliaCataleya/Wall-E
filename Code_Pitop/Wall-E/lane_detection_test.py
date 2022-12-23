import math
import cv2
import numpy as np
import matplotlib.pyplot as plt


def make_coordinates(image, line_parameters):
    print("line parameters: ", line_parameters)
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

##TODO: linke, rechte seite getrennt? --> höchstes und niedrigstes jeweils nehmen und linie ziehen!! --> fehler bei average slope

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
        print("left: ", left_fit)
        left_fit_average = np.average(left_fit, axis=0)
        print("left_fit_average: ", left_fit_average)

        #temp = np.zeros_like(image)
        #cv2.line(temp, (x1, y1), (x2, y2), (255, 0, 0), 10)
        left_line = make_coordinates(image, left_fit_average)
    else:
        left_line = np.array([0, 0, 0, 0])
    if right_fit:
        print("right: ", right_fit)
        right_fit_average = np.average(right_fit, axis=0)
        print("right_fit_average: ", right_fit_average)
        right_line = make_coordinates(image, right_fit_average)
    else:
        right_line = np.array([0, 0, 0, 0])
    """ for p in left_line:
        if p < 0:
            left_line = np.array([0, 0, 0, 0])
            break
    for p in right_line:
        if p < 0:
            right_line = np.array([0, 0, 0, 0])
            break """
    return np.array([left_line, right_line])

def canny(image):
    #verwandelt in Graustufen
    """ gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) """

    """ hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 30, 100])
    upper_yellow = ([30, 255, 255])
    mask = cv2.inRange(hsv, np.float32(lower_yellow), np.float32(upper_yellow))
    result = cv2.bitwise_and(image, image, mask=mask) """

    """ img = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 1)

    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l_channel, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(l_channel)
    limg = cv2.merge((cl, a, b))
    enhanced_img = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    #result = np.hstack((image, enhanced_img))

    hsv = cv2.cvtColor(enhanced_img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 30, 100])
    upper_yellow = ([30, 255, 255])
    mask = cv2.inRange(hsv, np.float32(lower_yellow), np.float32(upper_yellow))
    result = cv2.bitwise_and(image, image, mask=mask)

    #reduziert Rauschen über 5x5 Kernel
    blur = cv2.GaussianBlur(result, (5, 5), 0)
    #zeige Ecken mithilfe vom Gradienten
    canny = cv2.Canny(blur, 50, 150) """

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    canny = cv2.Canny(blurred, 260, 300)
    return canny

def display_lines(image, lines):
    line_img = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_img

def region_of_interest(image, width, height):
    """ #definiert Region of Interest nach:
    #plt.imshow(canny)
    #plt.show() statt cv2.imshow() & cv2.waitKey()
    polygon = np.array([[(0, height), (width, height), (width, 300), (0, 300)]]) #[(200, 550), (1200, 550), (650, 250)]
    #füge Polygon in schwarzes Bild ein, um Bit-Vergleich vorzubereiten
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygon, 255)
    #Bit-Vergleich
    masked_image = cv2.bitwise_and(image, mask) """

    polygon = np.array([[(0, height), (width, height), (width, 250), (0, 250)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygon, 255)
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

def calculate_offset(lines, width, height):
    print("lines: ", lines)
    print("0: ", lines[0])
    print("1: ", lines[1])
    if not lines[0].any() and not lines[1].any():
        print("ERROR: NO LINES DETECTED ---------------------------------")
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

def run(image):
    height = image.shape[0]
    width = image.shape[1]

    lane_img = np.copy(image)
    canny_img = canny(lane_img)
    """ plt.imshow(canny_img)
    plt.show() """

    cropped_img = region_of_interest(canny_img, width, height)
    """ cv2.imshow("region of interest", cropped_img)
    cv2.waitKey(0) """
    lines = cv2.HoughLinesP(cropped_img, 1, np.pi/180, threshold=50, minLineLength=10, maxLineGap=10) #113
    #print("lines: ", lines)

    line_img1 = display_lines(lane_img, lines)
    cv2.imshow("blablaabla", line_img1)
    cv2.waitKey(0)

    blank_img = np.zeros((cropped_img.shape[0], cropped_img.shape[1]), np.uint8)
    for line in lines:
        x = (line[0][0], line[0][1])
        y = (line[0][2], line[0][3])
        cv2.line(blank_img, x, y, (255, 255, 255), 1)
    

    #lines_two = cv2.HoughLinesP(blank_img, 1, np.pi/180, threshold=10, minLineLength=10, maxLineGap=200)
    """ line_img1 = display_lines(blank_img, lines_two)
    cv2.imshow("bla", line_img1)
    cv2.waitKey(0) """

    averaged_lines = average_slope_intercept(lane_img, lines)

    x_offset, y_offset = calculate_offset(averaged_lines, width, height)
    steering_angle = calculate_steering_angle(x_offset, y_offset)

    line_img = display_lines(lane_img, averaged_lines)
    combo_img = cv2.addWeighted(lane_img, 0.8, line_img, 1, 1)
    heading_img = display_heading_line(combo_img, steering_angle, (0, 0, 255), 10)

    cv2.imshow("cam", heading_img)
    cv2.waitKey(0)
    print("steering angle: ", steering_angle)

    return steering_angle

img = cv2.imread("/home/pi/Documents/Pictures/opencv_frame_10.png")
# cv2.imshow("result", img)
# cv2.waitKey(0)

run(img)