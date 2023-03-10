import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pitop import Camera
import sys

# Definieren der äußersten Punkte
def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

# Definieren/ Ziehen der gemittelten Linien
def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]

        if slope < 0: # Check, ob steigend oder fallend --> linke oder rechte Seite
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

# Kantenerkennung
def canny(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Definition gelber Farbraum
    lower_yellow = np.array([20, 50, 100]) # bisher bestes Ergebnis 
    upper_yellow = ([30, 255, 255])

    # alternative Farbräume, falls Lichtverhältnis nicht mit oberem kompatibel
    """ lower_yellow = np.array([20, 70, 0])
    upper_yellow = ([30, 255, 255]) """
    """ lower_yellow = np.array([20, 80, 100])
    upper_yellow = ([30, 255, 255]) """

    # Filtern gelber Farbinhalte (nur Straßenmarkierungen)
    mask = cv2.inRange(hsv, np.float32(lower_yellow), np.float32(upper_yellow))
    result = cv2.bitwise_and(image, image, mask=mask)

    # Vorbereitungen für Canny Edge Detection
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    canny = cv2.Canny(blurred, 190, 255, L2gradient=True) # Canny Edge Detection (190 und 255 stellen oberen und unteren threshold dar --> siehe Dokumentation)
    return canny

def display_lines(image, lines):
    line_img = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            try:
                if x1 > (len(line_img) - 1):
                    x1 = (len(line_img) - 1)
                elif x1 < 0: 
                    x1 = 0
                if x2 > (len(line_img) - 1):
                    x2 = (len(line_img) - 1)
                elif x2 < 0: 
                    x2 = 0
                if y1 > (len(line_img[0]) - 1):
                    y1 = (len(line_img[0]) - 1)
                elif y1 < 0: 
                    y1 = 0
                if y2 > (len(line_img[0]) - 1):
                    y2 = (len(line_img[0]) - 1)
                elif y2 < 0: 
                    y2 = 0
                cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 10)
            except:
                print("Error when trying to draw lines: x1: ", x1, "x2: ", x2, "y1: ", y1, "y2: ", y2)
    return line_img

# Entfernen unwichtiger Inhalte
def region_of_interest(image, width, height):
    polygon = np.array([[(0, height), (width, height), (width, 100), (0, 100)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

# Handler, wenn beide Fahrbanmarkierungen erkannt wurden
def two_lines_detected(lines, width, height):
    _, _, left_x2, _ = lines[0]
    _, _, right_x2, _ = lines[1]
    mid = int(width / 2)
    x_offset = ((left_x2 + right_x2) / 2) - mid
    y_offset = int(height / 2)
    return x_offset, y_offset

# Handler, wenn nur eine Fahrbahnmarkierung erkannt wurde
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

# Berechnung Richtungspunkt
def calculate_offset(lines, width, height):
    if not lines[0].any() and not lines[1].any():
        print("ERROR: NO LINES DETECTED ---------------------------------")
    elif not lines[0].any() or not lines[1].any():
        print("INFO: ONE LINE DETECTED")
        x_offset, y_offset = one_line_detected(lines, height)
    else:
        x_offset, y_offset = two_lines_detected(lines, width, height)
    return x_offset, y_offset

# Berechnung Lenkwinkel
def calculate_steering_angle(x_offset, y_offset):
    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
    return steering_angle

# Darstellung der gemittelten Richtungslinie
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

    # Ecken erkennen
    canny_img = canny(lane_img)
    #cv2.imwrite("/home/pi/Documents/repo/Wall-E/Code_Pitop/Pictures/canny2.png", canny_img)

    # unwichtige Inhalte abschneiden
    cropped_img = region_of_interest(canny_img, width, height)
    #cv2.imwrite("/home/pi/Documents/repo/Wall-E/Code_Pitop/Pictures/cropped2.png", cropped_img)

    # Ziehen der Linien
    lines = cv2.HoughLinesP(cropped_img, 1, np.pi/180, threshold=50, minLineLength=10, maxLineGap=10)

    # erkannte Linien mitteln
    averaged_lines = average_slope_intercept(lane_img, lines)

    # Berechnung anzusteuernder Punkt (Richtungspunkt)
    x_offset, y_offset = calculate_offset(averaged_lines, width, height)

    # Berechnung Lenkwinkel
    steering_angle = calculate_steering_angle(x_offset, y_offset)

    # entkommentieren, wenn Linien bildlich angezeigt werden sollen --> sinnvoll beim Debuggen
    """ line_img = display_lines(lane_img, averaged_lines)
    #cv2.imwrite("/home/pi/Documents/repo/Wall-E/Code_Pitop/Pictures/lines2.png", line_img)
    combo_img = cv2.addWeighted(lane_img, 0.8, line_img, 1, 1)
    #cv2.imwrite("/home/pi/Documents/repo/Wall-E/Code_Pitop/Pictures/combo2.png", combo_img)
    heading_img = display_heading_line(combo_img, steering_angle, (0, 0, 255), 10)
    #cv2.imwrite("/home/pi/Documents/repo/Wall-E/Code_Pitop/Pictures/heading2.png", heading_img) """

    """ cv2.imshow("cam", heading_img)
    cv2.waitKey(0) """

    return steering_angle

# entkommentieren, wenn nur image.py ausgeführt werden soll/ Wall-E nicht fahren soll
""" cam = Camera(format="OpenCV")
frame = cam.get_frame() """

""" frame = cv2.imread("/home/pi/Documents/repo/Wall-E/Code_Pitop/Pictures/opencv_frame_4.png")
steering = run(frame) """
