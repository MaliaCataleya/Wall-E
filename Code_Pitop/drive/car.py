from time import sleep
from pitop import Camera
import cv2
import time
from pitop import BrakingType, EncoderMotor, ForwardDirection

import image
import detect 

motor_left = EncoderMotor("M3", ForwardDirection.COUNTER_CLOCKWISE)
motor_right = EncoderMotor("M0", ForwardDirection.CLOCKWISE)
motor_left.braking_type = BrakingType.COAST
motor_right.braking_type = BrakingType.COAST

cam = Camera(format="OpenCV")

# Start der gespeicherten Frames definieren --> entkommentieren, wenn Frames als Bilder gespeichert werden sollen
#img_counter = 857

# Grundgeschwindigkeit
default_speed = 0.2

# wird gesetzt, sobald das erste Stoppschild erkannt wird
stop_timer = None


def turn(steering_angle):  
    delta_degree = abs(steering_angle-90)

    # Check ob Lenkwinkel mehr als 15° von 90° abweicht --> erst dann wird gelenkt, sonst geradeaus
    if delta_degree > 15:
        speed = round((((delta_degree*(0.44-default_speed))/45)+default_speed), 4) # Berechnung der benötigten Lenkgeschwindigkeit abhängig vom Winkel (linear, basierend auf Dreisatz)

        # Check, ob berechnete Geschwindigkeit Höchstgeschwindigkeit überschreitet
        if speed > 0.447:
            speed = 0.447
        
        # Definition gefahrener Strecke in Kurven
        curve = 0.08

        if default_speed == 0.1: # True, wenn in erkannter 30er-Zone (siehe handleDetection())
            speed = speed - 0.1 # Lenkgeschwindigkeit wird ebenfalls reduziert, damit der äußere Reifen nicht zu schnell dreht

        # Check, ob Links- oder Rechtskurve
        if steering_angle >= 90:
            motor_right.forward(speed, curve) 
            motor_left.forward(default_speed, curve)
        else:            
            motor_left.forward(speed, curve)
            motor_right.forward(default_speed, curve)
    else:
        motor_left.forward(default_speed, 0.2)
        motor_right.forward(default_speed, 0.2)

def handleDetection(bounding_box, category, speed, stop_timer):
    x, _, _, h = bounding_box.origin_x, bounding_box.origin_y, bounding_box.width, bounding_box.height
    if x < 130 and h > 50:
        if category == "stop":
            if stop_timer:
                if (time.time() - stop_timer) > 3: # Check ob in den letzten 3 Sekunden ein Stoppschild erkannt wurde, falls überhaupt schon eins erkannt wurde
                    motor_left.stop()
                    motor_right.stop()
                    speed = default_speed
                    sleep(3) # Wall-E stoppt für 3 Sekunden, da Stoppschild erkannt
                    stop_timer = time.time()
            else:
                motor_left.stop()
                motor_right.stop()
                speed = default_speed
                sleep(3)
                stop_timer = time.time()
        elif category == "no30":
            speed = 0.2
        elif category == "30":
            speed = 0.1
    
    return speed, stop_timer

try:
    start = time.time()
    while True:
        if (time.time() - start >= 0.7): # Alle 0.7 Sekunden wird ein Frame verarbeitet
            frame = cam.get_frame()

            #------- entkommentieren, um Bilder zu sammeln
            #cv2.imwrite("/home/pi/Documents/repo/Wall-E/Bilder/train/frame_{}.jpg".format(img_counter), frame)
            #img_counter += 1
            #-------

            # Ausführen der Object Detection --> Parameter siehe detect.py
            detection_result = detect.run("/home/pi/Documents/repo/Wall-E/Code_Pitop/drive/schilder.tflite", frame, 3, False)
            if len(detection_result.detections) > 0: # Check, ob Schilder erkannt wurden
                for detection in detection_result.detections:
                    for category in detection.categories:
                            speed, stop_timer = handleDetection(detection.bounding_box, category.category_name, default_speed, stop_timer)
                            default_speed = speed

            steering_angle = image.run(frame)
            turn(steering_angle)
            start = time.time()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
