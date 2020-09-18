# TODO: develop SpookyFollower class with servo, position, and angle information
# TODO: add light sensor to take a new static_back image if light intensity changes too much
# TODO: Add way to toggle program on/off (physical switch, input from ESP32, etc)

# importing OpenCV, time and Pandas library
import cv2, time
# importing datetime class from datetime library
from datetime import datetime

#Servo Imports
import RPi.GPIO as GPIO
import time

# CONFIG
MIN_POS= 480
MAX_POS = 11

MIN_ANG = 0
MAX_ANG = 25

POS_UPDATE_OFFSET = 10

def translate(value, MinPos, MaxPos, MinAng, MaxAng):
    PosSpan = MaxPos - MinPos
    AngSpan = MaxAng - MinAng

    valueScaled = float(value - MinPos) / float(PosSpan)
    angle = MinAng + (valueScaled * MaxAng)
    return (2+(angle/18))


# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and define as servo1 as PWM pin
GPIO.setup(8,GPIO.OUT)
servo1 = GPIO.PWM(8, 50) # pin 11 for servo1, pulse 50Hz

# Start PWM running, with value of 0 (pulse off)
servo1.start(0)

# Assigning our static_back to None
static_back = None


# Capturing video
video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.0)
print('get dafuq out da way fam')
time.sleep(3)

x_stored = 0

# Infinite while loop to treat stack of image as video
while True:
    # Reading frame(image) from video
    check, frame = video.read()

    # Initializing motion = 0(no motion)
    motion = 0

    # Converting color image to gray_scale image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Converting gray scale image to GaussianBlur
    # so that change can be find easily
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # In first iteration we assign the value
    # of static_back to our first frame
    if static_back is None:
        static_back = gray
        continue

    # Difference between static background
    # and current frame(which is GaussianBlur)
    diff_frame = cv2.absdiff(static_back, gray)

    # If change in between static background and
    # current frame is greater than 30 it will show white color(255)
    thresh_frame = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)[1]
    thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)

    rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 30))
    threshed = cv2.morphologyEx(thresh_frame, cv2.MORPH_CLOSE, rect_kernel)
    # cv2.imshow('threshed', threshed)

    cnts, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Finding contour of moving object
    # cnts, _ = cv2.findContours(thresh_frame.copy(),
    #                            cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    a = 0
    for contour in cnts:
    #if cnts:

        hull = cv2.convexHull(contour)
        # print(cv2.contourArea(hull))
        if cv2.contourArea(hull) < 8000:
            target = hull
            continue
        motion = 1
        cv2.drawContours(frame, [hull], -1, (0, 0, 255), 1)
        (x, y, w, h) = cv2.boundingRect(hull)  # changed from contour
        # making green rectangle around the moving object
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.putText(frame, 'fucc boi ' + str(a), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)
        # print(x)

        if (x > x_stored + POS_UPDATE_OFFSET or x < x_stored - POS_UPDATE_OFFSET):
            servoPos = translate(x, MIN_POS, MAX_POS, MIN_ANG, MAX_ANG)
            servo1.ChangeDutyCycle(servoPos)
            # time.sleep(1)
            # servo1.ChangeDutyCycle(0)
            x_stored = x
            print(x)
        a += 1


        # Displaying image in gray_scale
    # cv2.imshow("Gray Frame", gray)

    # Displaying the difference in currentframe to
    # the staticframe(very first_frame)
    # cv2.imshow("Difference Frame", diff_frame)

    # Displaying the black and white image in which if
    # intensity difference greater than 30 it will appear white
    # cv2.imshow("Threshold Frame", thresh_frame)

    # Displaying color frame with contour of motion of object
    cv2.imshow("Color Frame", frame)

    key = cv2.waitKey(1)
    # if q entered whole process will stop
    if key == ord('q'):
        break

video.release()

# Destroying all the windows
cv2.destroyAllWindows()