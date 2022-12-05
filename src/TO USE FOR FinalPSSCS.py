import os
import time
import sys
import signal
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import struct
import math
import RPi.GPIO as GPIO

import serial

import VL53L1X

baseSpeed = 2 #base angle speed for roll/pitch depending on direction

#open logging file
f = open("log.txt", "w")

# LiDAR order:
# 0x29 = front +base
# 0x2a = left +base
# 0x2b = back -base
# 0x2c = right -base

#Setup VL53L1X
MAX_DISTANCE = 2000.0
UPDATE_TIME_MICROS = 10000
INTER_MEASUREMENT_PERIOD_MILLIS = 20

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()  # Initialise the i2c bus and configure the sensor

# tof.set_distance_mode(3)

# Lower timing budgets allow for faster updates, but sacrifice accuracy
tof.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)

# Start ranging, mode 0 to leave timing unchanged
tof.start_ranging(0)

#PiCamera Initialization
frameWidth = 640
frameHeight = 480

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))

cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

#Actual running sequence
sys.stdout.write("\n")

def exit_handler(signal, frame):
    global running
    running = False
    tof.stop_ranging()
    #tof1.stop_ranging()
    #tof2.stop_ranging()
    #tof3.stop_ranging()
    sys.stdout.write("\n")
    sys.exit(0)

#Setup serial port
ser = serial.Serial('/dev/serial0', 115200)

signal.signal(signal.SIGINT, exit_handler)

running = True 
reading = 0
cycle = 0;
while running:
    if reading >= 50:
        reading = 0
    #read in and parse message
    if cycle >= 50:
        inMessage = ser.read_until('\n', 20)
        f.write(inMessage)
        cycle = 0
    
    #create and send mesage
    start = 65
    end = 66
    #getranging for other 4 sensors
    distance_in_mm = tof.get_distance()  # Grab the range in mm
    distance_in_mm2 = MAX_DISTANCE
    distance_in_mm3 = MAX_DISTANCE
    distance_in_mm4 = 50
    #print(distance_in_mm)
    #ser.write(str(distance_in_mm).encode() + b'\n')
    if cap is None or not cap.isOpened():
        camReady = 0
    else:
        camReady = 1
        
    #Navigation bologna
    curDist = distance_in_mm
    reverse = 1 #negative if 3 or 4 sensor
    direction = True #pitch = true, roll = false

    #find curDist CHANGE TO 450 DIST INSTEAD
    if distance_in_mm4 <= 50 and distance_in_mm > 20:
        curDist = distance_in_mm
        reverse = 1
        direction = True
    elif distance_in_mm <= 50 and distance_in_mm2 > 20:
        curDist = distance_in_mm2
        reverse = 1
        direction = False
    elif distance_in_mm2 <= 50 and distance_in_mm3 > 20:
        curDist = distance_in_mm3
        reverse = -1
        direction = True
    elif distance_in_mm3 <= 50 and distance_in_mm4 > 20:
        curDist = distance_in_mm4
        reverse = -1
        direction = False
    else:
        curDist = 0
        reverse = 1
        direction = True
    
    #find Speed
    currentCalcSpeed = (baseSpeed * ((curDist * reverse) / MAX_DISTANCE)) * math.exp(-reading / 5)
    reading += 1
    #get ypr
    yaw = 0
    if direction:
        pitch = currentCalcSpeed
        roll = 0
    else:
        pitch = 0
        roll = currentCalcSpeed
    
    #add crc (LiDAR data high and low byte xored together)
    packed = struct.pack('b c h h h h f f f b', start, str(camReady), distance_in_mm, distance_in_mm, distance_in_mm, distance_in_mm, yaw, pitch, roll, end)
    ser.write(packed)
    print(pitch)
    print(roll)
    time.sleep(INTER_MEASUREMENT_PERIOD_MILLIS / 1000.0)
    cycle += 1

f.close()
'''
def func(a):
    pass

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 240)#480
cv2.createTrackbar("Hue Min", "HSV",  0, 179, func)
cv2.createTrackbar("Hue Max", "HSV",  179, 179, func)
cv2.createTrackbar("Saturation Min", "HSV",  0, 255, func)
cv2.createTrackbar("Saturation Max", "HSV",  255, 255, func)
cv2.createTrackbar("Value Min", "HSV",  0, 255, func)
cv2.createTrackbar("Value Max", "HSV",  255, 255, func)

for frame in camera.capture_continuous(rawCapture, format = 'bgr', use_video_port = True):
    img = frame.array
    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("Hue Min","HSV")
    h_max = cv2.getTrackbarPos("Hue Max","HSV")
    s_min = cv2.getTrackbarPos("Saturation Min","HSV")
    s_max = cv2.getTrackbarPos("Saturation Max","HSV")
    v_min = cv2.getTrackbarPos("Value Min","HSV")
    v_max = cv2.getTrackbarPos("Value Max","HSV")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)

    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    hStack = np.hstack([img, mask, result])
    print(mask)
    
    rawCapture.truncate(0)
    
    # cv2.imshow("original", img)
    # cv2.imshow("HSV Color Space", imgHsv)
    # cv2.imshow("Mask", mask)
    # cv2.imshow("Result", result)
    cv2.imshow("Horizontal Stacking", hStack)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
'''