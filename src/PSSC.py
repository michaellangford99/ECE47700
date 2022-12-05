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

CALIBRATE = 50
WALL = 50
PADDING = 30

baseSpeed = 2 #base angle speed for roll/pitch depending on direction
# LiDAR order:
# 0x29 = front +base
# 0x2a = left +base
# 0x2b = back -base
# 0x2c = right -base

#open logging file
f = open("log.txt", "w")

#Setup VL53L1X
MAX_DISTANCE = 2000.0
UPDATE_TIME_MICROS = 10000
INTER_MEASUREMENT_PERIOD_MILLIS = 20

#Setup GPIO for XSHUT to change I2C address
# The number indicates what GPIO Pin is being used for sensors 0, 1, 2, 3, respectively
GPIO.setmode(GPIO.BOARD) #Change if numbers are wrong
XSHUT = 21
XSHUT1 = 22
XSHUT2 = 24
XSHUT3 = 25

GPIO.setup(XSHUT, GPIO.OUT)
GPIO.setup(XSHUT1, GPIO.OUT)
GPIO.setup(XSHUT2, GPIO.OUT)
GPIO.setup(XSHUT3, GPIO.OUT)

GPIO.output(XSHUT, GPIO.LOW)
GPIO.output(XSHUT1, GPIO.LOW)
GPIO.output(XSHUT2, GPIO.LOW)
GPIO.output(XSHUT3, GPIO.HIGH)

tof3 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof3.open()
tof3.change_address(new_address = 0x2c)

GPIO.output(XSHUT, GPIO.LO)
GPIO.output(XSHUT1, GPIO.LOW)
GPIO.output(XSHUT2, GPIO.HIGH)
GPIO.output(XSHUT3, GPIO.LOW)

tof2 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof2.open()
tof2.change_address(new_address = 0x2b)

GPIO.output(XSHUT, GPIO.LOW)
GPIO.output(XSHUT1, GPIO.HIGH)
GPIO.output(XSHUT2, GPIO.LOW)
GPIO.output(XSHUT3, GPIO.LOW)

tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof1.open()
tof1.change_address(new_address = 0x2a)

GPIO.output(XSHUT, GPIO.HIGH)
GPIO.output(XSHUT1, GPIO.HIGH)
GPIO.output(XSHUT2, GPIO.HIGH)
GPIO.output(XSHUT3, GPIO.HIGH)

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()  # Initialise the i2c bus and configure the sensor

# tof.set_distance_mode(3)

# Lower timing budgets allow for faster updates, but sacrifice accuracy
tof.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)
tof1.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)
tof2.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)
tof3.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)
# Start ranging, mode 0 to leave timing unchanged
tof.start_ranging(0)
tof1.start_ranging(0)
tof2.start_ranging(0)
tof3.start_ranging(0)

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
    tof1.stop_ranging()
    tof2.stop_ranging()
    tof3.stop_ranging()
    sys.stdout.write("\n")
    sys.exit(0)

#Setup serial port
ser = serial.Serial('/dev/serial0', 115200)

signal.signal(signal.SIGINT, exit_handler)

running = True 
reading = 0
cycle = 0;
initial = True #says if drone is calibrated to the first wall
needCalib = True
start = True
final = False

censorInUse = 0
startAvg = 0
calibDir = 1

currentData = 0
data = []
data1 = []
data2 = []
data3 = []

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
    #get ranging for other 4 sensors
    distance_in_mm = tof.get_distance()  # Grab the range in mm
    distance_in_mm2 = tof1.get_distance()
    distance_in_mm3 = tof2.get_distance()
    distance_in_mm4 = tof3.get_distance()
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
    
    #Initial Drone to wall calibration
    if len(data) < 10:
        data.append(distance_in_mm)
        data1.append(distance_in_mm2)
        data2.append(distance_in_mm3)
        data3.append(distance_in_mm4)
        
        yaw = 0
        pitch = 0
        roll = 0
        
        packed = struct.pack('b c h h h h f f f b', start, str(camReady), distance_in_mm, distance_in_mm, distance_in_mm, distance_in_mm, yaw, pitch, roll, end)
        ser.write(packed)
        time.sleep(INTER_MEASUREMENT_PERIOD_MILLIS / 1000.0)
        cycle += 1
        
    elif initial:
        data[currentData] = distance_in_mm
        data1[currentData] = distance_in_mm2
        data2[currentData] = distance_in_mm3
        data3[currentData] = distance_in_mm4
        currentData += 1
        if currentData >= 10:
            currentData = 0
        
        avg = sum(data) / len(data)
        avg1 = sum(data1) / len(data1)
        avg2 = sum(data2) / len(data2)
        avg3 = sum(data3) / len(data3)
        
        if needCalib:
            if start:
                if avg == min(avg, avg1, avg2, avg3):
                    censorInUse = 0
                    startAvg = avg
                    start = False
                elif avg1 == min(avg, avg1, avg2, avg3):
                    censorInUse = 1
                    startAvg = avg1
                    start = False
                elif avg2 == min(avg, avg1, avg2, avg3):
                    censorInUse = 2
                    startAvg = avg2
                    start = False
                elif avg3 == min(avg, avg1, avg2, avg3):
                    censorInUse = 3
                    startAvg = avg3
                    start = False
            
            #Sweeping motion
            if censorInUse == 0:
                if avg < startAvg - CALIBRATE: 
                    startAvg = avg
                elif avg > startAvg + CALIBRATE:
                    if calibDir == -1:
                        caibDir = 1
                    else:
                        calibDir = -1
                elif avg < startAvg + CALIBRATE and avg > startAvg - CALIBRATE:
                    if final:
                        initial = False
                    needCalib = False
                    calibDir = 0
            elif censorInUse == 1:
                if avg1 < startAvg:
                    startAvg = avg1
                elif avg1 > startAvg + CALIBRATE:
                    if calibDir == -1:
                        caibDir = 1
                    else:
                        calibDir = -1
                elif avg1 < startAvg + CALIBRATE and avg1 > startAvg - CALIBRATE:
                    if final:
                        initial = False
                    needCalib = False
                    calibDir = 0
            elif censorInUse == 2:
                if avg2 < startAvg:
                    startAvg = avg2
                elif avg2 > startAvg + CALIBRATE:
                    if calibDir == -1:
                        caibDir = 1
                    else:
                        calibDir = -1
                elif avg2 < startAvg + CALIBRATE and avg2 > startAvg - CALIBRATE:
                    if final:
                        initial = False
                    needCalib = False
                    calibDir = 0
            elif censorInUse == 3:
                if avg3 < startAvg:
                    startAvg = avg3
                elif avg3 > startAvg + CALIBRATE:
                    if calibDir == -1:
                        caibDir = 1
                    else:
                        calibDir = -1
                elif avg3 < startAvg + CALIBRATE and avg3 > startAvg - CALIBRATE:
                    if final:
                        initial = False
                    needCalib = False
                    calibDir = 0
            
            currentCalcSpeed = (baseSpeed * calibDir) * math.exp(-reading / 10)
            reading += 1
            
            yaw = currentCalcSpeed
            pitch = 0
            roll = 0
            
        else:
            if censorInUse == 0:
                if avg < WALL - PADDING:
                    curDist = avg
                    reverse = -1
                    direction = True
                elif avg > WALL + PADDING:
                    curDist = avg
                    reverse = 1
                    direction = True
                elif avg < WALL + PADDING and avg > WALL - PADDING:
                    curDist = 0
                    needCalib = True
                    final = True
            elif censorInUse == 1:
                if avg1 < WALL - PADDING:
                    curDist = avg1
                    reverse = -1
                    direction = True
                elif avg1 > WALL + PADDING:
                    curDist = avg1
                    reverse = 1
                    direction = True
                elif avg1 < WALL + PADDING and avg1 > WALL - PADDING:
                    curDist = 0
                    needCalib = True
                    final = True
            elif censorInUse == 2:
                if avg2 < WALL - PADDING:
                    curDist = avg2
                    reverse = 1
                    direction = True
                elif avg2 > WALL + PADDING:
                    curDist = avg2
                    reverse = -1
                    direction = True
                elif avg2 < WALL + PADDING and avg2 > WALL - PADDING:
                    curDist = 0
                    needCalib = True
                    final = True
            elif censorInUse == 3:
                if avg3 < WALL - PADDING:
                    curDist = avg3
                    reverse = 1
                    direction = True
                elif avg3 > WALL + PADDING:
                    curDist = avg3
                    reverse = -1
                    direction = True
                elif avg3 < WALL + PADDING and avg3 > WALL - PADDING:
                    curDist = 0
                    needCalib = True
                    final = True
            #find Speed
            currentCalcSpeed = (baseSpeed * ((curDist * reverse) / MAX_DISTANCE)) * math.exp(-reading / 10)
            reading += 1
            #get ypr
            yaw = 0
            if !direction:
                pitch = currentCalcSpeed
                roll = 0
            else:
                pitch = 0
                roll = currentCalcSpeed
            
        #add crc (LiDAR data high and low byte xored together)
        packed = struct.pack('b c h h h h f f f b', start, str(camReady), distance_in_mm, distance_in_mm, distance_in_mm, distance_in_mm, yaw, pitch, roll, end)
        ser.write(packed)
        time.sleep(INTER_MEASUREMENT_PERIOD_MILLIS / 1000.0)
        cycle += 1
        
        if finished:
            initial = False

    elif needCalib:
        data[currentData] = distance_in_mm
        data1[currentData] = distance_in_mm2
        data2[currentData] = distance_in_mm3
        data3[currentData] = distance_in_mm4
        currentData += 1
        if currentData >= 10:
            currentData = 0
        
        avg = sum(data) / len(data)
        avg1 = sum(data1) / len(data1)
        avg2 = sum(data2) / len(data2)
        avg3 = sum(data3) / len(data3)
        
        if start:
            if censorInUse == 0:
                startAvg = avg
                start = False
            elif censorInUse == 1:
                startAvg = avg1
                start = False
            elif censorInUse == 2:
                startAvg = avg2
                start = False
            elif censorInUse == 3:
                startAvg = avg3
                start = False
        
        #Sweeping motion
        if censorInUse == 0:
            if avg < startAvg - CALIBRATE: 
                startAvg = avg
            elif avg > startAvg + CALIBRATE:
                if calibDir == -1:
                    caibDir = 1
                else:
                    calibDir = -1
            elif avg < startAvg + CALIBRATE and avg > startAvg - CALIBRATE:
                needCalib = False
                calibDir = 0
        elif censorInUse == 1:
            if avg1 < startAvg:
                startAvg = avg1
            elif avg1 > startAvg + CALIBRATE:
                if calibDir == -1:
                    caibDir = 1
                else:
                    calibDir = -1
            elif avg1 < startAvg + CALIBRATE and avg1 > startAvg - CALIBRATE:
                needCalib = False
                calibDir = 0
        elif censorInUse == 2:
            if avg2 < startAvg:
                startAvg = avg2
            elif avg2 > startAvg + CALIBRATE:
                if calibDir == -1:
                    caibDir = 1
                else:
                    calibDir = -1
            elif avg2 < startAvg + CALIBRATE and avg2 > startAvg - CALIBRATE:
                needCalib = False
                calibDir = 0
        elif censorInUse == 3:
            if avg3 < startAvg:
                startAvg = avg3
            elif avg3 > startAvg + CALIBRATE:
                if calibDir == -1:
                    caibDir = 1
                else:
                    calibDir = -1
            elif avg3 < startAvg + CALIBRATE and avg3 > startAvg - CALIBRATE:
                needCalib = False
                calibDir = 0
        
        currentCalcSpeed = (baseSpeed * calibDir) * math.exp(-reading / 10)
        reading += 1
        
        yaw = currentCalcSpeed
        pitch = 0
        roll = 0
        
        packed = struct.pack('b c h h h h f f f b', start, str(camReady), distance_in_mm, distance_in_mm, distance_in_mm, distance_in_mm, yaw, pitch, roll, end)
        ser.write(packed)
        time.sleep(INTER_MEASUREMENT_PERIOD_MILLIS / 1000.0)
        cycle += 1
    
    #ADD CALIBRATION TO WALL AFTER EVERY DIRECTION CHANGE
    else:
        data[currentData] = distance_in_mm
        data1[currentData] = distance_in_mm2
        data2[currentData] = distance_in_mm3
        data3[currentData] = distance_in_mm4
        currentData += 1
        if currentData >= 10:
            currentData = 0
        
        avg = sum(data) / len(data)
        avg1 = sum(data1) / len(data1)
        avg2 = sum(data2) / len(data2)
        avg3 = sum(data3) / len(data3)
        
        #find curDist
        if distance_in_mm4 <= WALL and distance_in_mm > WALL - PADDING:
            curDist = distance_in_mm
            reverse = 1
            direction = True
        elif distance_in_mm <= WALL and distance_in_mm2 > WALL - PADDING:
            curDist = distance_in_mm2
            reverse = 1
            direction = False
        elif distance_in_mm2 <= WALL and distance_in_mm3 > WALL - PADDING:
            curDist = distance_in_mm3
            reverse = -1
            direction = True
        elif distance_in_mm3 <= WALL and distance_in_mm4 > WALL - PADDING:
            curDist = distance_in_mm4
            reverse = -1
            direction = False
        else:
            curDist = 0
            reverse = 1
            direction = True
        
        #find Speed
        currentCalcSpeed = (baseSpeed * ((curDist * reverse) / MAX_DISTANCE)) * math.exp(-reading / 10)
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