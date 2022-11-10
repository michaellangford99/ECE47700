import os
import time
import sys
import signal

import serial

import VL53L1X

UPDATE_TIME_MICROS = 10000
INTER_MEASUREMENT_PERIOD_MILLIS = 20

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()  # Initialise the i2c bus and configure the sensor

# tof.set_distance_mode(3)

# Lower timing budgets allow for faster updates, but sacrifice accuracy
tof.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)

# Start ranging, mode 0 to leave timing unchanged
tof.start_ranging(0)

sys.stdout.write("\n")

running = True 

def exit_handler(signal, frame):
    global running
    running = False
    tof.stop_ranging()
    sys.stdout.write("\n")
    sys.exit(0)

ser = serial.Serial('/dev/serial0', 115200)

signal.signal(signal.SIGINT, exit_handler)

while running:
    distance_in_mm = tof.get_distance()  # Grab the range in mm                   # Cap at our MAX_DISTANCE
    print(distance_in_mm)
    ser.write(str(distance_in_mm).encode() + b'\n')
    time.sleep(INTER_MEASUREMENT_PERIOD_MILLIS / 1000.0)