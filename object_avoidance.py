import time
from datetime import datetime
import os
from decimal import *
from subprocess import call
from csv import writer
import time
import serial
import smbus
import ms5837
import sys, tty, termios, pigpio

servo1 = 4
servo2 = 22
servo3 = 18
servo4 = 17

# initialization
dit = pigpio.pi()
current_datetime = datetime.now()
fnd = current_datetime.strftime("%b_%d_%Y_%H_%M_%S")
str_current_datetime = str(fnd)
file_name = str_current_datetime + ".csv"
file = open(file_name, 'w')
print("File created : ", file.name)
file.close()
time.sleep(.1)
bus = smbus.SMBus(1)
sensor = ms5837.MS5837_30BA()
IMU_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=5)
time.sleep(5)

yaw = 0
pitch = 0
roll = 0
pmbar = 0
depth = 0
temper = 0
latitude = 0
longitude = 0
al = 0
sp = 0

z = 1
while z < 5:
    dit.set_servo_pulsewidth(servo1, 1500)
    dit.set_servo_pulsewidth(servo2, 1500)
    dit.set_servo_pulsewidth(servo3, 1500)
    dit.set_servo_pulsewidth(servo4, 1500)
    z = z + 1
    time.sleep(2)


def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


def time_convert(sec):
    mins = sec // 60
    sec = sec % 60
    hours = mins // 60
    mins = mins % 60


l = 1


def data_write():
    today_1 = datetime.today()
    dt_string = today_1.strftime("%d/%m/%Y %H:%M:%S")
    data = (",yaw," + str(yaw) +
            ',pitch,' + str(pitch) +
            ',roll,' + str(roll) +
            ',pressure,' + str(pmbar) +
            ',depth,' + str(depth) +
            ',temper,' + str(temper) +
            ',lati,' + str(latitude) +
            ',lon,' + str(longitude) +
            ',AL,' + str(al) +
            ',sp,' + str(sp) +
            ',conf,' + str(conf) +
            ',distance,' + str(dist) +
            ',width,' + str(width) +
            ',height,' + str(height) +
            ',bearing_angle,' + str(ber_angle)
            )

    msg = 'Date,' + str(dt_string) + data
    write_data = msg.split(',')
    with open(file_name, 'a') as f_object:
        writer_object = writer(f_object)
        writer_object.writerow(write_data)
        f_object.close()

z = 1
while z < 5:
    dit.set_servo_pulsewidth(servo1, 1500)
    dit.set_servo_pulsewidth(servo2, 1500)
    dit.set_servo_pulsewidth(servo3, 1500)
    dit.set_servo_pulsewidth(servo4, 1500)
    z = z + 1
    time.sleep(2)

# Assume initial state variables for servo positions
initial_servo1_pulse = 1500
initial_servo2_pulse = 1500

# Flag to keep track of whether the AUV is avoiding an object
avoiding_object = False

# Variables to store previous state for returning after avoiding the object
previous_servo1_pulse = initial_servo1_pulse
previous_servo2_pulse = initial_servo2_pulse

# Distance to travel forward after detecting an object (in cm)
distance_to_travel = 40

# Flag to track whether the AUV is in the process of moving forward
moving_forward = False

# Time when the AUV started moving forward
forward_start_time = 0

# Duration to move forward (in seconds)
forward_duration = 3

# Flag to indicate if the AUV has reached 0.25m depth
reached_depth = False


count = 1
print("entering in loop")
ser = serial.Serial(
    port='/dev/ttyUSB1',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

reached_depth = False
moving_forward = False
forward_start_time = 0
forward_duration = 5  # 5 seconds
avoiding_object = False
previous_servo1_pulse = 1500
previous_servo2_pulse = 1500

while True:
    try:
        line = IMU_port.readline().decode('utf-8')
        if len(line) == 0:
            print("time out! ")
            sys.exit()

        IMU_info = line.split('=')
        IMU_data = IMU_info[1].split(',')

        yaw = IMU_data[0]
        pitch = IMU_data[1]
        roll = IMU_data[2]
        time.sleep(0.3)

        if not sensor.init():
            print("Sensor could not be initialized")
            exit(1)

        if sensor.read():
            pmbar = round(sensor.pressure(), 3)
            depth = round(sensor.depth(), 3)
            temper = round(sensor.temperature(), 3)

        data = ser.readline().decode().rstrip()
        if data:
            re_data = data.split(',')
            conf = int(re_data[0])
            dist = float(re_data[1])
            width = float(re_data[2])
            height = float(re_data[3])
            ber_angle = float(re_data[4])

        msg_data = "1500,1500,1500,1500"

        print("entering the data in csv ")
        data_write()

        if not reached_depth and depth < 0.2:
            print('AUV diving')
            msg_data = "1500,1500,1580,1580"
            time.sleep(0.3)
            print("down")
        elif 0.2 < depth <= 0.5 or 110 <= yaw < 150:
            if 0.2 < depth <= 0.5:
                msg_data = "1600,1600,1565,1565"
            elif 110 <= yaw < 150:
                msg_data = "1450,1580,1565,1565"
            time.sleep(0.3)

        elif depth > 0.5:
            msg_data = "1500,1500,1500,1500"
            time.sleep(0.4)
        elif reached_depth and not moving_forward:  # If not already moving forward
            # Start moving forward
            moving_forward = True
            forward_start_time = time.time()
            print("Starting forward motion")

        # If AUV is in the process of moving forward
        if moving_forward:
            # Calculate elapsed time since starting to move forward
            elapsed_time = time.time() - forward_start_time
            if elapsed_time < forward_duration:
                # Move forward
                dit.set_servo_pulsewidth(servo1, 1600)
                dit.set_servo_pulsewidth(servo2, 1600)
                print("Moving forward")
            else:
                # Stop moving forward
                moving_forward = False
                # Update previous servo positions for returning after avoiding the object
                previous_servo1_pulse = 1600
                previous_servo2_pulse = 1600
                forward_start_time = 0
                print("Stopping forward motion")

        # Object avoidance
        if not avoiding_object and dist <= 50:  # Object detected within 50cm
            avoiding_object = True
            if ber_angle > 0:  # Object on the right side
                # Adjust servo motors to move AUV right
                dit.set_servo_pulsewidth(servo1, 1600)
                dit.set_servo_pulsewidth(servo2, 1550)
                print("Moving right to avoid object")
            elif ber_angle < 0:  # Object on the left side
                # Adjust servo motors to move AUV left
                dit.set_servo_pulsewidth(servo1, 1550)
                dit.set_servo_pulsewidth(servo2, 1600)
                print("Moving left to avoid object")

        # If avoiding an object, check if object is still within range
        if avoiding_object and dist > 50:  # Object no longer detected within 50cm
            avoiding_object = False
            # Restore servo positions to previous state
            dit.set_servo_pulsewidth(servo1, previous_servo1_pulse)
            dit.set_servo_pulsewidth(servo2, previous_servo2_pulse)
            print("Object avoided, returning to previous state")
        # Rest of the code remains unchanged
        print(count)
        count = count + 1

        if l < 5:
            T_data = msg_data.split(",")
            dit.set_servo_pulsewidth(servo1, T_data[0])
            time.sleep(0.1)
            dit.set_servo_pulsewidth(servo2, T_data[1])
            time.sleep(0.1)
            dit.set_servo_pulsewidth(servo3, T_data[2])
            time.sleep(0.1)
            dit.set_servo_pulsewidth(servo4, T_data[3])
            time.sleep(2)
            # Print out the value and voltage
            l = l + 1

    except:
        continue
