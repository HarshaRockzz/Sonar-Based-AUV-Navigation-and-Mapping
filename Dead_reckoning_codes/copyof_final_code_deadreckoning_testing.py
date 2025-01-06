import time
import math
import serial
import pynmea2
import os
# import ms5837
from decimal import *
from subprocess import call
import csv
from csv import writer
from datetime import datetime
import sys, tty, termios
#from wldvl import WlDVL
import numpy as np
import sys, tty, termios, pigpio
from math import sin,cos
import random
import math
from datetime import datetime
import matplotlib.pyplot as plt
servo1=27#T1
servo2=18#T2
servo3=17#T3
servo4=13#T4
# #time.sleep(90)
time.sleep(5)
import threading
stop_threads = False
dit = pigpio.pi()
time_c="0:0:0"
time_prev="0:0:0"                                         
current_datetime = datetime.now()
fnd=current_datetime.strftime("%b_%d_%Y_%H_%M_%S")
str_current_datetime = str(fnd)
file_name = str_current_datetime+".csv"
file = open(file_name, 'w') 
print("File created : ", file.name)
file.close()

#time.sleep(.1)
bus = smbus.SMBus(1)
sensor = ms5837.MS5837_30BA() 
time.sleep(.1)

yaw=0
pitch=0
roll=0
Vx=0
Vy=0
Vz=0
Ax=0
Ay=0
Az=0
speed_prev=0
d_lat=0
d_lon=0
d_t=0
pmbar=0
depth=0
temper=0
latitude=0
longitude=0
al=0
sp=0
alt=0
speed=0
altitude=0
fom=0
covariance0=0
covariance1=0
covariance2=0
covariance3=0
covariance4=0
covariance5=0
covariance6=0
covariance7=0
covariance8=0
time_of_validity=0
time_of_transmission=0
timex=0
status=0
pos=[0,0,0]
pos_lat_long=[0,0,0]
estimated_longitude=0
estimated_latitude=0
gtime=[]
vx_data=[]
vy_data=[]
vz_data=[]
delt=1
roll_data=[0.0]
pitch_data=[0.0]
head_data=[0.0]
velocity_data=[]
heading=0
win = 0
wib = 0
velo_city=[0,0,0]
vel=velo_city
slope=[0,0,0]
position_array=[0,0,0]
position_ary=[0,0,0]
position_n=[]
position_e=[]
position_d=[]
prev_pos_d=0
prev_pos_n=0
prev_pos_e=0
cnb = 0
cbn = 0
valid=0
updated_position_lat=[]
updated_position_long=[]
updated_position_depth=[]
ini_latitude=0
ini_longitude=0
T1=0
T2=0
T3=0
T4=0
cpu_temp=0
distance=0 
bear_angle=0
confidance = 0  

class AUV:
    global ini_latitude,ini_longitude
    def __init__(self, initial_position=(0, 0, 0),initial_heading=0):
        self.position = initial_position  # Initial position (x, y) in local coordinates
        #self.heading = initial_heading    # Initial heading (angle in radians)
        
        # Earth coordinates ( latitude,longitude,0)
        self.earth_position = (13.555582,80.026043,0)
        #self.earth_position = (ini_latitude,ini_longitude,0)
        self.a = 6378137.0  # semi-major axis in meters
        self.f = 1 / 298.257223563  # flattening
        self.b = self.a * (1 - self.f)  # semi-minor axis
        self.e_sq = 1 - (self.b**2 / self.a**2)  # eccentricity squared# Initial position in Earth's coordinate system
    def lat_lon_to_cartesian(self,lat, lon):
    # Define constants for the projection
        earth_radius = 6378137.0  # Earth's radius in meters
        origin_shift = 2 * math.pi * earth_radius / 2.0

        # Convert latitude and longitude to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        # Convert from latitude/longitude to meters
        x = lon_rad * earth_radius
        y = math.log(math.tan((math.pi / 4) + (lat_rad / 2))) * earth_radius

        return x, y
    def cartesian_to_lat_lon(self,x, y):
    # Define constants for the projection
        earth_radius = 6378137.0  # Earth's radius in meters
        origin_shift = 2 * math.pi * earth_radius / 2.0

        # Convert from meters to latitude/longitude
        lon = (x / origin_shift) * 180.0
        lat = (y / origin_shift) * 180.0

        # Adjust latitude for the Mercator projection
        lat = 180.0 / math.pi * (2.0 * math.atan(math.exp(lat * math.pi / 180.0)) - math.pi / 2.0)

        return lat, lon
    
    def dead_reckoning(self,initial_position, velocities, orientation, time_interval):
        """
        Estimate new position using dead reckoning with yaw, pitch, roll, and velocities.

        :param initial_position: tuple (x, y, z) representing initial position in meters
        :param velocities: tuple (vx, vy, vz) representing velocities in meters per second
        :param orientation: tuple (yaw, pitch, roll) in radians
        :param time_interval: time interval in seconds
        :return: tuple (new_x, new_y, new_z) representing the new position
        """
        # Extract velocities
        vx, vy, vz = velocities
        

        # Extract orientation angles
        roll, pitch, yaw = orientation

        # Create rotation matrices for yaw, pitch, and roll
        R_roll = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R_pitch = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_yaw = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Combined rotation matrix
        R = np.dot(R_roll, np.dot(R_pitch, R_yaw))
        print("velocities",vx,vy,vz)
        print("R",R)
        print("Time_interval",time_interval)

        # Calculate the change in position
        delta_position = np.dot(R, np.array([vx, vy, vz]) * time_interval)

        # Calculate new position
        new_position = initial_position + delta_position
        lat=float(round(self.earth_position[0],7))
        lon=float(round(self.earth_position[1],7))
        alt=float(round(self.earth_position[2],7))
        x,y=self.lat_lon_to_cartesian(lat,lon)
        new_x=round(x,7)+delta_position[1]
        new_y=round(y,7)+delta_position[0]
        new_z=delta_position[2]
        new_lat, new_lon=self.cartesian_to_lat_lon(new_x, new_y)
        self.position=new_position
        self.earth_position=(round(new_lat,7),round(new_lon,7),new_z)


def time_difference(time1_str,time2_str):
    # Convert the time strings to datetime objects
    time_format = "%H:%M:%S"
    time1 = datetime.strptime(time1_str, time_format)
    time2 = datetime.strptime(time2_str, time_format)
    #print("time1,time2",time1,time2)
    # Calculate the time difference
    time_difference = time2 - time1
    #print("time difference", time_difference)

    # Extract the difference in hours, minutes, and seconds
    hours, remainder = divmod(time_difference.seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    sec=time_difference.seconds
    #print("sec",sec)
    # Display the result
   # print(f"Time Difference: " sec)
    return time_difference.seconds


def time_calculation():
    global time_prev, time_c
    p_t=str(time_prev).split(':')
    p_t_h=int(p_t[0])
    p_t_m=int(p_t[1])
    p_t_s=int(p_t[2])
    c_t=time_c.split(':')
    c_t_h=int(c_t[0])
    c_t_m=int(c_t[1])
    c_t_s=int(c_t[2])
    h_s=(c_t_h-p_t_h)*3600
    m_s=(c_t_m-p_t_m)*60
    s=c_t_s-p_t_s
    global T,d_t
    T=h_s+m_s+s
    d_t=T
    time_prev=time_c
    print("T",d_t)

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

def convert_to_decimal_degrees(coord):
    degrees = int(coord / 100)
    minutes = coord % 100
    return degrees + minutes / 60

def convert_to_positive_degrees(angle):
    if angle < 0:
        return angle + 360
    else:
        return angle

def convert_to_dms(deg):
    d = int(deg)
    md = abs(deg - d) * 60
    m = int(md)
    sd = (md - m) * 60
    return [d, m, sd]

def data_write():
    while not stop_threads:
        #try:
            print("entered in thread")
            today_1 = datetime.today()
            dt_string = today_1.strftime("%d/%m/%Y %H:%M:%S")
            data=",yaw,"+ str(heading) +',' +',pitch,'+ str(pitch) +',' +',roll,'+ str(roll) +',' +',lati,'+ str(latitude) +',' +',lon,'+ str(longitude) +',' +',alt,'+ str(alt) +',' +',Vx,'+ str(Vx)+',' +',Vy,'+ str(Vy) +',' +',Vz,'+ str(Vz)+','+',speed'+str(speed)+',' +',valid,'+ str(valid)+',' +',altitude,'+ str(altitude)+',' +',fom,'+ str(fom)+',' +',covariance0,'+ str(covariance0)+',' +',covariance1,'+ str(covariance1)+',' +',covariance2,'+ str(covariance2)+',' +',covariance3,'+ str(covariance3)+',' +',covariance4,'+ str(covariance4)+',' +',covariance5,'+ str(covariance5)+',' +',covariance6,'+ str(covariance6)+',' +',covariance7,'+ str(covariance7)+',' +',covariance8,'+ str(covariance8)+',' +',time_of_validity,'+ str(time_of_validity)+',' +',time_of_transmission,'+ str(time_of_transmission)+',' +',time,'+ str(timex)+',' +',status,'+ str(status)+ ','+',ini_longitude,'+','+str(ini_longitude)+','+',ini_latitude,'+','+str(ini_latitude)+','+',T,'+str(d_t)+','+',esimated_longitude,'+str(estimated_longitude)+','+',esimated_latitude,'+str(estimated_latitude)+','+',Depth,'+str(depth)+','+',Ambient_Temperature,'+str(temper)+','+',pressure,'+str(pmbar)+','+',T1,'+str(T1)+','+',T2,'+str(T2)+','+',T3,'+str(T3)+','+',T4,'+str(T4)+','+',cpu_temp,'+str(cpu_temp)+','+',distance_recevied,'+str(distance)+','+',bear_angle_recevied,'+str(bear_angle)+','+',confidance_recived,'+str(confidance)
            msg='Date,'+str(dt_string)+data
            write_data=msg.split(',')
            with open(file_name, 'a') as f_object:
                writer_object = writer(f_object)
                writer_object.writerow(write_data)
                f_object.close()
                print("data written to csv")
#         except:
#                 continue
first_value=False

def read_ins_data():
    global ini_latitude,ini_longitude,roll, pitch, yaw, latitude, longitude,heading
    port = '/dev/ttyUSB1'
    serial_port = '/dev/ttyUSB2'
    ser = serial.Serial(port, 115200)
    gps = serial.Serial(serial_port)
    while not stop_threads:
        try:
            ins_data = ser.readline().decode('utf-8').strip()
            
            gps_data = gps.readline().decode('utf-8').strip()
            
            if gps_data.startswith('$GNGLL'):
            
                # Parse the GPS sentence
                data = pynmea2.parse(gps_data)
                
                # Extract latitude and longitude
                latitude = data.latitude
                longitude = data.longitude
#                 latitude=10.2355
#                 longitude=80.25655
                print("data iss",latitude,longitude)
                
                # Print coordinates
                print(f'Latitude: {latitude:.6f}, Longitude: {longitude:.6f}')
            #ins_data=data.
            if ins_data.startswith(('$GPRMC', '$PASHR', '$GPGGA', '$PRDID')):
                fields = ins_data.split(',')
                if ins_data.startswith('$PRDID'):
                    pitch = float(fields[1])
                    roll = float(fields[2])
                    heading = float(fields[3].split('*')[0])
                    yaw = heading
            print("INS values are",pitch,roll,yaw)       
            
        except:
                print("error")
                continue

def read_dvl_data():
    port1 = '/dev/ttyUSB0'
    ser1 = serial.Serial(port1, 115200)
    global Vx,Vy,Vz,Ax,Ay,Az,valid,altitude,fom,covariance0,covariance1,covariance2,covariance3,covariance4,covariance5,covariance6,covariance7,covariance8,time_of_validity,time_of_transmission,timex,status
    while not stop_threads:
        try:
            #print('entered')
            line1=ser1.readline().decode('utf-8').strip()
            #print('ee')
            print("line1",line1)
            if line1 and line1.startswith('wrz'):
                #print(f"Recieved data:{line1}")
                data_points=[item for part in line1.split(',') for item in part.split(';')]
                Vx=float(data_points[1])
                Vy=float(data_points[2])
                Vz=float(data_points[3])
                Ax=Vx
                Ay=Vy
                Az=Vz
                valid=data_points[4]
                altitude=data_points[5]
                fom=data_points[6]
                covariance0=data_points[7]
                covariance1=data_points[8]
                covariance2=data_points[9]
                covariance3=data_points[10]
                covariance4=data_points[11]
                covariance5=data_points[12]
                covariance6=data_points[13]
                covariance7=data_points[14]
                covariance8=data_points[15]
                time_of_validity=data_points[16]
                time_of_transmission=data_points[17]
                timex=data_points[18]
                status=data_points[19]
                
                print('velocitiees',Vx,Vy,Vz)
                
        except:
                continue
    
    
def presure_sensor():
    bus = smbus.SMBus(1)
    sensor = ms5837.MS5837_30BA()
    global pmbar,depth,temper
    while not stop_threads:
        try:
            if not sensor.init():
                print("Sensor could not be initialized")
                exit(1)
            if sensor.read():
                pmbar = round(sensor.pressure(), 3)
                depth = round(sensor.depth(), 3)
                temper = round(sensor.temperature(), 3)
                print(f"Pressure: {pmbar} mbar, Depth: {depth:.3f} m, Temperature: {temper:.3f} F")
        except:
                continue# print('1')

def dead_reckoning_thread(auv):
    global ini_latitude,ini_longitude,time_prev,estimated_latitude,estimated_longitude
    print(time_prev)
    lon, lat, pos_x, pos_y = [], [], [], []
    while not stop_threads:
#         try:

        orientation = (np.radians(float(roll)), np.radians(float(pitch)), np.radians(float(yaw)))
        velocities = (Vx, Vy, Vz)
        today_1 = datetime.today()
        time_c = today_1.strftime("%H:%M:%S")
        time_interval = float(time_difference(time_prev, time_c))
        initial_position = np.array([auv.position[0], auv.position[1], auv.position[2]])
        auv.dead_reckoning(initial_position, velocities, orientation, time_interval)
        lon.append(float(auv.earth_position[1]))
        lat.append(float(auv.earth_position[0]))
        pos_x.append(float(auv.position[0]))
        pos_y.append(float(auv.position[1]))
        print("Local Position:", auv.position)
        print("Earth Position:", auv.earth_position)
        estimated_latitude=auv.earth_position[0]
        estimated_longitude=auv.earth_position[1]
        time_prev = time_c
        
#         except:
#                 continue



def move_forward():
    msg_data="1552,1552,1462,1500"
    T_data=msg_data.split(",")
    dit.set_servo_pulsewidth(servo1,T_data[0])
    #time.sleep(0.1)  
    dit.set_servo_pulsewidth(servo2,T_data[1])
    #time.sleep(0.1)  
    dit.set_servo_pulsewidth(servo3,T_data[2])
    #time.sleep(0.1)  
    dit.set_servo_pulsewidth(servo4,T_data[3])
    #time.sleep(0.1) 
    

def stop():
    msg_data="1500,1500,1500,1500"
    T_data=msg_data.split(",")
    dit.set_servo_pulsewidth(servo1,T_data[0])
    #time.sleep(0.1)  
    dit.set_servo_pulsewidth(servo2,T_data[1])
    #time.sleep(0.1)  
    dit.set_servo_pulsewidth(servo3,T_data[2])
    #time.sleep(0.1)  
    dit.set_servo_pulsewidth(servo4,T_data[3])
    #time.sleep(0.1) 
    print("stopped")

def turn_left():
        msg_data="1540,1580,1462,1500"
        T_data=msg_data.split(",")
        dit.set_servo_pulsewidth(servo1,T_data[0])
        #time.sleep(0.1)  
        dit.set_servo_pulsewidth(servo2,T_data[1])
        #time.sleep(0.1)  
        dit.set_servo_pulsewidth(servo3,T_data[2])
        #time.sleep(0.1)  
        dit.set_servo_pulsewidth(servo4,T_data[3])
        #time.sleep(0.1) 

def turn_right():
        msg_data="1580,1540,1462,1500"
        T_data=msg_data.split(",")
        dit.set_servo_pulsewidth(servo1,T_data[0])
        #time.sleep(0.1)  
        dit.set_servo_pulsewidth(servo2,T_data[1])
        #time.sleep(0.1)  
        dit.set_servo_pulsewidth(servo3,T_data[2])
        #time.sleep(0.1)  
        dit.set_servo_pulsewidth(servo4,T_data[3])
        #time.sleep(0.1)


z=1
while z<5:
        dit.set_servo_pulsewidth(servo1,1500)
        dit.set_servo_pulsewidth(servo2,1500)
        dit.set_servo_pulsewidth(servo3,1500)
        dit.set_servo_pulsewidth(servo4,1500)
        z=z+1
        time.sleep(.1)
        
count=1

def get_cpu_temperature():
    global cpu_temp
    while not stop_threads:
        cpu_temp = os.popen("vcgencmd measure_temp").readline()
        cpu_temp=cpu_temp.replace("temp=", "").strip()
        print(cpu_temp)

import serial

def read_serial_data():
    global distance, bear_angle, confidance  
    port = '/dev/ttyUSB0'  
    baud_rate = 9600       

    ser = serial.Serial(port, baud_rate)  

    while not stop_threads:  
        try:
            serial_data = ser.readline().decode('utf-8').strip()  

            if serial_data:  
                fields = serial_data.split(',')  
                # if len(fields) >= 3:  
                distance = float(fields[0])
                bear_angle = float(fields[1])
                confidance = float(fields[2])
                print(f"Field 1: {distance}, Field 2: {bear_angle}, Field 3: {confidance}")
        except :
            continue


# def thrusterscontrol():
#     global count,T1,T2,T3,T4
#     while not stop_threads:
#         #try:
#             # if(heading>2 and heading<=15):
# #             if(count<=15):
# #                 msg_data="1500,1500,1430,1430"
# #                 time.sleep(0.1)
#             if(count<=10):
#                 msg_data="1500,1500,1590,1595"
#                 #time.sleep(0.1)
#             elif(count>10 and count<=45):
#                 msg_data="1580,1580,1585,1590"
#                 #time.sleep(0.1)
#             elif(count>45 and count<=60):
#                 msg_data="1580,1540,1585,1590"
#                 #time.sleep(0.1)
#             elif(count>60 and count<=75):
#                 msg_data="1580,1580,1585,1590"
#                 #time.sleep(0.1)        
# #             elif(count>47 and count<=54):
# #                 msg_data="1600,1540,1420,1420"
# #                 time.sleep(0.1)                
#             elif(count>75 and count<=90):
#                 msg_data="1580,1540,1585,1590"
#                 #count=1            
#             else:
#                 msg_data="1500,1500,1450,1450"
# #                 #count=0
#                 #time.sleep(0.1)
#             count=count+1
       
#             T_data=msg_data.split(",")
#             T1=T_data[0]
#             T2=T_data[1]
#             T3=T_data[2]
#             T4=T_data[3]
#             dit.set_servo_pulsewidth(servo1,T_data[0])
#             time.sleep(0.1)
#             dit.set_servo_pulsewidth(servo2,T_data[1])
#             time.sleep(0.1)
#             dit.set_servo_pulsewidth(servo3,T_data[2])
#             time.sleep(0.1)
#             dit.set_servo_pulsewidth(servo4,T_data[3])
#             time.sleep(0.1)
#             #time.sleep(3)
#             print(f"count is equal to value of:{count} is \n")
# #         except:
# #                 continue



def thrusterscontrol():
    global count,T1,T2,T3,T4
    while not stop_threads:
        if(distance>0.3):
             move_forward()
        elif(distance<0.3 and bear_angle>5):
             turn_left()
        elif(distance<0.3 and bear_angle<-5):
             turn_right()
           
#         except:
#                 continue


if __name__ == "__main__":
    today_1 = datetime.today()
    time_prev = today_1.strftime("%H:%M:%S")
    auv = AUV()
    data=threading.Thread(target=data_write).start()
    serial_data_jetson=threading.Thread(target=read_serial_data).start()
    ins=threading.Thread(target=read_ins_data).start()
    dvl=threading.Thread(target=read_dvl_data).start()    
    presure=threading.Thread(target=presure_sensor).start()
    dead=threading.Thread(target=dead_reckoning_thread, args=(auv,)).start()
    thrusters=threading.Thread(target=thrusterscontrol).start()
    act_temp=threading.Thread(target=get_cpu_temperature).start()

    input("Press Enter to stop the threads...\n")
    stop_threads = True
     
    ins.join()
    dvl.join()
    serial_data_jetson.join()
    presure.join()
    act_temp.join()
    data.join()
    print("datasent")
    dead.join()
    thrusters.join()





