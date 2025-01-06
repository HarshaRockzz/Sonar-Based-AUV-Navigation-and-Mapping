import random
import math
from datetime import datetime
import csv
import numpy as np
class AUV:
    def __init__(self, initial_position=(0, 0, 0), initial_heading=0):
        self.position = initial_position  # Initial position (x, y) in local coordinates
        self.earth_position = (13.5556252,80.0259484,0)
    def lat_lon_to_cartesian(sel,lat, lon):
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
   
        earth_radius = 6378137.0  # Earth's radius in meters
        origin_shift = 2 * math.pi * earth_radius / 2.0

        # Convert from meters to latitude/longitude
        lon = (x / origin_shift) * 180.0
        lat = (y / origin_shift) * 180.0

        # Adjust latitude for the Mercator projection
        lat = 180.0 / math.pi * (2.0 * math.atan(math.exp(lat * math.pi / 180.0)) - math.pi / 2.0)

        return lat, lon
    
    def dead_reckoning(self,x0, y0, z0, yaw, pitch, roll, vx, vy, vz, delta_t):
        """
        Estimate new position using dead reckoning with yaw, pitch, roll, and velocities.

        :param initial_position: tuple (x, y, z) representing initial position in meters
        :param velocities: tuple (vx, vy, vz) representing velocities in meters per second
        :param orientation: tuple (yaw, pitch, roll) in radians
        :param time_interval: time interval in seconds
        :return: tuple (new_x, new_y, new_z) representing the new position
        """
        # Extract velocities
        yaw = math.radians(roll)
        pitch = math.radians(pitch)
        roll = math.radians(yaw)

        # Calculate the rotation matrix
        R_yaw = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        R_pitch = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        R_roll = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        # Combined rotation matrix
        R = R_yaw @ R_pitch @ R_roll

        # Velocity vector
        velocity = np.array([vx, vy, vz])

        # Rotate the velocity vector
        rotated_velocity = R @ velocity

        # Calculate the change in position
        delta_position = rotated_velocity * delta_t

        # Update the location
        x_new = x0 + delta_position[0]
        y_new = y0 + delta_position[1]
        z_new = z0 + delta_position[2]

    #return x_new, y_new, z_new
        lat=float(round(self.earth_position[0],7))
        lon=float(round(self.earth_position[1],7))
        alt=float(round(self.earth_position[2],7))
        x,y=self.lat_lon_to_cartesian(lat,lon)
        new_x=round(x,7)+delta_position[1]
        new_y=round(y,7)+delta_position[0]
        new_z=delta_position[2]
        new_lat, new_lon=self.cartesian_to_lat_lon(new_x, new_y)
        self.position=(x_new,y_new,z_new)
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

    
    hours, remainder = divmod(time_difference.seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    sec=time_difference.seconds
    print("sec",sec)
   
   # print(f"Time Difference: " sec)
    return time_difference.seconds


# Example usage:
if __name__ == "__main__":
    # Create AUV object
    auv = AUV()
    lon=[]
    lat=[]
    data=[]
    pos_x=[]
    pos_y=[]
    position_data=[]
    prev_time="13:11:17"
    with open(r"C:\Users\malla\Downloads\L_shape_Aug_31_2024_13_11_17_ki.csv", 'r') as file:
        csv_reader = csv.reader(file)
        #filename=data_niot_dead.csv
        #filename=actualdata1_12_06.csv
        # Skip the header row if present
        next(csv_reader, None)
        for row in csv_reader:
            # Assuming the CSV file has columns for x, y, heading, and possibly other data
            
            # time = str(row[0])
            # #print(row[1])
            # vx = float(row[1])
            
            # vy= float(row[2])
            # vz=float(row[3])
            # #roll=0
            # #pitch=0
            # roll=float(row[4])
            # pitch=float(row[5])
            # yaw = float(row[6])
            time = str(row[1])
            #print(row[1])
            vx = float(row[21])
            
            vy= float(row[24])
            vz=float(row[27])
            #roll=0
            #pitch=0
            roll=float(row[3])
            pitch=float(row[6])
            yaw = float(row[9])
            # Add the data to a list of tuples
            #velocities = (vx, vy,vz)
            #orientation = (np.radians(roll), np.radians(pitch), np.radians(yaw))
            delta_t=1
            #delta_t=time_difference(prev_time,time)
            initial_position=np.array([auv.position[0], auv.position[1], auv.position[2]])
            auv.dead_reckoning(auv.position[0], auv.position[1], auv.position[2], yaw, pitch, roll, vx, vy, vz, delta_t)
            lon.append(float(auv.earth_position[1]))
            lat.append(float(auv.earth_position[0]))
            position_data.append((auv.earth_position[0],auv.earth_position[1]))
            pos_x.append(float(auv.position[0]))
            pos_y.append(float(auv.position[1]))
            print("Local Position:", auv.position)
            print("Earth Position:", auv.earth_position)
            prev_time=time
    
import matplotlib.pyplot as plt
plt.plot(lat,lon)
#print(lat[0],lon[0])
plt.plot(lat[0],lon[0],linewidth=10,color='red',linestyle='--', marker='o', markersize=8)
#plt.plot(pos_x,pos_y)
#plt.plot(act_lat,act_lon)

plt.show()


