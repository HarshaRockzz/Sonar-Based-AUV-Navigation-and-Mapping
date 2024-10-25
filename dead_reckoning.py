import math
import numpy as np
import csv
import time
import matplotlib.pyplot as plt
from multiprocessing import Queue, Process

# Convert latitude and longitude to cartesian coordinates (meters)
def lat_lon_to_cartesian(lat, lon):
    earth_radius = 6378137.0  # Earth's radius in meters
    origin_shift = 2 * math.pi * earth_radius / 2.0

    # Convert latitude and longitude to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    # Convert from latitude/longitude to meters
    x = lon_rad * earth_radius
    y = math.log(math.tan((math.pi / 4) + (lat_rad / 2))) * earth_radius

    return x, y

# Convert from cartesian to latitude and longitude
def cartesian_to_lat_lon(x, y):
    earth_radius = 6378137.0  # Earth's radius in meters
    origin_shift = 2 * math.pi * earth_radius / 2.0

    # Convert from meters to latitude/longitude
    lon = (x / origin_shift) * 180.0
    lat = (y / origin_shift) * 180.0

    # Adjust latitude for the Mercator projection
    lat = 180.0 / math.pi * (2.0 * math.atan(math.exp(lat * math.pi / 180.0)) - math.pi / 2.0)

    return lat, lon

def cartesian_to_polar(x, y):
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    return r, theta

# Calculate yaw, pitch, and roll
def calculate_orientation(lat1, lon1, lat2, lon2):
    # Convert lat/lon to cartesian coordinates
    x1, y1 = lat_lon_to_cartesian(lat1, lon1)
    x2, y2 = lat_lon_to_cartesian(lat2, lon2)

    # Calculate deltas
    delta_x = x2 - x1
    delta_y = y2 - y1
    
    # Yaw: angle in the x-y plane (horizontal movement)
    yaw = math.atan2(delta_y, delta_x)  # Heading or yaw in radians

    # Pitch: since there's no altitude, assume pitch is 0 (flat plane assumption)
    pitch = 0.0
    
    # Roll: assuming no roll (flat movement)
    roll = 0.0

    # Convert to degrees
    yaw_deg = math.degrees(yaw)
    
    return yaw_deg, pitch, roll

# Function to simulate dead reckoning based on bearing and distance
def dead_reckoning_simulation(queue):
    latitudes = []
    longitudes = []
    bearing_angles = []
    distances_km = []  # Using distances_km to avoid confusion
    position_data = []
    prev_time = "08:32:10"  # Just an initial value, adjust as needed
    
    with open("C:/Users/mamid/OneDrive/Desktop/Data Transfer/saved_points.csv", 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader, None)  # Skip the header row if present
        
        for row in csv_reader:
            # Reading the relevant columns from the CSV
            latitude = float(row[0])
            longitude = float(row[1])
            wave_latitude = float(row[2])
            wave_longitude = float(row[3])
            bearing_angle = float(row[4])
            distance_km = float(row[5])  # Ensure correct access to distance

            # Append to lists for later use or debugging
            latitudes.append(latitude)
            longitudes.append(longitude)
            bearing_angles.append(bearing_angle)
            distances_km.append(distance_km)
            position_data.append((latitude, longitude))

            # Debugging prints for checking local and wave positions
            print(f"Local Latitude, Longitude: {latitude}, {longitude}")
            print(f"Wave Latitude, Longitude: {wave_latitude}, {wave_longitude}")
            print(f"Bearing Angle: {bearing_angle}°, Distance: {distance_km} km")
            
            # Assuming speed is based on the distance in km (can adjust this logic if needed)
            speed = distance_km  
            
            # Calculate heading (same as bearing angle in this case)
            heading = bearing_angle
            
            # Altitude and polar coordinates (hypothetical values, adjust as needed)
            altitude = 0  # You can change this logic to compute actual altitude from the data if available
            r, theta = cartesian_to_polar(latitude, longitude)
            
            # Send data to the queue (send it as a dictionary for clarity)
            queue_data = {
                "Latitude": latitude,
                "Longitude": longitude,
                "Wave Latitude": wave_latitude,
                "Wave Longitude": wave_longitude,
                "Bearing Angle": bearing_angle,
                "Distance (km)": distance_km,
                "Heading": heading,
                "Speed": speed,
                "Altitude": altitude,
                "Polar_Radius": r,
                "Polar_Theta": theta,
                "Time": prev_time  # Keeping the time as constant, update logic if you need real time
            }
            queue.put(queue_data)

            # Check queue size and content
            print(f"Queue size after put: {queue.qsize()}")  # Check queue size
            print("Data sent to queue:", queue_data)
            
            prev_time = "new_time"  # Update this with actual time logic if needed


def plot_data(queue):
    lat, lon = [], []
    last_update_time = time.time()

    while True:
        if not queue.empty():
            data = queue.get()
            lat.append(data["Latitude"])
            lon.append(data["Longitude"])

            # Only update the plot every 0.5 seconds to avoid unnecessary refreshes
            if time.time() - last_update_time >= 0.5:
                plt.clf()
                plt.plot(lon, lat)
                plt.xlabel("Longitude")
                plt.ylabel("Latitude")
                plt.title("Dead Reckoning Path")
                plt.draw()
                plt.pause(0.01)
                last_update_time = time.time()

def main():
    queue = Queue()
    process_simulation = Process(target=dead_reckoning_simulation, args=(queue,))

    try:
        process_simulation.start()

        plt.ion()
        plot_data(queue)

    except KeyboardInterrupt:
        print("Terminating simulation and plot...")

    finally:
        if process_simulation.is_alive():
            process_simulation.terminate()

        process_simulation.join()

if __name__ == "__main__":
    main()
