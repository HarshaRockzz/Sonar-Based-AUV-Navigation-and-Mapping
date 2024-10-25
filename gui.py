import sys
import multiprocessing
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QPushButton, QGridLayout
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtWebEngineWidgets import QWebEngineView
import folium
import io
import time
import csv
import geopy.distance
import math
import geopy.distance 
from matplotlib.figure import Figure
from matplotlib.ticker import FormatStrFormatter
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from multiprocessing import Queue
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import random
import matplotlib.patches as patches
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.image import imread
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDial
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtCore import QTimer, QPropertyAnimation, pyqtProperty, QEasingCurve
from data_update_process import data_update
from queue import Empty

class DataUpdateThread(QThread):
    data_signal = pyqtSignal(dict)

    def __init__(self, queue):
        super().__init__()
        self.queue = queue

    def run(self):
        while True:
            try:
                data = self.queue.get(timeout=10)  # Simulated data fetch
                if data:
                    self.data_signal.emit(data)
            except Empty:
                print("No data received within timeout.")
            except Exception as e:
                print(f"Error in data update thread: {e}")

class AnimatedDial(QDial):
    def __init__(self, parent=None):
        super(AnimatedDial, self).__init__(parent)
        self._current_value = 0

        # Animation setup
        self.animation = QPropertyAnimation(self, b"current_value", self)
        self.animation.setEasingCurve(QEasingCurve.OutQuad)  # Smooth easing curve
        self.animation.setDuration(800)  # Animation duration in milliseconds

    @pyqtProperty(int)
    def current_value(self):
        return self._current_value

    @current_value.setter
    def current_value(self, new_value):
        self._current_value = new_value
        self.setValue(new_value)

    def animate_to_value(self, new_value):
        self.animation.stop()  # Stop any ongoing animation
        self.animation.setStartValue(self.current_value)
        self.animation.setEndValue(new_value)
        self.animation.start()  # Start animation

import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QDial
from PyQt5.QtGui import QFont, QColor, QPainter, QPen, QPalette
from PyQt5.QtCore import QRect, Qt

class GaugeWidget(QWidget):
    def __init__(self, label_text, gauge_color, indicator_color, parent=None):
        super(GaugeWidget, self).__init__(parent)
        
        layout = QVBoxLayout(self)
        
        # Set label properties
        self.label = QLabel(label_text)
        self.label.setFont(QFont("Arial", 14))
        self.label.setStyleSheet(f"color: white; background-color: black; padding: 5px;")
        
        # Set dial properties
        self.dial = QDial()
        self.dial.setRange(0, 360)
        self.dial.setNotchesVisible(True)
        
        # Set gauge color
        palette = self.dial.palette()
        palette.setColor(QPalette.Button, QColor(gauge_color))  # Dial's background
        palette.setColor(QPalette.Highlight, QColor(indicator_color))  # Dial's indicator
        self.dial.setPalette(palette)

        layout.addWidget(self.label)
        layout.addWidget(self.dial)
        
        self.setLayout(layout)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Define the region where the dial will be drawn
        rect = QRect(0, 70, self.width(), self.height() - 100)  # Adjust according to your layout
        
        # Add text labels around the dial (e.g., 0, 90, 180, 270, 360 degrees)
        painter.setPen(QPen(Qt.white, 2))
        font = painter.font()
        font.setPointSize(10)
        painter.setFont(font)
        
        # Map positions (0, 90, 180, 270, 360 degrees)
        angle_labels = [(0, "0"), (270, "270"), (180, "180"), (90, "90"), (360, "360")]
        radius = rect.width() // 3  # Adjust this to fit your dial size
        
        for angle, text in angle_labels:
            radian = (angle - 90) * (np.pi / 180)  # Convert degrees to radians
            x = rect.center().x() + radius * np.cos(radian) - 10  # Adjust for text width
            y = rect.center().y() + radius * np.sin(radian) + 5   # Adjust for text height
            painter.drawText(int(x), int(y), text)

    def update_value(self, value):
        self.dial.setValue(int(value))
        self.label.setText(f"{self.label.text().split(':')[0]}: {value:.2f}")

class Object3DWidget(QWidget):
    def __init__(self, parent=None):
        super(Object3DWidget, self).__init__(parent)
        
        self.initUI()

        # Initialize with default values
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.speed = 0

        # Timer to update dials every second
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_dials)
        self.timer.start(1000)

    def initUI(self):
        self.layout = QVBoxLayout(self)
        
        # Create the four gauges
        self.gauges_layout = QHBoxLayout()
        self.yaw_gauge = GaugeWidget("Yaw", "#FF4500", "#FF6347")  # Orange gauge with tomato indicator
        self.pitch_gauge = GaugeWidget("Pitch", "#3CB371", "#2E8B57")  # Medium sea green gauge with sea green indicator
        self.roll_gauge = GaugeWidget("Roll", "#1E90FF", "#4169E1")  # Dodger blue gauge with royal blue indicator
        self.speed_gauge = GaugeWidget("Speed", "#FFD700", "#FFA500")  # Gold gauge with orange indicator

        self.gauges_layout.addWidget(self.yaw_gauge)
        self.gauges_layout.addWidget(self.pitch_gauge)
        self.gauges_layout.addWidget(self.roll_gauge)
        self.gauges_layout.addWidget(self.speed_gauge)

        self.layout.addLayout(self.gauges_layout)
        
    def set_data(self, yaw, pitch, roll, speed):
    # Set the real data from outside the widget
     self.yaw = yaw
     self.pitch = pitch
     self.roll = roll
     self.speed = speed
    
    # Print updated values
     print(f"Set Data - Yaw: {self.yaw}, Pitch: {self.pitch}, Roll: {self.roll}, Speed: {self.speed}")

    def update_dials(self):
     print("Updating Dials...")
    # Update dials with the stored values
     self.yaw_gauge.update_value(self.yaw)
     self.pitch_gauge.update_value(self.pitch)
     self.roll_gauge.update_value(self.roll)
     self.speed_gauge.update_value(self.speed)


        
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np

class PolarPlotWindow(QMainWindow):
    def __init__(self, queue):
        super().__init__()
        self.setWindowTitle("Polar Plot Window")
        self.setGeometry(100, 100, 600, 500)  # Set window size and position
        
        # Initialize queue for incoming data
        self.queue = queue
        
        # Create a central widget for the window
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        
        # Set the layout
        layout = QVBoxLayout(central_widget)
        
        # Create a figure for the polar plot
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        
        # Create a timer to trigger plot updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot_from_queue)
        self.timer.start(1000)  # Update every 1 second (adjust as necessary)
        
        # Initialize data storage
        self.data = []
        
        # Draw the initial polar plot
        self.plot_polar_coordinates()

        # Connect hover event
        self.canvas.mpl_connect('motion_notify_event', self.on_hover)

        # Create a label to show the coordinates
        self.coord_label = QLabel()
        layout.addWidget(self.coord_label)

    def plot_polar_coordinates(self):
        # Clear the previous plot and create a new one
        self.figure.clear()
        ax = self.figure.add_subplot(111, polar=True)
        
        # Extract data if available
        if self.data:
            theta = [item['Polar_Theta'] for item in self.data]
            r = [item['Polar_Radius'] for item in self.data]
            ax.plot(theta, r, 'o')  # Plot with markers
        
        # Refresh the canvas
        self.canvas.draw()

    def update_plot_from_queue(self):
        # Check if there's new data in the queue
        if not self.queue.empty():
            data = self.queue.get()  # Retrieve new data from the queue
            
            # Append new data to the list
            self.data.append(data)
            
            # Optionally limit the amount of data
            if len(self.data) > 100:  # Keep the last 100 data points
                self.data.pop(0)
            
            # Update the polar plot with the new data
            self.plot_polar_coordinates()

    def on_hover(self, event):
        # Check if the event is within the axes
        if event.inaxes:
            # Get the polar axes
            ax = event.inaxes
            # Get the data
            for line in ax.get_lines():
                # Check if the mouse is over a line
                if line.contains(event)[0]:
                    # Get the data points
                    xdata = line.get_xdata()
                    ydata = line.get_ydata()
                    # Find the closest point to the mouse
                    dist = np.sqrt((xdata - event.xdata) ** 2 + (ydata - event.ydata) ** 2)
                    idx = np.argmin(dist)
                    theta = xdata[idx]
                    r = ydata[idx]
                    # Update the label with the coordinates
                    self.coord_label.setText(f'Theta: {theta:.2f}, Radius: {r:.2f}')
                    break
            else:
                # Clear the label if not hovering over a line
                self.coord_label.setText('')

# Example usage:
# Create the polar plot window and show it
def open_polar_plot_window():
    polar_window = PolarPlotWindow()
    polar_window.show()
    return polar_window


class MainWindow(QMainWindow):
    def __init__(self, queue):
        super().__init__()
        self.queue = queue
        self.icon_image = imread(r"C:\Users\mamid\OneDrive\Desktop\Data Transfer\auv-800.jpg") 
        self.setWindowTitle("Main Window")
        self.setGeometry(100, 100, 1000, 600)

        self.data = {
            "Latitude": [],
            "Longitude": [],
            "Speed": [],
            "Distance": [],
            "Heading": [],
            "Altitude": [],
            "Yaw": [],
            "Pitch": [],
            "Roll": [],
            "Polar_Radius": [],
            "Polar_Theta": []
        }

        central_widget = QWidget()
        main_layout = QGridLayout(central_widget)

        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        font = QFont("Arial", 14)
        color = QColor(255, 255, 255)

        self.labels = {}
        self.value_labels = {}
        for key in ["Latitude", "Longitude", "Speed", "Distance", "Heading", "Altitude"]:
            label = QLabel(f"{key}:")
            label.setFont(font)
            label.setStyleSheet(f"color: {color.name()};")
            left_layout.addWidget(label)
            self.labels[key] = label

            value_label = QLabel("0.00")
            value_label.setFont(font)
            value_label.setStyleSheet("background-color: #282828; color: #00FF00; padding: 5px; border-radius: 5px;")
            left_layout.addWidget(value_label)
            self.value_labels[key] = value_label

        frame_label = QLabel("Frame:")
        frame_label.setFont(font)
        frame_label.setStyleSheet(f"color: {color.name()};")
        left_layout.addWidget(frame_label)
        
        self.frame_combo = QComboBox()
        self.frame_combo.setFont(font)
        self.frame_combo.addItems(["Frame 1", "Frame 2", "Frame 3"])
        left_layout.addWidget(self.frame_combo)

        mission_type_label = QLabel("Mission Type:")
        mission_type_label.setFont(font)
        mission_type_label.setStyleSheet(f"color: {color.name()};")
        left_layout.addWidget(mission_type_label)

        self.mission_type_combo = QComboBox()
        self.mission_type_combo.setFont(font)
        self.mission_type_combo.addItems(["Type 1", "Type 2", "Type 3"])
        left_layout.addWidget(self.mission_type_combo)

        submit_button = QPushButton("Start")
        submit_button.setFont(font)
        submit_button.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        submit_button.clicked.connect(self.start_mission)
        left_layout.addWidget(submit_button)

        self.graph_widget = QWidget()
        self.graph_layout = QVBoxLayout(self.graph_widget)
        
        self.graphs = {}
        for field in ["Speed", "Distance", "Heading", "Altitude"]:
            fig, ax = plt.subplots(facecolor='black')
            canvas = FigureCanvas(fig)
            canvas.mpl_connect('motion_notify_event', self.on_hover)
            self.graph_layout.addWidget(canvas)
            self.graphs[field] = (ax, canvas)

        right_layout.addWidget(self.graph_widget)

        lat_lon_widget = QWidget()
        lat_lon_layout = QVBoxLayout(lat_lon_widget)
        fig, ax = plt.subplots(facecolor='black')
        canvas = FigureCanvas(fig)
        canvas.mpl_connect('motion_notify_event', self.on_hover)
        lat_lon_layout.addWidget(canvas)
        self.graphs["Latitude vs Longitude"] = (ax, canvas)

        main_layout.addLayout(left_layout, 0, 0, 1, 1)
        main_layout.addWidget(lat_lon_widget, 0, 1, 1, 3)
        main_layout.addLayout(right_layout, 0, 4, 1, 3)
        
        
        polar_plot_button = QPushButton("Show Polar Plot")
        polar_plot_button.setFont(font)
        polar_plot_button.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        polar_plot_button.clicked.connect(self.open_polar_plot_window)
        main_layout.addWidget(polar_plot_button, 1, 0, 1, 7)

        switch_button = QPushButton("Switch to Map and 3D View")
        switch_button.setFont(font)
        switch_button.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        switch_button.clicked.connect(self.switch_to_map3d)

        main_layout.addWidget(switch_button, 2, 0, 1, 7)

        self.setCentralWidget(central_widget)
        self.setStyleSheet("background-color: #151B54;")
        self.showMaximized()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.start_mission)
        self.timer.start(1000)
        
    def open_polar_plot_window(self):
        self.polar_window = PolarPlotWindow(self.queue)
        self.polar_window.show()
        
    def on_hover(self, event):
        if event.inaxes:
            for field, (ax, canvas) in self.graphs.items():
                if event.inaxes == ax:
                    x, y = event.xdata, event.ydata
                    self.setToolTip(f"x: {x:.2f}, y: {y:.2f}")
                    
    def switch_to_map3d(self):
        self.map_window = Map3DWindow(self)
        self.map_window.show()
        self.close()
    def update_graphs(self):
     for field in self.graphs:
        ax, canvas = self.graphs[field]
        ax.clear()
        ax.set_facecolor('#151B54')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        ax.tick_params(axis='x', colors='white')
        ax.tick_params(axis='y', colors='white')
        
        if field == "Latitude vs Longitude":
            x_data = self.data["Longitude"]
            y_data = self.data["Latitude"]
            x_label = "Longitude"
            y_label = "Latitude"
            title = "Latitude vs Longitude"
            
            ax.plot(x_data, y_data, '-o', color='#ff7f0e', markersize=5, linewidth=2, label=field)
            
            if x_data and y_data and self.icon_image is not None:
                latest_x = x_data[-1]
                latest_y = y_data[-1]
                imagebox = OffsetImage(self.icon_image, zoom=0.1)
                ab = AnnotationBbox(imagebox, (latest_x, latest_y), frameon=False)
                ax.add_artist(ab)
            
            ax.set_xlabel(x_label)
            ax.set_ylabel(y_label)
            ax.set_title(title)

            # Ensure the axis shows 6 decimal places
            ax.xaxis.set_major_formatter(FormatStrFormatter('%.6f'))
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.6f'))

            # Set axis limits based on data
            ax.set_xlim(min(x_data), max(x_data))
            ax.set_ylim(min(y_data), max(y_data))

            # Set 10 ticks on x and y axes using np.linspace
            ax.set_xticks(np.linspace(min(x_data), max(x_data), num=10))
            ax.set_yticks(np.linspace(min(y_data), max(y_data), num=10))

            # Annotating the x and y ticks with formatted values
            ax.set_xticklabels([f'{l:.7f}' for l in np.linspace(min(x_data), max(x_data), num=10)], rotation=45, ha='right', fontsize=8)
            ax.set_yticklabels([f'{l:.7f}' for l in np.linspace(min(y_data), max(y_data), num=10)], fontsize=8)

        else:
            x_data = list(range(len(self.data[field])))
            y_data = self.data[field]
            
            ax.plot(x_data, y_data, '-o', color='#ff7f0e', markersize=5, linewidth=2, label=field)
            ax.set_xlabel("Time")
            ax.set_ylabel(field)
            ax.set_title(f"{field} over Time")
            
            # Convert min and max values to integers for tick calculations
            min_y = int(min(y_data))
            max_y = int(max(y_data))
            
            x_ticks = list(range(0, len(x_data), max(1, len(x_data) // 10)))
            y_ticks = list(range(min_y, max_y + 1, max(1, int((max_y - min_y) / 10))))
            
            ax.set_xticks(x_ticks)
            ax.set_yticks(y_ticks)
            
            # Annotating the ticks with their values
            for tick in x_ticks:
                ax.text(tick, min_y, str(tick), color='white', fontsize=10, ha='center', va='bottom')
            for tick in y_ticks:
                ax.text(0, tick, str(tick), color='white', fontsize=10, ha='right', va='center')

        ax.legend(facecolor='#151B54', labelcolor='white')
        ax.grid(True, color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
        canvas.draw()

    def start_mission(self):
     try:
        if not self.queue.empty():
            # Fetch data from the queue
            data = self.queue.get_nowait()  # Use get_nowait to avoid blocking
            print(f"Data received from queue: {data}")  # Debug print

            # Check if the received data contains required fields
            if all(key in data for key in ["Latitude", "Longitude", "Time", "Speed", "Distance (km)", "Heading", "Altitude", "Polar_Radius", "Polar_Theta"]):
                # Update the data dictionary
                self.data["Latitude"].append(data["Latitude"])
                self.data["Longitude"].append(data["Longitude"])
                self.data["Speed"].append(data["Speed"])
                self.data["Distance"].append(data["Distance (km)"])  # Corrected key
                self.data["Heading"].append(data["Heading"])
                self.data["Altitude"].append(data["Altitude"])
                self.data["Polar_Radius"].append(data["Polar_Radius"])
                self.data["Polar_Theta"].append(data["Polar_Theta"])

                # Add yaw, pitch, and roll if they're provided (assuming you calculate these elsewhere)
                if "Yaw" in data and "Pitch" in data and "Roll" in data:
                    self.data["Yaw"].append(data["Yaw"])
                    self.data["Pitch"].append(data["Pitch"])
                    self.data["Roll"].append(data["Roll"])
                
                # Update the labels with the received data
                self.value_labels["Latitude"].setText(f"{data['Latitude']:.6f}")
                self.value_labels["Longitude"].setText(f"{data['Longitude']:.6f}")
                self.value_labels["Altitude"].setText(f"{self.data['Altitude'][-1]:.1f}")

                # Update other labels
                self.value_labels["Speed"].setText(f"{self.data['Speed'][-1]:.2f}")
                self.value_labels["Distance"].setText(f"{self.data['Distance'][-1]:.2f}")  # Corrected to reflect distance in km
                self.value_labels["Heading"].setText(f"{self.data['Heading'][-1]:.2f}")

                # Update the graphs
                self.update_graphs()
            else:
                print("Incomplete data received from queue.")
        else:
            print("Queue is empty.")
    
     except Empty:
        print("Queue was empty at timeout.")
     except Exception as e:
        print(f"An error occurred: {e}")



import numpy as np  # Make sure you have numpy installed to handle wave generation

import numpy as np
import folium
import geopy.distance
import io
import csv
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtGui import QFont

class Map3DWindow(QMainWindow):
    def __init__(self, main_window):
        super().__init__()

        self.main_window = main_window

        self.setWindowTitle("Map and 3D View")
        self.setGeometry(100, 100, 1000, 600)

        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        map_layout = QHBoxLayout()
        self.map_view = QWebEngineView()

        # Initialize the map
        self.update_map()

        map_layout.addWidget(self.map_view)
        main_layout.addLayout(map_layout)

        back_button = QPushButton("Return to Main Window")
        back_button.setFont(QFont("Arial", 14))
        back_button.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        back_button.clicked.connect(self.return_to_main)

        main_layout.addWidget(back_button)

        self.setCentralWidget(central_widget)
        self.setStyleSheet("background-color: #00008B;")
        self.showMaximized()

    # Interpolate points for a smoother path
    def interpolate_points(self, coords, num_points=5):
        interpolated_coords = []
        for i in range(len(coords) - 1):
            lat1, lon1 = coords[i]
            lat2, lon2 = coords[i + 1]

            # Linear interpolation
            lat_interp = np.linspace(lat1, lat2, num_points)
            lon_interp = np.linspace(lon1, lon2, num_points)

            interpolated_coords.extend(zip(lat_interp, lon_interp))

        return interpolated_coords

    # Function to plot two paths
    # Function to plot two paths with fewer markers
    def plot_map_with_two_paths(self, latitudes, longitudes, wave_latitudes, wave_longitudes):
     original_coords = list(zip(latitudes, longitudes))
     wave_coords = list(zip(wave_latitudes, wave_longitudes))

    # Interpolate the points to generate smoother paths
     interpolated_original_coords = self.interpolate_points(original_coords, num_points=10)
     interpolated_wave_coords = self.interpolate_points(wave_coords, num_points=10)

    # Center the map on the midpoint
     lat_center = sum(latitudes) / len(latitudes)
     lon_center = sum(longitudes) / len(longitudes)
     m = folium.Map(location=[lat_center, lon_center], zoom_start=15)

    # Add the accurate (original) path in green
     folium.PolyLine(interpolated_original_coords, color="green", weight=2.5, opacity=0.8, tooltip="Accurate Path").add_to(m)

    # Add the wave-modified (normal) path in blue
     folium.PolyLine(interpolated_wave_coords, color="blue", weight=2.5, opacity=0.8, tooltip="Normal Path").add_to(m)

    # Add markers on both paths at every 10th point
     marker_interval = 10  # Place markers every 10th point
     for i, coord in enumerate(interpolated_wave_coords):
        if i % marker_interval == 0:  # Plot fewer markers by checking interval
            folium.Marker(location=coord, icon=folium.Icon(color='blue')).add_to(m)

    # Save and return the map
     return m

    # Update the map with new coordinates and paths
    def update_map(self):
        latitudes = self.main_window.data["Latitude"]
        longitudes = self.main_window.data["Longitude"]
        speeds = self.main_window.data["Speed"]
        distances = self.main_window.data["Distance"]
        altitudes = self.main_window.data["Altitude"]

        if len(latitudes) > 0 and len(latitudes) == len(longitudes):
            coordinates = list(zip(latitudes, longitudes))

            # Add wave pattern to the coordinates
            wave_amplitude = 0.001  # Adjust the amplitude as needed
            wave_frequency = 0.5  # Adjust the frequency as needed

            # Apply wave to latitude and longitude
            wave_latitudes = [
                lat + wave_amplitude * np.sin(wave_frequency * i)
                for i, lat in enumerate(latitudes)
            ]
            wave_longitudes = [
                lon + wave_amplitude * np.cos(wave_frequency * i)
                for i, lon in enumerate(longitudes)
            ]
            
            previous_coord = None
            wave_data = []  # To store data that will be written to the new CSV

            for i, (lat, lon, wave_lat, wave_lon) in enumerate(zip(latitudes, longitudes, wave_latitudes, wave_longitudes)):
                point_data = {}
                point_data['Original Latitude'] = lat
                point_data['Original Longitude'] = lon
                point_data['Wave Latitude'] = wave_lat
                point_data['Wave Longitude'] = wave_lon
                point_data['Speed'] = speeds[i]
                point_data['Altitude'] = altitudes[i]

                if previous_coord:
                    # Calculate distance and bearing between two points
                    current_coord = (wave_lat, wave_lon)
                    distance = geopy.distance.geodesic(previous_coord, current_coord).meters  # in meters
                    point_data['Distance from Previous'] = distance

                    # Calculate the bearing
                    bearing = calculate_initial_compass_bearing(previous_coord, current_coord)
                    point_data['Bearing'] = f"{bearing:.3f}"  # Format bearing to 3 decimal places
                else:
                    point_data['Distance from Previous'] = 0.0  # No previous point for the first entry
                    point_data['Bearing'] = "N/A"  # No bearing for the first entry

                # Append the point's data to the list
                wave_data.append(point_data)

                # Update previous coordinate for the next loop
                previous_coord = (wave_lat, wave_lon)

            # Now write the wave_data to a new CSV file
            with open('wave_points_details.csv', mode='w', newline='') as file:
                fieldnames = ['Original Latitude', 'Original Longitude', 'Wave Latitude', 'Wave Longitude', 'Distance from Previous', 'Speed', 'Altitude', 'Bearing']
                writer = csv.DictWriter(file, fieldnames=fieldnames)

                writer.writeheader()  # Write the header row
                for row in wave_data:
                    writer.writerow(row)

                print("Wave points details saved successfully to wave_points_details.csv.")

            # Creating the map with wave coordinates
            wave_coordinates = list(zip(wave_latitudes, wave_longitudes))
            lat_center = sum(wave_latitudes) / len(wave_latitudes)
            lon_center = sum(wave_longitudes) / len(wave_longitudes)

            m = folium.Map(location=[lat_center, lon_center], zoom_start=15)

            # Add markers and lines on the map
            for i in range(0, len(wave_latitudes), 3):  # Only plot every 10th point
                coord = (latitudes[i], longitudes[i])  # Original coordinates
                wave_coord = (wave_latitudes[i], wave_longitudes[i])  # Wave-modified coordinates
                speed = speeds[i]
                altitude = altitudes[i]
                distance = wave_data[i // 10]['Distance from Previous']
                bearing = wave_data[i // 10]['Bearing']  # Get the formatted bearing

                # Create the popup content showing all relevant details
                popup_content = f"""
                    <b>Original Coordinates:</b> {coord[0]:.6f}, {coord[1]:.6f}<br>
                    <b>Wave Coordinates:</b> {wave_coord[0]:.6f}, {wave_coord[1]:.6f}<br>
                    <b>Speed:</b> {speed}<br>
                    <b>Distance from Previous:</b> {distance:.2f} meters<br>
                    <b>Altitude:</b> {altitude}<br>
                    <b>Bearing:</b> {bearing}
                """

                folium.Marker(
                    location=wave_coord,  # Use wave-modified coordinates for marker position
                    popup=folium.Popup(popup_content, max_width=300),  # Display the data in the popup
                    icon=folium.Icon(color='blue')
                ).add_to(m)

            # Mark the latest coordinate with a custom icon and popup
            latest_coord = wave_coordinates[-1]
            latest_speed = speeds[-1]
            latest_distance = wave_data[-1]['Distance from Previous']
            latest_altitude = altitudes[-1]
            latest_bearing = wave_data[-1]['Bearing']

            latest_popup_content = f"""
                <b>Coordinates:</b> {latest_coord[0]:.6f}, {latest_coord[1]:.6f}<br>
                <b>Speed:</b> {latest_speed}<br>
                <b>Distance from Previous:</b> {latest_distance:.2f} meters<br>
                <b>Altitude:</b> {latest_altitude}<br>
                <b>Bearing:</b> {latest_bearing}
            """

            folium.Marker(
                location=latest_coord,
                popup=folium.Popup(latest_popup_content, max_width=300),
                icon=folium.CustomIcon(r"C:\Users\mamid\OneDrive\Desktop\Data Transfer\auv-800.jpg", icon_size=(30, 30))
            ).add_to(m)

            # Draw a line connecting all the wave-modified coordinates
            folium.PolyLine(list(zip(wave_latitudes, wave_longitudes)), color="blue", weight=2.5, opacity=1).add_to(m)

            # Add additional layers to the map
            folium.TileLayer('Stamen Terrain', attr="Stamen Terrain").add_to(m)
            folium.TileLayer('Stamen Toner', attr="Stamen Toner").add_to(m)
            folium.LayerControl().add_to(m)

            # Plot map with two paths: accurate and wave-modified
            m = self.plot_map_with_two_paths(latitudes, longitudes, wave_latitudes, wave_longitudes)

            # Convert the map to HTML and display it in the QWebEngineView
            data = io.BytesIO()
            m.save(data, close_file=False)
            self.map_view.setHtml(data.getvalue().decode())
        else:
            print("No coordinates available or mismatched latitude and longitude data.")

    def return_to_main(self):
        self.main_window.show()
        self.close()

# Ensure that you have the necessary function for calculating bearing
def calculate_initial_compass_bearing(pointA, pointB):
    # Convert latitude and longitude from degrees to radians
    lat1 = np.radians(pointA[0])
    lat2 = np.radians(pointB[0])
    diff_long = np.radians(pointB[1] - pointA[1])

    # Calculate the bearing
    x = np.sin(diff_long) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(diff_long))
    initial_bearing = np.arctan2(x, y)

    # Convert radians to degrees and normalize the bearing to 0-360 degrees
    initial_bearing = np.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing




if __name__ == "__main__":
    queue = Queue()
    app = QApplication(sys.argv)
    window = MainWindow(queue)
    window.show()
    sys.exit(app.exec_())
