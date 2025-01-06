import sys
import os
import csv
import multiprocessing
from PyQt5.QtCore import Qt, QTimer,QPoint,QSize
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QFontMetrics, QPolygon,QPixmap,QTransform,QPalette
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,QPushButton, QComboBox, QGridLayout, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5 import QtCore
import folium
import time
import math
import random

class CompassWidget(QWidget):
    def __init__(self, label_text):
        super().__init__()
        self.label = QLabel(label_text)
        self.label.setStyleSheet("QLabel { font-weight: bold; color: #4CAF50; }")
        self.angle = 0.0
        self.angle_display = QLabel("0.0")
        self.angle_layout = QHBoxLayout()
        self.angle_layout.addWidget(self.angle_display)
        self.setMinimumSize(250, 250)
        self.setMaximumSize(250, 250)
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(self.label)
        main_layout.addLayout(self.angle_layout)

        self.graphics_view = QGraphicsView(self)
        self.graphics_view.setMinimumSize(250, 250)
        self.graphics_view.setMaximumSize(250, 250)
        main_layout.addWidget(self.graphics_view)

        self.scene = QGraphicsScene()
        self.graphics_view.setScene(self.scene)

        self.compass_item = QGraphicsPixmapItem()
        compass_pixmap = QPixmap("compass1.png")
        self.compass_item.setPixmap(compass_pixmap.scaled(250, 250, Qt.AspectRatioMode.KeepAspectRatio))
        self.scene.addItem(self.compass_item)

        self.arrow_item = QGraphicsPixmapItem()
        arrow_pixmap = QPixmap("arrow.png")
        self.arrow_item.setPixmap(arrow_pixmap.scaled(150,100, Qt.AspectRatioMode.KeepAspectRatio))
        self.arrow_item.setOffset(76 ,48)
        self.scene.addItem(self.arrow_item)

    def set_angle(self, angle):
        self.angle = angle
        self.angle_display.setText(str(angle))
        self.update_arrow_rotation()

    def update_arrow_rotation(self):
        transform = QTransform()
        transform.translate(125, 125)
        transform.rotate(self.angle)
        transform.translate(-125, -125)
        self.arrow_item.setTransform(transform)

    def sizeHint(self):
        return QtCore.QSize(250, 250)

class GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mission Planner GUI")
        self.setGeometry(200, 100, 1500, 800)
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        left_layout = QGridLayout()
        right_layout = QVBoxLayout()

        lat_label = QLabel("Latitude:")
        lon_label = QLabel("Longitude:")
        self.lat_edit = QLineEdit()
        self.lon_edit = QLineEdit()
        self.speed_label = QLabel("Speed (in m/s):")
        self.speed_edit = QLineEdit()
        self.distance_label = QLabel("Distance/radius/length (in mts):")
        self.distance_edit = QLineEdit()
        self.heading_label = QLabel("Heading Angle (in degrees):")
        self.heading_edit = QLineEdit()
        trajectory_label = QLabel("Choose Trajectory:")
        self.trajectory_combo = QComboBox()
        self.trajectory_combo.addItem("Straight Line")
        self.trajectory_combo.addItem("Circular Path")
        self.trajectory_combo.addItem("S Shape")
        self.trajectory_combo.currentIndexChanged.connect(self.show_details)
        generate_button = QPushButton("Generate Waypoints")
        generate_button.clicked.connect(self.generate_waypoints)
        start_button = QPushButton("Start Simulation")
        start_button.clicked.connect(self.start_simulation)
        self.map_view = QWebEngineView()

        self.speed_label_display = QLabel()
        self.lat_label_display = QLabel()
        self.lon_label_display = QLabel()
        self.heading_label_display = QLabel()
        self.compass_yaw = CompassWidget("Yaw")
        self.compass_pitch = CompassWidget("Pitch")
        self.compass_roll = CompassWidget("Roll")

        left_layout.addWidget(trajectory_label, 0, 0, 1, 2)
        left_layout.addWidget(self.trajectory_combo, 1, 0, 1, 2)
        left_layout.addWidget(lat_label, 2, 0)
        left_layout.addWidget(self.lat_edit, 3, 0)
        left_layout.addWidget(lon_label, 2, 1)
        left_layout.addWidget(self.lon_edit, 3, 1)
        left_layout.addWidget(self.speed_label, 4, 0)
        left_layout.addWidget(self.speed_edit, 5, 0)
        left_layout.addWidget(self.distance_label, 6, 0)
        left_layout.addWidget(self.distance_edit, 6, 1)
        left_layout.addWidget(self.heading_label, 4, 1)
        left_layout.addWidget(self.heading_edit, 5, 1)
        left_layout.addWidget(generate_button, 7, 0, 1, 2)
        left_layout.addWidget(self.speed_label_display, 9, 0, 1, 2)
        left_layout.addWidget(self.lat_label_display, 10, 0, 1, 2)
        left_layout.addWidget(self.lon_label_display, 11, 0, 1, 2)
        left_layout.addWidget(self.heading_label_display, 12, 0, 1, 2)
        left_layout.addWidget(self.compass_yaw, 13, 1, 1, 2)
        left_layout.addWidget(self.compass_pitch, 14, 0, 1, 2)
        left_layout.addWidget(self.compass_roll, 14,1, 1, 2)
        left_layout.addWidget(start_button, 8,0,1,2)
        right_layout.addWidget(self.map_view)
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        main_layout.setStretch(0, 1)
        main_layout.setStretch(1, 2)
        self.timer = QTimer()
        self.lat_label_display.setText("Latitude: {}")
        self.lon_label_display.setText("Longitude: {}")
        self.timer.timeout.connect(self.update_display_values)
        self.timer.start(2000)

        self.waypoints = []
        self.current_waypoint_index = 0
        self.map = None
        self.generate_map()

    def update_display_values(self):
        speed = self.speed_edit.text()
        heading = self.heading_edit.text()
        self.speed_label_display.setText("Speed: {} m/s".format(speed))
        self.heading_label_display.setText("Heading Angle: {}°".format(heading))

    def show_details(self, index):
        trajectory = self.trajectory_combo.currentText()
        if trajectory == "Straight Line":
            self.speed_edit.setEnabled(True)
            self.distance_edit.setEnabled(True)
            self.heading_edit.setEnabled(True)
            self.distance_label.setVisible(True)
            self.heading_label.setVisible(True)
            self.speed_label.setVisible(True)
            self.heading_label_display.setVisible(True)
        elif trajectory == "Circular Path":
            self.speed_edit.setEnabled(False)
            self.distance_edit.setEnabled(True)
            self.heading_edit.setEnabled(False)
            self.distance_label.setVisible(True)
            self.heading_label.setVisible(False)
            self.speed_label.setVisible(False)
            self.heading_label_display.setVisible(False)
        elif trajectory == "S Shape":
            self.speed_edit.setEnabled(True)
            self.distance_edit.setEnabled(True)
            self.heading_edit.setEnabled(True)
            self.distance_label.setVisible(True)
            self.heading_label.setVisible(True)
            self.speed_label.setVisible(True)
            self.heading_label_display.setVisible(True)

    def generate_waypoints(self):
        trajectory = self.trajectory_combo.currentText()
        if trajectory == "Straight Line":
            lat = float(self.lat_edit.text())
            lon = float(self.lon_edit.text())
            speed = float(self.speed_edit.text())
            distance = float(self.distance_edit.text())
            heading = float(self.heading_edit.text())
            num_waypoints = int(distance / speed) + (2 if (distance % speed) != 0 else 1)
            angular_distance = speed / 6371000
            heading_rad = math.radians(heading)
            current_lat = lat
            current_lon = lon
            self.waypoints = [(current_lat, current_lon)]
            
            for _ in range(1, num_waypoints):
                current_lat += math.degrees(angular_distance) * math.cos(heading_rad)
                current_lon += math.degrees(angular_distance) * math.sin(heading_rad)
                self.waypoints.append((current_lat, current_lon))

            self.save_waypoints_to_csv()
            self.update_map()

        elif trajectory == "Circular Path":
            lat = float(self.lat_edit.text())
            lon = float(self.lon_edit.text())
            distance = float(self.distance_edit.text())
            num_waypoints = 16

            center_lat = lat + math.degrees(distance / 6371000)
            center_lon = lon
            self.waypoints = []
            for i in range(num_waypoints):
                angle = math.radians(float(i * 360) / num_waypoints)
                dx = distance * math.cos(angle)
                dy = distance * math.sin(angle)
                waypoint_lat = center_lat + math.degrees(dy / 6371000)
                waypoint_lon = center_lon + math.degrees(dx / (6371000 * math.cos(math.radians(center_lat))))
                self.waypoints.append((waypoint_lat, waypoint_lon))
            self.update_map2()

        elif trajectory == "S Shape":
            lat = float(self.lat_edit.text())
            lon = float(self.lon_edit.text())
            distance_between_waypoints = float(self.distance_edit.text())
            self.waypoints = []
            current_lat = lat
            current_lon = lon
            self.waypoints.append((current_lat, current_lon))
            current_lon += math.degrees(distance_between_waypoints / 6371000)
            self.waypoints.append((current_lat, current_lon))
            current_lat += math.degrees(distance_between_waypoints / (6371000 * math.cos(math.radians(current_lat))))
            self.waypoints.append((current_lat, current_lon))
            current_lon -= math.degrees(distance_between_waypoints / 6371000)
            self.waypoints.append((current_lat, current_lon))
            current_lat += math.degrees(distance_between_waypoints / (6371000 * math.cos(math.radians(current_lat))))
            self.waypoints.append((current_lat, current_lon))
            current_lon += math.degrees(distance_between_waypoints / 6371000)
            self.waypoints.append((current_lat, current_lon))
            self.update_map3()
            
    def save_waypoints_to_csv(self):
     with open("waypoints.csv", mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["lat", "lon", "wave_lat", "wave_lon", "bearing", "distance"])

        for i, (lat, lon) in enumerate(self.waypoints):
            if i == 0:
                bearing = distance = 0  # No bearing or distance for the first point
                wave_lat, wave_lon = lat, lon  # No wave coordinates for the first point
            else:
                prev_lat, prev_lon = self.waypoints[i - 1]
                bearing = self.calculate_bearing(prev_lat, prev_lon, lat, lon)
                distance = self.calculate_distance(prev_lat, prev_lon, lat, lon)
                
                # Calculate wave coordinates (custom function for wave offsets)
                wave_lat, wave_lon = self.calculate_wave_coordinates(lat, lon, bearing, distance)

            writer.writerow([lat, lon, wave_lat, wave_lon, round(bearing, 2), round(distance, 2)])
            
    import math

    def calculate_wave_coordinates(self, lat, lon, bearing, distance, wave_factor=0.0001):
     """
     Calculate wave coordinates by introducing a small offset from the original coordinates.
    The offset is based on the bearing and distance and is adjusted by a wave factor.
    
    Parameters:
        lat (float): Original latitude.
        lon (float): Original longitude.
        bearing (float): Bearing angle in degrees.
        distance (float): Distance from previous point in meters.
        wave_factor (float): Factor to control the wave effect size. Default is 0.0001.
        
    Returns:
        wave_lat (float): Latitude of the wave offset point.
        wave_lon (float): Longitude of the wave offset point.
    """
    # Convert bearing to radians
     bearing_rad = math.radians(bearing)

    # Calculate the wave offset distance, proportional to the original distance
     wave_distance = distance * wave_factor

    # Earth radius in meters
     earth_radius = 6371000

    # Calculate offset in latitude
     wave_lat = lat + (wave_distance / earth_radius) * (180 / math.pi)

    # Calculate offset in longitude
     wave_lon = lon + (wave_distance / earth_radius) * (180 / math.pi) / math.cos(math.radians(lat))

     return wave_lat, wave_lon



    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c  # Distance in meters

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)

        x = math.sin(delta_lambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        bearing = math.degrees(math.atan2(x, y))

        return (bearing + 360) % 360  # Normalize to 0-360 degrees

    def generate_map(self):
        m = folium.Map(location=[20, 78], zoom_start=4)
        map_file = "map.html"
        m.save(map_file)
        self.map_view.load(QtCore.QUrl.fromLocalFile(os.path.abspath(map_file)))

    def update_map(self):
        self.map_view.page().runJavaScript("map.remove();")
        initial_lat = float(self.lat_edit.text())
        initial_lon = float(self.lon_edit.text())
        self.map = folium.Map(location=[initial_lat, initial_lon], zoom_start=18)
        total_waypoints = len(self.waypoints)
        for i, waypoint in enumerate(self.waypoints):
            lat, lon = waypoint
            if i == 0 or i == total_waypoints - 1:
                folium.Marker(location=[lat, lon], popup=f"Waypoint {i + 1}\nLatitude: {lat}\nLongitude: {lon}",
                              icon=folium.Icon(color="green")).add_to(self.map)
            else:
                folium.Marker(location=[lat, lon], popup=f"Waypoint {i + 1}\nLatitude: {lat}\nLongitude: {lon}").add_to(self.map)
        folium.PolyLine(locations=self.waypoints, color="red").add_to(self.map)
        
        map_file = "map.html"
        self.map.save(map_file)
        self.map_view.load(QtCore.QUrl.fromLocalFile(os.path.abspath(map_file)))


    def update_map2(self):
        self.map_view.page().runJavaScript("map.remove();")
        initial_lat = float(self.lat_edit.text())
        initial_lon = float(self.lon_edit.text())
        self.map = folium.Map(location=[initial_lat, initial_lon], zoom_start=18)
        total_waypoints = len(self.waypoints)
        initial_lat = float(self.lat_edit.text())
        initial_lon = float(self.lon_edit.text())

        for i, waypoint in enumerate(self.waypoints):
            lat, lon = waypoint
            if lat == initial_lat and lon == initial_lon:
                folium.Marker(location=[lat, lon], popup=f"Waypoint {i + 1}\nLatitude: {lat}\nLongitude: {lon}",
                              icon=folium.Icon(color="green")).add_to(self.map)
            else:
                folium.Marker(location=[lat, lon], popup=f"Waypoint {i + 1}\nLatitude: {lat}\nLongitude: {lon}").add_to(self.map)

        folium.PolyLine(locations=self.waypoints, color="red").add_to(self.map)
        map_file = "map.html"
        self.map.save(map_file)
        self.map_view.load(QtCore.QUrl.fromLocalFile(os.path.abspath(map_file)))

    def update_map3(self):
        self.map_view.page().runJavaScript("map.remove();")
        initial_lat = float(self.lat_edit.text())
        initial_lon = float(self.lon_edit.text())
        self.map = folium.Map(location=[initial_lat, initial_lon], zoom_start=18)
        total_waypoints = len(self.waypoints)
        initial_lat = float(self.lat_edit.text())
        initial_lon = float(self.lon_edit.text())

        for i, waypoint in enumerate(self.waypoints):
            lat, lon = waypoint
            if lat == initial_lat and lon == initial_lon:
                folium.Marker(location=[lat, lon], popup=f"Waypoint {i + 1}\nLatitude: {lat}\nLongitude: {lon}",
                              icon=folium.Icon(color="green")).add_to(self.map)
            else:
                angle = math.radians(float(self.heading_edit.text()))
                dx = math.cos(angle) * (lon - initial_lon) - math.sin(angle) * (lat - initial_lat)
                dy = math.sin(angle) * (lon - initial_lon) + math.cos(angle) * (lat - initial_lat)
                new_lat = initial_lat + dy
                new_lon = initial_lon + dx
                self.waypoints[i] = (new_lat, new_lon)
                folium.Marker(location=[new_lat, new_lon],
                              popup=f"Waypoint {i + 1}\nLatitude: {new_lat}\nLongitude: {new_lon}").add_to(self.map)

        folium.PolyLine(locations=self.waypoints, color="red").add_to(self.map)
        map_file = "map.html"
        self.map.save(map_file)
        self.map_view.load(QtCore.QUrl.fromLocalFile(os.path.abspath(map_file)))

    def start_simulation(self):
        self.current_waypoint_index = 0
        self.move_to_next_waypoint()
     
    def move_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            lat, lon = self.waypoints[self.current_waypoint_index]
            print(f"Moving to waypoint {self.current_waypoint_index + 1} - Latitude: {lat}, Longitude: {lon}")
            self.lat_label_display.setText(f"Current Latitude: {lat}")
            self.lon_label_display.setText(f"Current Longitude: {lon}")
            delay = 2000
            icon = folium.CustomIcon(icon_image="auv-800.jpg", icon_size=(60, 60))
            m = folium.Map(location=[lat, lon], zoom_start=18)

            for i, waypoint in enumerate(self.waypoints):
                wpt_lat, wpt_lon = waypoint
                if i == self.current_waypoint_index:
                    folium.Marker(location=[wpt_lat, wpt_lon],
                                popup=f"Current Waypoint - Latitude: {wpt_lat}, Longitude: {wpt_lon}",
                                icon=icon).add_to(m)
                else:
                    folium.Marker(location=[wpt_lat, wpt_lon],
                                popup=f"Waypoint {i + 1} - Latitude: {wpt_lat}, Longitude: {wpt_lon}",
                                icon=folium.Icon(icon="cloud")).add_to(m)

            m.save("map.html")
            self.map_view.load(QtCore.QUrl.fromLocalFile(os.path.abspath("map.html")))
            self.current_waypoint_index += 1
            QtCore.QTimer.singleShot(delay, self.move_to_next_waypoint)
            self.timer.timeout.connect(self.update_compass_values)
        else:
            self.timer.stop()
            print("Simulation finished")
        
    def update_compass_values(self):
        yaw_angle = int(random.uniform(0.0, 360.0))
        pitch_angle = int(random.uniform(-90.0, 90.0))
        roll_angle = int(random.uniform(-180.0, 180.0))
        self.compass_yaw.set_angle(yaw_angle)
        self.compass_pitch.set_angle(pitch_angle)
        self.compass_roll.set_angle(roll_angle)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet("QPushButton { background-color: #4CAF50 !important; color: black; }"
                      "QPushButton:hover { background-color: #45A049 !important; }"
                      "QLabel { color: #FF5733; }"
                      "QLineEdit { background-color: #F0F0F0; color: blue;}"
                      "QComboBox { background-color: #F0F0F0;color: blue; }"
                      "QWidget { background-color:  #D3D3D3; }"
                      "QGraphicsView { background-color: #303030; }")

    gui = GUI()
    gui.show()
    sys.exit(app.exec_())

