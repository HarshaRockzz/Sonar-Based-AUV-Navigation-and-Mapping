import csv
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtCore import QObject, pyqtSlot
from trajectory import GUI as TrajectoryGUI
import io
import folium
import math


class MapHandler(QObject):
    @pyqtSlot(float, float)
    def handle_map_click(self, lat, lon):
        """Handle the map click event and process the clicked coordinates."""
        window.handle_map_click(lat, lon)


class Map3DWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Map and 3D View")
        self.setGeometry(100, 100, 1000, 600)

        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        self.map_view = QWebEngineView()

        # Initialize the first point (fixed starting position)
        self.start_point = (13.5556252, 80.0259484)
        self.clicked_points = [self.start_point]

        # Create the CSV file on initialization
        self.create_csv()

        # Set up the WebChannel
        self.channel = QWebChannel()
        self.handler = MapHandler()
        self.channel.registerObject('pywebchannel', self.handler)
        self.map_view.page().setWebChannel(self.channel)

        # Update the map initially with the starting point
        self.update_map()
        
        switch_button = QPushButton("Path Wise Trajectory")
        switch_button.setFont(self.font())
        switch_button.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        switch_button.clicked.connect(self.open_trajectory_gui)
        main_layout.addWidget(switch_button)

        main_layout.addWidget(self.map_view)
        self.setCentralWidget(central_widget)
        self.showMaximized()

    def create_csv(self):
        """ Create the CSV file and add headers. """
        with open('saved_points.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Latitude', 'Longitude', 'Wave Latitude', 'Wave Longitude', 'Bearing Angle', 'Distance (km)'])

    def update_map(self):
        """Update the map with clicked points."""
        lat_center = self.clicked_points[-1][0]
        lon_center = self.clicked_points[-1][1]
        m = folium.Map(location=[lat_center, lon_center], zoom_start=15)

        # Add previously clicked points as markers with popups and lines connecting them
        if len(self.clicked_points) > 1:
            folium.PolyLine(self.clicked_points, color="red", weight=2.5).add_to(m)

        # Add the starting point with its marker
        folium.Marker(
            location=[self.start_point[0], self.start_point[1]],
            popup="Starting Point",
            icon=folium.Icon(color='red')
        ).add_to(m)

        # Add each clicked point as a marker with a popup
        for i, point in enumerate(self.clicked_points[1:], start=1):  # Skip the start point as it's already added
            wave_lat, wave_lon = self.calculate_wave_coordinates(point)
            bearing = self.calculate_bearing(self.clicked_points[i-1], point)
            distance = self.calculate_distance(self.clicked_points[i-1], point)

            popup_text = f"Coordinates: {point[0]}, {point[1]}<br>Wave Coord: {wave_lat}, {wave_lon}<br>Bearing: {bearing:.2f}°<br>Distance: {distance:.2f} km"
            folium.Marker(
                location=[point[0], point[1]],
                popup=popup_text,
                icon=folium.Icon(color='blue')
            ).add_to(m)

        # Inject custom JavaScript for handling map clicks and adding a marker
        click_js = """
            <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
            <script>
            function addClickHandlerToMap() {
                var mapElement = document.querySelector('.folium-map');
                var mapId = mapElement.id;
                var mapInstance = window[mapId];  // Folium maps are stored in window with their IDs

                new QWebChannel(qt.webChannelTransport, function(channel) {
                    window.pywebchannel = channel.objects.pywebchannel;
                });

                mapInstance.on('click', function(e) {
                    var lat = e.latlng.lat.toFixed(6);
                    var lon = e.latlng.lng.toFixed(6);

                    // Add marker to clicked location
                    var marker = L.marker([lat, lon]).addTo(mapInstance);

                    // Show popup with coordinates
                    marker.bindPopup("Coordinates: " + lat + ", " + lon).openPopup();

                    // Send the clicked coordinates back to Python (PyQt)
                    if (window.pywebchannel) {
                        window.pywebchannel.handle_map_click(parseFloat(lat), parseFloat(lon));
                    }
                });
            }

            document.addEventListener('DOMContentLoaded', function() {
                addClickHandlerToMap();
            });
            </script>
        """

        # Add the JavaScript after the map has been created
        m.get_root().html.add_child(folium.Element(click_js))

        # Save map to HTML and display in QWebEngineView
        data = io.BytesIO()
        m.save(data, close_file=False)
        self.map_view.setHtml(data.getvalue().decode())

    def handle_map_click(self, lat, lon):
        """Handle the map click event and process the clicked coordinates."""
        self.clicked_points.append((lat, lon))

        # Calculate wave coordinates, bearing angle, and distance
        if len(self.clicked_points) > 1:
            wave_lat, wave_lon = self.calculate_wave_coordinates((lat, lon))
            bearing = self.calculate_bearing(self.clicked_points[-2], (lat, lon))
            distance = self.calculate_distance(self.clicked_points[-2], (lat, lon))

            # Save to CSV
            self.save_clicked_point(lat, lon, wave_lat, wave_lon, bearing, distance)

        # Update the map with the new point
        self.update_map()

    def save_clicked_point(self, lat, lon, wave_lat, wave_lon, bearing, distance):
        """Save clicked point details to CSV."""
        with open('saved_points.csv', mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([lat, lon, wave_lat, wave_lon, bearing, distance])

    def calculate_wave_coordinates(self, point):
        """Calculate wave coordinates based on some offset (e.g., 0.001)."""
        # Small offset for the wave coordinates (could be customized)
        offset = 0.001
        wave_lat = point[0] + offset
        wave_lon = point[1] + offset
        return wave_lat, wave_lon

    def calculate_bearing(self, pointA, pointB):
        """Calculate the bearing between two points."""
        lat1 = math.radians(pointA[0])
        lon1 = math.radians(pointA[1])
        lat2 = math.radians(pointB[0])
        lon2 = math.radians(pointB[1])

        d_lon = lon2 - lon1
        x = math.sin(d_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(d_lon))

        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing

    def calculate_distance(self, pointA, pointB):
        """Calculate the Haversine distance between two points."""
        R = 6371  # Radius of the Earth in kilometers
        lat1 = math.radians(pointA[0])
        lon1 = math.radians(pointA[1])
        lat2 = math.radians(pointB[0])
        lon2 = math.radians(pointB[1])

        d_lat = lat2 - lat1
        d_lon = lon2 - lon1

        a = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance
    
    def open_trajectory_gui(self):
        """Open the trajectory GUI from trajectory.py."""
        self.trajectory_window = TrajectoryGUI()
        self.trajectory_window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Map3DWindow()
    window.show()
    sys.exit(app.exec_())
