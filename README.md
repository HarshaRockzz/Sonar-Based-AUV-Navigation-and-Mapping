# Sonar-Based AUV Navigation and Mapping

This project focuses on developing an Autonomous Underwater Vehicle (AUV) capable of navigating and mapping underwater environments using sonar technology. The AUV utilizes sonar sensors to detect obstacles, map the surroundings, and autonomously plan its path through the environment.

🌊 🤖 🗺️

## Features
- **Sonar-Based Mapping:** Real-time generation of underwater maps using sonar data.
- **Autonomous Navigation:** Path planning and obstacle avoidance to ensure efficient and safe navigation.
- **Real-Time Data Processing:** Onboard data processing for immediate analysis and decision-making.
- **Visualization Tools:** Graphical representation of the AUV’s path and the mapped environment.

## Technologies Used
- **Hardware:** [Specify the microcontroller, sensors, actuators, etc., used]
- **Software:** Python, ROS (Robot Operating System), OpenCV, [other relevant software]
- **Communication:** [Specify communication protocols like UART, I2C, etc.]
- **Mapping Algorithms:** SLAM (Simultaneous Localization and Mapping), A* or D* for path planning, etc.

## System Architecture
[Include a brief description of the system architecture, including the interaction between different components, data flow, and key modules.]

## Setup Instructions
### Prerequisites
- Python 3.x
- ROS Noetic/ROS Melodic
- [List any other software, libraries, or tools required]

### Installation
```bash
git clone https://github.com/HarshaRockzz/Sonar-Based-AUV-Navigation-and-Mapping.git
cd sonar-auv-navigation
pip install -r requirements.txt
```

### Setup ROS Workspace
```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### Running the Project
- **Launch the AUV Node:**
```bash
roslaunch auv_navigation auv_navigation.launch
```
- **Start Mapping and Navigation:**
```bash
rosrun auv_navigation mapping.py
rosrun auv_navigation navigation.py
```

### Visualization
- Use RViz to visualize the AUV’s navigation and mapping in real-time.
- Access the dashboard at [web_address_or_port] to monitor the system.

## Usage
- **Sonar Mapping:** The AUV will automatically start mapping the underwater environment as it navigates.
- **Manual Control:** Optionally, you can override the autonomous navigation and control the AUV manually using a joystick or keyboard inputs.
- **Data Logging:** Sonar data and navigation logs are saved in the /logs directory for post-mission analysis.


## Contact
For any questions, feel free to contact Harshavardhan at mamidipaka2003@gmail.com.

🚀 Happy Navigating! 🌊
