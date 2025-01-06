
# Autonomous Navigation System for Autonomous Underwater Vehicles (AUVs)

## Introduction
This project focuses on developing an Autonomous Navigation System for Autonomous Underwater Vehicles (AUVs), addressing challenges such as GPS-denied environments, object detection, avoidance, and efficient mission planning. The system is designed to improve underwater exploration, especially in hazardous or inaccessible areas, while ensuring real-time performance and reliability.

## Table of Contents
1. [Abstract](#abstract)
2. [Key Features](#key-features)
3. [Methodology](#methodology)
4. [Experimental Results](#experimental-results)
5. [Technologies and Tools Used](#technologies-and-tools-used)
6. [Datasets and Links](#datasets-and-links)
7. [Conclusion and Future Work](#conclusion-and-future-work)

---

## Abstract
Autonomous Underwater Vehicles (AUVs) play a critical role in underwater operations such as exploring shipwrecks, monitoring offshore structures, and performing construction stability analysis. This project advances existing systems by focusing on:

- Developing robust object detection and avoidance mechanisms.
- Implementing a dead reckoning algorithm for navigation in GPS-deprived areas.
- Designing an intuitive GUI for mission planning and real-time monitoring.

## Key Features

### 1. **Object Detection and Avoidance**
- Trained object detection models using **YOLOv5**, **YOLOv8**, and **Mobilenet**.
- Calculated object dimensions, distances, and bearings using **OpenCV**.
- Implemented object avoidance algorithms integrated with thruster control.

### 2. **Dead Reckoning Navigation**
- Estimated AUV coordinates using sensor data from:
  - **Inertial Navigation Systems (INS):** Yaw, Pitch, Roll.
  - **Doppler Velocity Logs (DVL):** Velocity.
- Applied corrections for bias, scale-factor errors, and random walk to minimize errors.

### 3. **Mission Planning GUI**
- Developed with **PyQt5**, enabling:
  - Real-time trajectory visualization.
  - Waypoint-based navigation.
  - Critical decision-making based on sensor monitoring.

### 4. **Integrated System**
- Communication between **Jetson Nano** (object detection) and **Raspberry Pi** (thruster control).
- Achieved real-time synchronization and performance optimization through multiprocessing.


## Methodology

### 1. **Object Detection and Avoidance**
- Collected datasets from underwater environments.
- Trained models with sonar and vision-based data.
- Implemented avoidance algorithms that leverage thruster control for safe navigation.

### 2. **Dead Reckoning**
- Used mathematical models to compute AUV’s position and velocity.
- Integrated GPS modules for calibration and comparative evaluations.

### 3. **Mission Planner GUI**
- Designed an intuitive interface for trajectory planning and navigation.
- Real-time animations and sensor integration for decision-making.

---

## Experimental Results

### Object Detection
- Enhanced accuracy with robust datasets and advanced algorithms.
- Results validated in controlled environments, such as college water tanks.

### Dead Reckoning
- Achieved accurate coordinate estimation in straight-line paths.
- Incorporated modifications for improved performance in turns.

### Mission Planner GUI
- Delivered smooth real-time performance with high refresh rates and multiprocessing support.

---

## Technologies and Tools Used
- **Languages:** Python
- **Libraries:** PyQt5, OpenCV
- **Hardware:** Jetson Nano, Raspberry Pi
- **Algorithms:** YOLO (v5, v8), Mobilenet, Dead Reckoning
- **Sensors:** INS, DVL, GPS modules
- **Frameworks:** Roboflow

---

## Datasets and Links
- **Dataset:** Underwater images for training object detection models. <b>Link:<b/> https://drive.google.com/file/d/1NCc6A4K5yWa8bp0Pi6s1XH0MXGNiQWnh/view?usp=drive_link
- **Model Training:** Conducted using **Roboflow**. <b>Link:<b/> https://roboflow.com/
- **Output Results:** Validated on **Jetson Nano**. <b>Link:<b/> https://drive.google.com/file/d/1NEHvKLt4-xgPayptMmRkTbl57szVXV5b/view
- **Project Files:** [Complete Project Drive] <b>Link:<b/> https://docs.google.com/document/d/1lOyl_Cy6FecUcqXIcjAvhJygHFv-J0v9/edit

---

## Conclusion and Future Work

### Conclusion
The project successfully demonstrates autonomous navigation for AUVs by addressing key challenges like object detection, avoidance, and navigation in GPS-denied environments.

### Future Work
- Scaling algorithms for larger and more diverse datasets.
- Enhancing navigation precision in dynamic underwater conditions.
- Deploying the system in real-world underwater missions.

---
