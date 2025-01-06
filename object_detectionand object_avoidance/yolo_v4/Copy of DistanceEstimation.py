import cv2 as cv
import numpy as np
import os
import math

# Distance constants
KNOWN_DISTANCE = 45  # INCHES
PERSON_WIDTH = 16  # INCHES
MOBILE_WIDTH = 3.0  # INCHES

# Object detector constants
CONFIDENCE_THRESHOLD = 0.4
NMS_THRESHOLD = 0.3

# Colors for object detection
COLORS = [(255, 0, 0), (255, 0, 255), (0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
# Defining fonts
FONTS = cv.FONT_HERSHEY_COMPLEX

# Initialize class_names as a global variable
class_names = []

# Getting class names from classes.txt file
classes_file_path = "Yolov4-Detector-and-Distance-Estimator-master/classes.txt"

if not os.path.isfile(classes_file_path):
    print(f"Error: '{classes_file_path}' not found. Make sure the file exists.")
    exit()

with open(classes_file_path, "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]

# Setting up OpenCV net
yoloNet = cv.dnn.readNet('Yolov4-Detector-and-Distance-Estimator-master/yolov4-tiny.weights', 'Yolov4-Detector-and-Distance-Estimator-master/yolov4-tiny.cfg')

yoloNet.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
yoloNet.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)

model = cv.dnn_DetectionModel(yoloNet)
model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

# Object detector function/method
def object_detector(image):
    global class_names  # Access the global variable

    # Check if the image is valid
    if image is None or image.size == 0:
        print("Error: Invalid image.")
        return []

    # Resize the image if it has valid dimensions
    resized_image = cv.resize(image, (416, 416))

    # Detect objects using YOLO model
    classes, scores, boxes = model.detect(resized_image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)

    # Creating an empty list to add object data
    data_list = []
    for (classid, score, box) in zip(classes, scores, boxes):
        # Define color of each object based on its class id 
        color = COLORS[int(classid) % len(COLORS)]

        label = "%s : %f" % (class_names[classid], score)

        # Draw rectangle on and label on object
        cv.rectangle(image, box, color, 2)
        cv.putText(image, label, (box[0], box[1]-14), FONTS, 0.5, color, 2)

        # Getting the data 
        # 1: class name, 2: object width in pixels, 3: position where to draw text (distance)
        if classid == 0:  # Person class id 
            data_list.append([class_names[classid], box[2], (box[0], box[1]-2)])

            # Add dimensions
            object_width_in_inches = KNOWN_DISTANCE * PERSON_WIDTH / box[2]
            dimensions_text = f"Dimensions: {round(object_width_in_inches, 2)} inches"
           # cv.putText(image, dimensions_text, (box[0], box[1] - 30), FONTS, 0.5, color, 2)

        elif classid == 67:  # Mobile class id
            data_list.append([class_names[classid], box[2], (box[0], box[1]-2)])

            # Add dimensions
            object_width_in_inches = KNOWN_DISTANCE * MOBILE_WIDTH / box[2]
            dimensions_text = f"Dimensions: {round(object_width_in_inches, 2)} inches"
           # cv.putText(image, dimensions_text, (box[0], box[1] - 30), FONTS, 0.5, color, 2)

        # If you want to include more classes, add more [elif] statements here

        # Add bearing angle
        bearing_angle = math.atan2((box[0] + box[2] / 2) - (resized_image.shape[1] / 2), KNOWN_DISTANCE) * (180 / math.pi)
        bearing_text = f"Bearing Angle: {round(bearing_angle, 2)} degrees"
        #cv.putText(image, bearing_text, (box[0], box[1] - 10), FONTS, 0.5, color, 2)

    # Returning list containing the object data 
    return data_list

# Focal length finder function 
def focal_length_finder(measured_distance, real_width, width_in_rf):
    focal_length = (width_in_rf * measured_distance) / real_width
    return focal_length

# Distance finder function 
def distance_finder(focal_length, real_object_width, width_in_frame):
    distance = (real_object_width * focal_length) / width_in_frame
    return distance

# Reading the reference image from dir 
ref_person = cv.imread('Yolov4-Detector-and-Distance-Estimator-master/ReferenceImages/image14.png')
ref_mobile = cv.imread('Yolov4-Detector-and-Distance-Estimator-master/ReferenceImages/image4.png')

# Check if reference images are loaded successfully
if ref_person is None or ref_mobile is None:
    print("Error: Unable to load reference images.")
    exit()

mobile_data = object_detector(ref_mobile)
mobile_width_in_rf = mobile_data[0][1]

person_data = object_detector(ref_person)
person_width_in_rf = person_data[0][1]

print(f"Person width in pixels: {person_width_in_rf} Mobile width in pixel: {mobile_width_in_rf}")

# Finding focal length 
focal_person = focal_length_finder(KNOWN_DISTANCE, PERSON_WIDTH, person_width_in_rf)
focal_mobile = focal_length_finder(KNOWN_DISTANCE, MOBILE_WIDTH, mobile_width_in_rf)


# Video capture loop
# ... (previous code remains the same)

# Video capture loop
cap = cv.VideoCapture(0)
while True:
    ret, frame = cap.read()

    # Check if the frame is captured successfully
    if not ret or frame is None or frame.size == 0:
        print("Error: Unable to capture frame.")
        continue

    data = object_detector(frame)
    num_objects = len(data)

    # Display number of objects detected
    num_objects_text = f"Objects Detected: {num_objects}"
    cv.putText(frame, num_objects_text, (frame.shape[1] - 250, 30), FONTS, 0.5, GREEN, 2)

    # Create a white rectangle on the right side
    cv.rectangle(frame, (frame.shape[1] - 240, 60), (frame.shape[1], frame.shape[0]), (255, 255, 255), -1)

    # Set a constant gap between object details
    gap_between_objects = 20

    # Display details for each detected object inside the white box
    for idx, d in enumerate(data):
        distance = distance_finder(focal_person if d[0] == 'person' else focal_mobile,
                                   PERSON_WIDTH if d[0] == 'person' else MOBILE_WIDTH, d[1])

        # Extract coordinates based on the number returned by the object detector
        if len(d[2]) == 2:
            x, y = map(int, d[2])  # Round to integers
            w, h = 0, 0  # Set width and height to 0
        elif len(d[2]) == 4:
            x, y, w, h = map(int, d[2])  # Round to integers

        # Draw rectangle and label on the object
        cv.rectangle(frame, (x, y), (x + w, y + h), BLACK, -1)

        # Display object detection inside the white box
        object_detection_text = f"Object Detection: {d[0]}"
        cv.putText(frame, object_detection_text, (frame.shape[1] - 230, 80 + idx * (100 + gap_between_objects)), FONTS, 0.5, GREEN, 2)

        # Display dimensions inside the white box
        dimensions_text = f"Dimensions: {round(PERSON_WIDTH, 2)} x {round(MOBILE_WIDTH, 2)} inches"
        cv.putText(frame, dimensions_text, (frame.shape[1] - 230, 110 + idx * (100 + gap_between_objects)), FONTS, 0.5, GREEN, 2)

        # Display bearing angle inside the white box (adjusted for positivity)
        bearing_angle = round(np.arctan2(x - frame.shape[1] // 2, KNOWN_DISTANCE) * (180 / np.pi), 2)
        if bearing_angle < 0:
            bearing_angle += 360  # Ensure positive angle
        bearing_angle_text = f"Bearing Angle: {bearing_angle} degrees"
        cv.putText(frame, bearing_angle_text, (frame.shape[1] - 230, 140 + idx * (100 + gap_between_objects)), FONTS, 0.5, GREEN, 2)

        # Display distance inside the white box
        distance_text = f"Distance: {round(distance, 2)} inch"
        cv.putText(frame, distance_text, (frame.shape[1] - 230, 170 + idx * (100 + gap_between_objects)), FONTS, 0.5, GREEN, 2)

    cv.imshow('frame', frame)

    key = cv.waitKey(1)
    if key == ord('q'):
        break

cv.destroyAllWindows()
cap.release()