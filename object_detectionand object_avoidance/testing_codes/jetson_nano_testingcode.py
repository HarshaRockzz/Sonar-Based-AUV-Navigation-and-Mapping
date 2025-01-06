import random
import cv2
import numpy as np
import csv
from ultralytics import YOLO

# Opening the file in read mode
my_file = open(r"utils\cococopy.txt", "r")
# Reading the file
data = my_file.read()
# Replacing end splitting the text | when newline ('\n') is seen.
class_list = data.split("\n")
my_file.close()

# Generate random colors for the class list
detection_colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in class_list]

# Load a pretrained YOLOv8n model
model = YOLO(r"C:\Users\malla\Downloads\best_final.pt")

# Dimensions to resize video frames | decrease frame size
frame_wid = 640
frame_hyt = 480

# White box dimensions
box_width = 800  # Increased width
box_height = frame_hyt

# Object dimensions (in meters)
object_width = 0.2  # Example: Assuming a width of 0.2 meters

# Focal length of the camera (example value, you should replace it with the actual focal length)
focal_length = 1280

# Open the webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Initialize the information box
info_box = np.ones((box_height, box_width, 3), dtype=np.uint8) * 255

with open('object_data_final.csv', mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Class', 'Confidence', 'Distance', 'Width', 'Height', 'Bearing Angle'])

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Resize the frame | decrease frame size
        frame = cv2.resize(frame, (frame_wid, frame_hyt))

        # Predict on image
        detect_params = model.predict(source=[frame], conf=0.45, save=False)

        # Convert tensor array to numpy
        DP = detect_params[0].numpy()

        # Reset the information box
        info_box[:] = 255

        if len(DP) != 0:
            for i in range(len(detect_params[0])):
                boxes = detect_params[0].boxes
                box = boxes[i]  # returns one box
                clsID = box.cls.numpy()[0]
                conf = box.conf.numpy()[0]
                bb = box.xyxy.numpy()[0]

                cv2.rectangle(
                    frame,
                    (int(bb[0]), int(bb[1])),
                    (int(bb[2]), int(bb[3])),
                    detection_colors[int(clsID)],
                    3,
                )

                # Display class name and confidence on the frame
                font = cv2.FONT_HERSHEY_COMPLEX
                cv2.putText(
                    frame,
                    f"Class: {class_list[int(clsID)]}",
                    (int(bb[0]), int(bb[1]) - 10),
                    font,
                    0.5,
                    (255, 255, 255),
                    1,
                )
                cv2.putText(
                    frame,
                    f"Confidence: {round(conf * 100, 2)}%",
                    (int(bb[0]), int(bb[1]) + 10),
                    font,
                    0.5,
                    (255, 255, 255),
                    1,
                )

                # Estimate distance
                distance = (object_width * focal_length) / (bb[2] - bb[0])

                # Calculate object dimensions in real-world space
                object_height = (object_width * (bb[3] - bb[1])) / (bb[2] - bb[0])

                # Calculate bearing angle
                frame_center_x = frame_wid // 2
                object_center_x = int((bb[2] + bb[0]) / 2)
                bearing_angle = np.degrees(np.arctan2(object_center_x - frame_center_x, focal_length))

                # Make sure the bearing angle is positive
                bearing_angle = (bearing_angle + 360) % 360
                # Write data to CSV
                csv_writer.writerow([class_list[int(clsID)], round(conf * 100, 2), round(distance, 2),
                    round(bb[2] - bb[0], 2), round(bb[3] - bb[1], 2), round(bearing_angle, 2)])

                # Display information in the information box
                info_text = f"Class: {class_list[int(clsID)]}\n"
                info_text += f"Confidence: {round(conf * 100, 2)}%\n"
                info_text += f"Distance: {round(distance, 2)} meters\n"
                info_text += f"Dimensions: {round(object_width, 2)} x {round(object_height, 2)} meters\n"
                info_text += f"Bearing Angle: {round(bearing_angle, 2)} degrees\n"

                # Display information text in the information box
                cv2.putText(
                    info_box,
                    info_text,
                    (10, 50 + i * 100),  # Adjust the vertical spacing
                    font,
                    0.5,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )

        # Concatenate the frame and information box horizontally
        combined_frame = np.hstack((frame, info_box))

        # Display the resulting frame
        cv2.imshow("ObjectDetection", combined_frame)

        # Terminate run when "Q" pressed
        if cv2.waitKey(1) == ord("q"):
            break


csv_file.close()
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()