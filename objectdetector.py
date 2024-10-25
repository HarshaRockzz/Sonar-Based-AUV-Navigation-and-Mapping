import numpy as np
import cv2
from scipy.spatial import distance as dist
import math
import time


Known_distance = 30  # Inches
Known_width = 5.7  # Inches
thres = 0.5  # Threshold to detect object
nms_threshold = 0.2  # (0.1 to 1) 1 means no suppress, 0.1 means high suppress

# Colors in BGR Format (BLUE, GREEN, RED)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
BLACK = (0, 0, 0)
YELLOW = (0, 255, 255)
WHITE = (255, 255, 255)
CYAN = (255, 255, 0)
MAGENTA = (255, 0, 242)
GOLDEN = (32, 218, 165)
LIGHT_BLUE = (255, 9, 2)
PURPLE = (128, 0, 128)
CHOCOLATE = (30, 105, 210)
PINK = (147, 20, 255)
ORANGE = (0, 69, 255)
# ... (other colors)

font = cv2.FONT_HERSHEY_PLAIN
thres = 0.3 
cap = cv2.VideoCapture(0)
cap.set(3, 1280)  # Width
cap.set(4, 720)   # Height

classNames = []
with open('coco.names', 'r') as f:
    classNames = f.read().splitlines()

Colors = np.random.uniform(0, 255, size=(len(classNames), 3))

yolo_weights = "yolov3.weights"
yolo_config = "yolov3.cfg"
yolo_classes = "coco.names"

net = cv2.dnn.readNet(yolo_weights, yolo_config)

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output21.avi', fourcc, 30.0, (640, 480))
def calculate_speed(x1, y1, x2, y2, time_diff):
    distance = dist.euclidean((x1, y1), (x2, y2))
    speed = distance / time_diff  # Speed in pixels per second
    return speed

def get_output_layers(net):
    layer_names = net.getUnconnectedOutLayersNames()
    return [layer_names[i] for i in range(len(layer_names))]




def post_process(frame, outs, conf_threshold):
    frame_height, frame_width, _ = frame.shape
    class_ids = []
    confidences = []
    boxes = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > conf_threshold:
                center_x = int(detection[0] * frame_width)
                center_y = int(detection[1] * frame_height)
                w = int(detection[2] * frame_width)
                h = int(detection[3] * frame_height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    class_ids = [class_ids[i] for i in indices]  # Fix here
    confidences = [confidences[i] for i in indices]
    boxes = [boxes[i] for i in indices]

    return class_ids, confidences, boxes


prev_data = {}
start_time = time.time()
while True:
    _, frame = cap.read()
    if frame is None:
        break
    frame_height, frame_width, _ = frame.shape

    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(get_output_layers(net))

    classIds, confs, bbox = post_process(frame, outs, thres)

    all_speeds = []

    for i in range(len(bbox)):
        box = bbox[i]
        confidence = str(round(confs[i], 2))
        color = (int(Colors[classIds[i] - 1][0]), int(Colors[classIds[i] - 1][1]), int(Colors[classIds[i] - 1][2]))
        x, y, w, h = box[0], box[1], box[2], box[3]
        prev_x, prev_y, prev_speed = prev_data.get(i, (x, y, 0))

        speed = calculate_speed(prev_x, prev_y, x, y, time_diff=time.time() - start_time)
        prev_data[i] = (x, y, speed)

        object_name = classNames[classIds[i] - 1]
        cv2.putText(frame, f"{object_name} Speed: {round(speed, 2)} px/s", (int(x) + 10, int(y + h + 50)), font, 0.7, (0, 0, 0), 2)


        all_speeds.append(f"Object {i + 1}: {round(speed, 2)} px/s")
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), color, thickness=2)

        cv2.putText(frame, f"{object_name} {confidence}", (int(x) + 10, int(y) + 20), font, 1, color, 2)


    # Display all speeds accumulated from detected objects
    for idx, speed_text in enumerate(all_speeds):
        cv2.putText(frame, f"{speed_text}", (10, 50 + idx * 20), font, 1, (0, 0, 0), 2)

    cv2.imshow('Output', frame)
    out.write(frame)
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
