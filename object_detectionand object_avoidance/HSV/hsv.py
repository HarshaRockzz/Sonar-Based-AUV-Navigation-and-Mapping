import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('detecting')
cv2.resizeWindow('detecting', 400, 200)  
cv2.moveWindow('detecting', 50, 50) 
cv2.createTrackbar('LH','detecting',0,255,nothing)
cv2.createTrackbar('LS','detecting',0,255,nothing)
cv2.createTrackbar('LV','detecting',0,255,nothing)
cv2.createTrackbar('UH','detecting',255,255,nothing)
cv2.createTrackbar('US','detecting',255,255,nothing)
cv2.createTrackbar('UV','detecting',255,255,nothing)

# Open video capture object
cap=cv2.VideoCapture(0)  # Change this to your video file path

while True:
    ret,frame=cap.read()
    if not ret:
        break
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    l_h = cv2.getTrackbarPos('LH','detecting')
    l_s = cv2.getTrackbarPos('LS','detecting')
    l_v = cv2.getTrackbarPos('LV','detecting')
    u_h = cv2.getTrackbarPos('UH','detecting')
    u_s = cv2.getTrackbarPos('US','detecting')
    u_v = cv2.getTrackbarPos('UV','detecting')
    
    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])
    
    Mask = cv2.inRange(hsv, l_b, u_b)
    res = cv2.bitwise_and(frame, frame, mask=Mask)

    # Apply Morphological Operations
    kernel = np.ones((5,5), np.uint8)
    Mask = cv2.morphologyEx(Mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(Mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    cv2.imshow('frame', frame)
    cv2.imshow('mask', Mask)
    cv2.imshow('result', res)
    
    k = cv2.waitKey(1)
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
