import cv2
import numpy as np
from picamera2 import Picamera2
import time

# For standard camera use "imx219.json"
tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)

config = picam2.create_video_configuration(main={"size": (1280, 720),"format": 'RGB888'})
picam2.configure(config)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()

#Define colour ranges
lower_red = np.array([0, 120, 88])
upper_red = np.array([13, 255, 255])
lower_green = np.array([52, 120, 78])
upper_green = np.array([70, 255, 255])
lower1_black = np.array([37, 65, 20])
upper1_black = np.array([65, 130, 60])
lower2_black = np.array([40, 130, 50])
upper2_black = np.array([49, 175, 90])
# The 'magenta' parking pieces also show up as red!


while True:
    # Read a frame from the camera
    frame = picam2.capture_array()
    #corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_roi = hsv_frame[350:715, 0:1280]
    # Create a mask to detect colour
    mask_red = cv2.inRange(hsv_roi, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_roi, lower1_black, upper1_black) + cv2.inRange(hsv_roi, lower2_black, upper2_black)

    # Find contours for red,green and black. We don't want the hierarchy
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    black_contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on respective masks and original image
    for contour in red_contours:
        offset_contour = contour + (0,350)
        if cv2.contourArea(contour) > 500:
            cv2.drawContours(mask_red, contour, -1, (200,200,255), -1)
            cv2.drawContours(frame, [offset_contour], -1, (100,100,255), -1)
    for contour in green_contours:
        if cv2.contourArea(contour) > 500:
            offset_contour = contour + (0,350)
            cv2.drawContours(mask_green, contour, -1, (200,255,200), -1)
            cv2.drawContours(frame, [offset_contour], -1, (100,255,100), -1)
    for contour in black_contours:
        if cv2.contourArea(contour) > 500:
            offset_contour = contour + (0,350)
            cv2.drawContours(mask_black, contour, -1, (130,130,130), -1)
            cv2.drawContours(frame, [offset_contour], -1, (255,255,255), -1)
    # Display the contour frames
    cv2.imshow('Contour Detection', frame)
    #cv2.imshow('Red Contours', mask_red)
    #cv2.imshow('Green Contours', mask_green)
    #cv2.imshow('Black Contours', mask_black)
    

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the capture and destroy all windows
picam2.stop_preview()
picam2.stop()
cv2.destroyAllWindows()