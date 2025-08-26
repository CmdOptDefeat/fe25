import cv2
import numpy as np
from picamera2 import Picamera2
import time

tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)
config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()

# Global variables to store mouse coordinates and HSV value
mouse_x, mouse_y = -1, -1
hsv_value = None

def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y, hsv_value
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y
        if param is not None:  # Ensure a frame is passed
            # Get BGR value at mouse position
            bgr = param[y, x]
            # Convert BGR to HSV
            hsv_value = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]

cv2.namedWindow('Frame')

while True:
    frame = picam2.capture_array()
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Different standards are used    
    # Set the mouse callback with the current frame as parameter
    cv2.setMouseCallback('Frame', mouse_callback, param=corrected_frame)

    cv2.imshow('Frame', corrected_frame)

    if hsv_value is not None:
        print(f"HSV at ({mouse_x}, {mouse_y}): {hsv_value}")
        hsv_value = None # Reset to avoid repeated printing for the same position

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
