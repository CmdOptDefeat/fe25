import cv2
import numpy as np
from picamera2 import Picamera2
import time

# For standard camera use "imx219.json"
tuning = Picamera2.load_tuning_file("imx219_noir.json")
picam2 = Picamera2(tuning = tuning)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()

def canny_edge_detection(frame):
    # Convert the frame to grayscale for edge detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and smoothen edges
    blurred = cv2.GaussianBlur(src=gray, ksize=(3, 5), sigmaX=0.5)

    # Perform Canny edge detection
    edges = cv2.Canny(blurred, 70, 135)

    return blurred, edges

while True:
    # Read each frame from the webcam
    frame = picam2.capture_array()

    # Convert the frame to hsv color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    '''
    Hue (H): 0 - 179 (Represents the color type, dominant wavelength)
    Saturation (S): 0 - 255 (Represents the intensity or purity of the color) 
    Value (V): 0 - 255 (Represents the brightness or darkness of the color)
    '''

    # Define the range for colour color in hsv space
    lower_red = np.array([3, 80, 88])
    upper_red = np.array([10, 90, 98])
    lower_green = np.array([107, 74, 79])
    upper_green = np.array([117, 84, 89])
    # Create a mask to detect colour
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    mask = mask_red + mask_green

    # Apply the mask on the original image
    masked_image = cv2.bitwise_and(frame, frame, mask=mask)

    blurred, edges = canny_edge_detection(masked_image)
    # Display the original frame and the result
    cv2.imshow("Original", masked_image)
    cv2.imshow("Colour Object Detection", edges)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
picam2.stop()
cv2.destroyAllWindows()
