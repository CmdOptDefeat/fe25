import cv2
from picamera2 import Picamera2
import time

# Define the output file name and video properties
output_filename = 'pi_camera_video.mp4'
# The native Picamera resolution is 4608x2592, but for recording, a standard size is better
width, height = 1920, 1080 
fps = 30 # Frames per second

# Initialize the Picamera2 object
picam2 = Picamera2()

# Configure the camera for video capture
video_config = picam2.create_video_configuration(main={"format": 'RGB888', "size": (width, height)})
picam2.configure(video_config)

# Get the native camera sensor format to ensure compatibility with OpenCV
# You need to manually specify RGB888 format so OpenCV reads it correctly
# If omitted, OpenCV may assume BGR and produce odd colors

# Define the codec and create a VideoWriter object for saving the video
# 'mp4v' or 'avc1' are common choices for .mp4 containers
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_filename, fourcc, fps, (width, height))

# Start the camera and give it time to warm up
picam2.start()
time.sleep(1) 

print("Recording started. Press 'q' to stop.")

while True:
    # Capture a frame from the camera as a NumPy array
    frame = picam2.capture_array()

    # Write the frame to the output video file
    out.write(frame)

    # Display the frame in a window for a live preview
    cv2.imshow('Recording', frame)

    # Break the loop if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and writer objects
picam2.stop()
out.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
print(f"Recording finished and saved as '{output_filename}'")