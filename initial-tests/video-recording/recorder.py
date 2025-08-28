import cv2
from picamera2 import Picamera2
import os

# Setup
save_dir = "initial-tests/video-recording/videos"
os.makedirs(save_dir, exist_ok=True)
output_path = os.path.join(save_dir, "video_output.mp4")

# Initialize Pi Camera 3 Wide
picam3 = Picamera2()
video_config = picam3.create_video_configuration(main={"size": (1280, 720)})  # Adjust resolution if needed
picam3.configure(video_config)
picam3.start()

# VideoWriter setup
fps = 25 
frame_size = (1280, 720)
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Format
out = cv2.VideoWriter(output_path, fourcc, fps, frame_size)

print("Recording started. Press Ctrl+C to stop.")

try:
    while True:
        frame = picam3.capture_array()
        out.write(frame)
        cv2.imshow('Recording', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Recording stopped.")
finally:
    out.release()
    picam3.stop()
    cv2.destroyAllWindows()
    print("Exited")
