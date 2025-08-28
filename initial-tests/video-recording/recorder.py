import cv2
from picamera2 import Picamera2
import os

# Setup
save_dir = "videos"
os.makedirs(save_dir, exist_ok=True)
output_path = os.path.join(save_dir, "video_output.avi")

# Initialize Pi Camera 3 Wide
picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (1280, 720)})  # Adjust resolution if needed
picam2.configure(video_config)
picam2.start()

# VideoWriter setup
fps = 30  # You can change this
frame_size = (1280, 720)
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Use 'MJPG', 'X264', or 'mp4v' for other formats
out = cv2.VideoWriter(output_path, fourcc, fps, frame_size)

print("Recording started. Press Ctrl+C to stop.")

try:
    while True:
        frame = picam2.capture_array()
        out.write(frame)
        # Optional: Display frame
        cv2.imshow('Recording', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Recording stopped.")
finally:
    out.release()
    picam2.stop()
    cv2.destroyAllWindows()
