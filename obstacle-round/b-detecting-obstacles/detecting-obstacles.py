import cv2
import numpy as np
from picamera2 import Picamera2
import time

# For standard camera use "imx219.json"
tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)

config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()

def init():
    global lower_red, upper_red, lower_green, upper_green, lower1_black, upper1_black, lower2_black, upper2_black
    global red_obs, green_obs
    
    #Define colour ranges
    lower_red = np.array([0, 120, 88])
    upper_red = np.array([10, 255, 255])
    lower_green = np.array([52, 120, 78])
    upper_green = np.array([70, 255, 255])
    lower1_black = np.array([37, 65, 20])
    upper1_black = np.array([65, 130, 60])
    lower2_black = np.array([40, 130, 50])
    upper2_black = np.array([49, 175, 90])
    # The 'magenta' parking pieces also show up as red!

    # These are currently seen obstacles
    red_obs = []
    green_obs = []

def process_frame():
    global hsv_frame, red_obs, green_obs, hsv_roi, corrected_frame
    # Create a mask to detect colour
    mask_red = cv2.inRange(hsv_roi, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_roi, lower1_black, upper1_black) + cv2.inRange(hsv_roi, lower2_black, upper2_black)
    
    # Find contours for red,green and black. We don't want the hierarchy
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    black_contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Contours may be drawn
    #for contour in red_contours: cv2.drawContours(corrected_frame, [contour + (0,350)], -1, (100,100,255), -1)
    #for contour in green_contours: cv2.drawContours(corrected_frame, [contour + (0,350)], -1, (100,255,100), -1)
    #for contour in black_contours: cv2.drawContours(corrected_frame, [contour + (0,350)], -1, (130,130,130), -1)

    # Filter and locate obstacle bounding boxes
    red_obs = get_obstacle_positions(red_contours, red_obs)
    green_obs = get_obstacle_positions(green_contours, green_obs)
  
    # Draw contours for visualization
    print(f"Red: {red_obs}")
    print(f"Green: {green_obs}")
    for item in red_obs:
        x = item[0][0]
        y = item[0][1] + 350    #ROI was cropped
        w = item[0][2]
        h = item[0][3]
        cv2.rectangle(corrected_frame, (x,y), (x+w,y+h), (0,255,0), 2)
    for item in green_obs:
        x = item[0][0]
        y = item[0][1] + 350    # ROI was cropped
        w = item[0][2]
        h = item[0][3]
        cv2.rectangle(corrected_frame, (x,y), (x+w,y+h), (0,255,0), 2)

    return corrected_frame, red_obs, green_obs

def get_obstacle_positions(contours, obs):
    obs = []
    min_area = 500  # minimum contour area to be obstacle in pixels
    max_area = 50000

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area and cv2.contourArea(cnt) < max_area:
            x,y,w,h = cv2.boundingRect(cnt)
            if h > w:              # Prevents parking walls being detected as obstacles in most orientations
                # TODO when driving integration done; Second tuple gives grid pos
                obs.append([(x,y,w,h), (0,0,0)])
            
    return obs


def decide_path(red_obs, green_obs):
    # Needs to be updated
    # If red obstacle detected as nearest, drive left of it
    # If green obstacle detected as nearest, drive right of it

    current_obs = nearest_obstacle()
    print(f'The current obstacle to tackle is {current_obs}')
    path = 'Straight'
    if current_obs[0][1] > 40:  
        if current_obs[1] == 'green': path = 'Left'
        elif current_obs[1] == 'red': path = 'Right'
    return path

def nearest_obstacle():
    global red_obs, green_obs
    nearest_obs = [(0,0,0,0),'']
    for obs in red_obs: 
        if nearest_obs[0][1] < obs[0][1]: 
            nearest_obs = obs
            nearest_obs[1] = 'red'
    for obs in green_obs: 
        if nearest_obs[0][1] < obs[0][1]: 
            nearest_obs = obs
            nearest_obs[1] = 'green'
    return nearest_obs

def run():
    global frame, hsv_frame, corrected_frame, hsv_roi
    while True:
        # Read a frame from the camera
        frame = picam2.capture_array()
        corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        hsv_roi = hsv_frame[350:715, 0:1280]        # Our region of interest is only the bottom half of the camera feed

        # Process camera frame for obstacles
        frame_processed, red_obs, green_obs = process_frame()

        # Decide navigation based on obstacle detection
        path_action = decide_path(red_obs, green_obs)

        # Show output
        cv2.putText(frame_processed, f"Path: {path_action}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        cv2.imshow("Obstacle Detection", frame_processed)
        #cv2.imshow("Contours", corrected_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop_preview()
    picam2.stop()
    cv2.destroyAllWindows()


init()
run()