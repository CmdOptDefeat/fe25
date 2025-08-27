'''
This does not work, 
Refer to basic-nav-v2.py
'''

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import math
import RPi.GPIO as GPIO

# Status LED
led = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(led, GPIO.OUT)

#Camera config
tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)
config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start_preview()
GPIO.output(led, GPIO.HIGH)
time.sleep(1.5)  # Let the camera warm up
GPIO.output(led, GPIO.LOW)
picam2.start()

# Serial config
# usb-Raspberry_Pi_Pico_E6625887D3859130-if00 - Pranav
# usb-Raspberry_Pi_Pico_E6625887D3482132-if00 - Adbhut
ser = serial.Serial('/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6625887D3859130-if00', 115200, timeout=1)


# Some starting values
yaw, target_yaw, total_error, distance = 0, 0, 0, 0
left_dist, front_dist, right_dist = 35, 100, 35
turns, turning = 0, False
prev_dist, min_dist, prev_str = 0, 0, 90

#Define colour ranges
lower_red = np.array([0, 120, 88])
upper_red = np.array([19, 255, 255])
lower_green = np.array([52, 120, 78])
upper_green = np.array([70, 255, 255])
lower1_black = np.array([37, 65, 20])
upper1_black = np.array([65, 130, 60])
lower2_black = np.array([40, 130, 50])
upper2_black = np.array([49, 175, 90])
# The 'magenta' parking pieces also show up as red!

# Related to obstacles
red_obs = [[[0,0,0,0],[0,0,0]]]
green_obs = [[[0,0,0,0],[0,0,0]]]
all_prev_obs = []
all_obs = []
prev_obs = [[0,0,0,0],'']
corner_obs =  ''
obs_passed_dist = 0
dist_from_passed_obs = 0

def process_frame():
    global hsv_frame, hsv_roi, corrected_frame
    global all_obs, all_prev_obs, red_obs, green_obs

    # Create a mask to detect colours
    mask_red = cv2.inRange(hsv_roi, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
    #mask_black = cv2.inRange(hsv_roi, lower1_black, upper1_black) + cv2.inRange(hsv_roi, lower2_black, upper2_black)
    
    # Find contours for red and green. We don't want the hierarchy
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #black_contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter and locate obstacle bounding boxes
    red_obs = get_obstacle_positions(red_contours, red_obs)
    green_obs = get_obstacle_positions(green_contours, green_obs)

    all_prev_obs = all_obs
    all_obs = []
    for obs in red_obs, green_obs:
        all_obs.append(obs)
    all_obs.sort(key=lambda obs: obs[0][1])

    if len(all_prev_obs)-len(all_obs)>0:
        for obs in all_prev_obs:
            if abs(600-obs[0][0]) > 500: 
                corner_obs = obs 
                print(f"{corner_obs} corner obs passed out of view")

    # Draw contours for visualization
    #print(f"Red Obs: {red_obs}")
    #print(f"Green Obs: {green_obs}")

    # Draw rectangles around object for checking
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
    min_area = 500      # minimum contour area to be obstacle in pixels
    max_area = 30000    # largest obstacle size possible

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area and cv2.contourArea(cnt) < max_area:
            x,y,w,h = cv2.boundingRect(cnt)
            if h > w:
                # TODO when driving integration done; Second tuple gives grid pos
                obs.append([[x,y,w,h], [0,0,0]]) 
    if obs == []: obs = [[[0,0,0,0],[0,0,0]]]           
    return obs

def nearest_obstacle():
    global red_obs, green_obs

    nearest_obs = [[0,0,0,0],'']
    for obs in red_obs: 
        if nearest_obs[0][1] < obs[0][1]: 
            nearest_obs = obs
            nearest_obs[1] = 'red'
    for obs in green_obs: 
        if nearest_obs[0][1] < obs[0][1]: 
            nearest_obs = obs
            nearest_obs[1] = 'green'
    print(f'Nearest obstacle is {nearest_obs[1]} at {nearest_obs[0]}')
    return nearest_obs

def decide_path():
    global min_dist, prev_dist, prev_str, prev_obs, turning
    global target_yaw, turns, total_error
    global dist_from_passed_obs, obs_passed_dist
    # red obstacle --> right; green obstacle --> left
    speed = 130                                  # Low speed for testing
    steering = prev_str
    path = 'Straight'
    current_obs = nearest_obstacle()
    x, y, w, h = current_obs[0][0], current_obs[0][1], current_obs[0][2], current_obs[0][3]
    obs_colour = current_obs[1]
    # All these values are currently estimates, need to be updated later with data
    image_centre = 600      # Obstacles have some size, so 640 is reached easily
    min_offset, max_offset = 60, 600        # Offset x-coordinate of the obstacle from centre to pass it
    dist_from_passed_obs = distance
    if current_obs[1]=='' or (current_obs[0][1] < 25 and (620-current_obs[0][0]) > 400 and front_dist > 110):
        path = 'Straight'
        target_yaw = (turns * 90) % 360
        error = target_yaw - yaw
        if error > 180: error = error - 360
        elif error < -180: error = error + 360
        total_error += error
        if error > 0: correction = error * 2.3 - total_error * 0.001    #correction to the right
        elif error < 0: correction = error * 2.2 - total_error * 0.001  #correction to the left
        steering = 90 + correction
        return path, speed, steering
    if not turning:
        if prev_obs[1] == current_obs[1] and current_obs[0][1]-prev_obs[0][1] <= 4:
            if dist_from_passed_obs - obs_passed_dist > 5:
                norm_y = current_obs[0][1] / (720 - 350)
                proximity = 1 - math.exp(-1 * norm_y)           # Exponential antilog-style non-linear scaling of y wrt distance
                offset_x = min_offset + (max_offset - min_offset) * proximity # Calculate the ideal position of the obstacle for this case
                if obs_colour == 'green': target_x = abs(image_centre + offset_x)        # 
                elif obs_colour == 'red': target_x = -abs(image_centre - offset_x)
                target_x = max(0, min(1200, target_x)) # Set limits on the target
                delta = target_x - x
                if delta >= 0: k = 60/600
                elif delta < 0: k = 90/600
                steering = 90 + k * delta
            else: steering = 90
        elif prev_obs[1] == current_obs[1] and current_obs[0][1]-prev_obs[0][1]>4 and front_dist > 150: 
            steering = 90
            obs_passed_dist = distance
        elif prev_obs[1] != current_obs[1]:
            obs_passed_dist = distance    
    elif turning:
        pass        # We'll look into this later
    
    # Weird steering - 0°->150° (About 65° each way)!!!
    steering = max(min(steering,150),1)         # Set limits for steering value
    prev_obs = current_obs
    prev_str = steering

    return path, speed, int(steering)


def drive_data(motor_speed,servo_steering):
    # It sends driving commands to RP2040 and gets back sensor data
    global yaw, distance, left_dist, front_dist, right_dist
    # Send command
    command = f"{motor_speed},{servo_steering}\n"
    ser.write(command.encode())

    # Wait for response from RP2040
    response = ser.readline().decode().strip()
    values = response.split(",")[0:]
    yaw = float(values[0])
    distance = -float(values[14]) / 43
    left_dist = int(values[12])
    front_dist = int(values[9])
    right_dist = int(values[10])
    print(f"Received Data - Yaw: {yaw}, Distance: {distance} Left: {left_dist}, Front: {front_dist}, Right: {right_dist}\n")


try:
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
        path, speed, steering = decide_path()

        # Send the command to the peripherals board and get back data from sensors
        print(f'Going {path} - Steering: {steering}, Speed: {speed}')
        drive_data(speed,steering)

        # Show data on camera feed
        cv2.putText(frame_processed, f"Path:{path} Steering:{steering}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("Obstacle Detection", frame_processed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    drive_data(0,90)
    picam2.stop_preview()
    picam2.stop()
    cv2.destroyAllWindows()
    GPIO.output(led, GPIO.HIGH)
    time.sleep(1.5)  # Indicate stopping
    GPIO.output(led, GPIO.LOW)
    GPIO.cleanup()