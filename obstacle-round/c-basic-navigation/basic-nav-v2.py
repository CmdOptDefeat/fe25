import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import RPi.GPIO as GPIO

# Status LED
LED = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED, GPIO.OUT)

# Serial config
# usb-Raspberry_Pi_Pico_E6625887D3859130-if00 - Pranav
# usb-Raspberry_Pi_Pico_E6625887D3482132-if00 - Adbhut
ser = serial.Serial('/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6625887D3859130-if00', 115200, timeout=1)

tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)

config = picam2.create_video_configuration(main={"size": (1280, 720)})

# Declaring some global variables
yaw, target_yaw, total_error= 0, 0, 0
distance, start_dist = 0, 0
front_dist, left_dist, back_dist, right_dist = 100, 35, 100, 35
turns, turning = 0, False

#Define colour ranges
lower_red = np.array([0, 120, 88])
upper_red = np.array([10, 255, 255])
lower_green = np.array([52, 120, 78])
upper_green = np.array([70, 255, 255])
lower1_black = np.array([37, 65, 20])
upper1_black = np.array([65, 130, 60])
lower2_black = np.array([40, 130, 50])
upper2_black = np.array([49, 175, 90])
# The pink parking pieces also show up as red at home!

# These are currently seen obstacles
red_obs = []
green_obs = []

# Assigns merely colour to the row, and plan to go in that direction
all_obs = [['','',''],
           ['','',''],
           ['','',''],
           ['','','']
           ]
def led(duration=1.5):
    GPIO.output(LED, GPIO.HIGH)
    time.sleep(duration)  # Let the camera warm up
    GPIO.output(LED, GPIO.LOW)

def drive_data(motor_speed,servo_steering):
    # It sends driving commands to RP2040 and gets back sensor data
    global yaw, distance, start_dist
    global left_dist, front_dist, right_dist
    # Send command
    command = f"{motor_speed},{servo_steering}\n"
    ser.write(command.encode())

    # Wait for response from RP2040
    response = ser.readline().decode().strip()
    values = response.split(",")
    yaw = float(values[0])
    distance = -float(values[14]) / 43
    left_dist = int(values[12])
    front_dist = int(values[9])
    right_dist = int(values[10])
    print(f"Received Data - Yaw: {yaw}, Distance: {distance-start_dist} Left: {left_dist}, Front: {front_dist}, Right: {right_dist}\n")

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

    # Contours may be drawn by referring to section-a coloured-contours-roi
    # Filter and locate obstacle bounding boxes
    red_obs = get_obstacle_positions(red_contours, red_obs)
    green_obs = get_obstacle_positions(green_contours, green_obs)
  
    # Draw boxes around obstacles for visualization
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
            if h > w or (y > 140 and abs(1200-x) < 250 and h > 100):
                # TODO when driving integration done
                obs.append([(x,y,w,h), (0,0)])
    return obs

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

def decide_path(red_obs, green_obs):
    # Needs to be updated
    # If red obstacle detected as nearest, drive left of it
    # If green obstacle detected as nearest, drive right of it
    global yaw, total_error, turns
    current_obs = nearest_obstacle()
    print(f'The current obstacle to tackle is {current_obs}')
    speed = 200
    steering = 90
    path = 'Straight'
    y = current_obs[0][1]
    x = current_obs[0][0]
    colour = current_obs[1]
    if colour == 'green' and y > 40 and x < 1180: 
        path = 'LEFT'
        steering -= (1200-x) * 0.075
    elif colour == 'red' and y > 40 and x > 100: 
        path = 'RIGHT'
        steering += x*0.055
    else:
        # Go straight for a bi after obstacle goes out of view
        target_yaw = (turns * 90) % 360
        error = target_yaw - yaw
        correction = 0
        if error > 180: error = error - 360
        elif error < -180: error = error + 360
        total_error += error
        if error > 0: correction = error * 2.3 - total_error * 0.001    #correction to the right
        elif error < 0: correction = error * 2.2 - total_error * 0.001  #correction to the left
        steering = 90 + correction
        print("PI Straight")
    return path, speed, steering


def run():
    global frame, hsv_frame, corrected_frame, hsv_roi
    global yaw, distance, left_dist, front_dist, right_dist, turning, turns, start_dist
    while True:
        # Read a frame from the camera
        frame = picam2.capture_array()
        corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        hsv_roi = hsv_frame[350:720, 0:1280]        # Our region of interest is only the bottom half of the camera feed

        # Process camera frame for obstacles
        frame_processed, red_obs, green_obs = process_frame()

        # Decide navigation based on obstacle detection
        path_action, speed, steering = decide_path(red_obs, green_obs)
        if front_dist < 100 and (distance - start_dist) > 120: 
            drive_data(0,150)
            break
        drive_data(speed, steering)
        print(f"Steering: {steering}")
        # Show output
        cv2.putText(frame_processed, f"Speed {speed} Steering {steering}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2)

        cv2.imshow("Obstacle Detection", frame_processed)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

try:
    picam2.configure(config)
    picam2.start_preview()
    led()
    picam2.start()
    drive_data(0,90)
    start_dist = distance
    run()

finally:
    drive_data(0,90)    # Stop robot
    picam2.stop_preview()       # Close camera
    picam2.stop()
    cv2.destroyAllWindows()     # Close camera feed
    led(1.5)
    GPIO.cleanup()