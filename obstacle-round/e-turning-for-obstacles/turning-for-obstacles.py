'''
This programs starts from where the basic-nav-v2 program ends, i.e start of turn, and ensure the robot completes the turn upto crossing the obstacle markers nearest to the turn
'''

import cv2      # Import all necessary libraries
import numpy as np
from picamera2 import Picamera2
import time, serial, os, logging
import RPi.GPIO as GPIO
from datetime import datetime

if True:    # System setup
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
    config = picam2.create_video_configuration(main={"size": (1280, 720), "format": 'RGB888'})

if True:    # Logging, video setup
    log_dir = 'obstacle-round/e-turning-for-obstacles/logs'
    now = datetime.now()
    log_filename = f"log_{now.strftime('%Y-%m-%d_%H-%M-%S')}.log"
    log_path = os.path.join(log_dir, log_filename)
    # Configure logging to append mode
    logging.basicConfig(
        filename=log_path,
        filemode='a',  # Append mode
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )

    video_dir = "obstacle-round/e-turning-for-obstacles/videos"
    os.makedirs(video_dir, exist_ok=True)
    output_path = os.path.join(video_dir, f"video_{now.strftime('%Y-%m-%d_%H-%M-%S')}.mp4")

    fps = 20
    frame_size = (1280, 720)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Format
    video_out = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
    print("\n\nCreated log file, initialised video\n\n")

if True:    # Variable declarations
    yaw, target_yaw, total_error= 0, 0, 0
    distance, start_dist = 0, 0
    front_dist, left_dist, back_dist, right_dist = 100, 35, 100, 35
    turn_dir, turns, prev_turns, turning, turn_forward = 1, 0, 0, False, 0  # 1:Clockwise; -1:Anticlockwise

    #Define colour ranges
    lower_red = np.array([0, 120, 88])
    upper_red = np.array([10, 255, 255])
    lower_green = np.array([62, 120, 78])
    upper_green = np.array([71, 255, 255])
    lower1_black = np.array([37, 65, 20])
    upper1_black = np.array([65, 130, 60])
    lower2_black = np.array([40, 130, 50])
    upper2_black = np.array([49, 175, 90])
    # The pink parking pieces also show up as red at home!

    start_pos = 'outer'     # inner-closer to inner wall; outer-closer to outer wall

    # These are currently seen obstacles
    red_obs = []
    green_obs = []
    prev_obs = [(0,0,0,0),'']
    prev_steering = 90
    turn_obs = [(0,0,0,0),'']

def led(duration=1.5):
    GPIO.output(LED, GPIO.HIGH)
    time.sleep(duration)  # Let the camera warm up
    GPIO.output(LED, GPIO.LOW)

def drive_data(motor_speed,servo_steering):
    # It sends driving commands to RP2040 and gets back sensor data
    global yaw, distance, start_dist
    global left_dist, front_dist, right_dist, back_dist
    global location
    # Send command
    command = f"{motor_speed},{servo_steering},1\n"
    ser.write(command.encode())

    # Wait for response from RP2040
    response = ser.readline().decode().strip()
    values = response.split(",")
    values.pop()
    for index in range(0,len(values)): 
        if values[index]!='': values[index] = float(values[index])
    logging.info(values)    # Logging
    yaw = values[0]
    distance = -round(values[14]/42, 1)
    front_dist = int(values[9])
    right_dist = int(values[10])
    back_dist = int(values[11])
    left_dist = int(values[12])
    print(f"Received Data - Yaw: {yaw}, Distance: {distance-start_dist} Left: {left_dist}, Front: {front_dist}, Right: {right_dist}, Back: {back_dist}\n")
    if left_dist < 10: logging.warning("Close to the left wall!")
    elif right_dist < 10: logging.warning("Close to the right wall!")
    elif front_dist < 15: logging.warning("Might crash, too close")

def forward(speed,steering, target_dist, stop=False):
    global distance
    first_dist = distance
    while not distance - first_dist > target_dist - 8:
        drive_data(speed,steering)
        time.sleep(0.0001)
    if stop: drive_data(0,steering)
    
def backward(speed,steering, target_dist, stop=False):
    global distance
    first_dist = distance
    while not first_dist - distance > target_dist - 8:
        drive_data(-abs(speed),steering)    # Prevent errors caused by thinking speed must be negative
        time.sleep(0.0001)
    if stop: drive_data(0,steering)

def process_frame():
    global hsv_frame, red_obs, green_obs, hsv_roi, frame
    mask_red = cv2.inRange(hsv_roi, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
    
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_obs = get_obstacle_positions(red_contours, red_obs)
    green_obs = get_obstacle_positions(green_contours, green_obs)
  
    print(f"Red: {red_obs}")
    print(f"Green: {green_obs}")
    for item in red_obs:
        x = item[0][0]
        y = item[0][1] + 350    #ROI was cropped
        w = item[0][2]
        h = item[0][3]
        cv2.rectangle(frame, (x,y), (x+w,y+h), (100,100,255), 2)
    for item in green_obs:
        x = item[0][0]
        y = item[0][1] + 350    # ROI was cropped
        w = item[0][2]
        h = item[0][3]
        cv2.rectangle(frame, (x,y), (x+w,y+h), (100,255,100), 2)

    return frame, red_obs, green_obs

def get_obstacle_positions(contours, obs):
    obs = []
    min_area = 500  # minimum contour area to be obstacle in pixels
    max_area = 50000

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area and cv2.contourArea(cnt) < max_area:
            x,y,w,h = cv2.boundingRect(cnt)
            if ((h*2>w and not turning) or (turning and h*1.2>w)) or (y > 140 and abs(1200-x) < 250 and h > 100):
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

def pi_control(error):
    global total_error
    total_error += error
    correction = 0
    if error > 0: correction = error * 2.5 - total_error * 0.001    #right
    elif error < 0: correction = error * 3.3 - total_error * 0.0013 - 2  #left
    print(error)
    steering = 90 + correction 
    steering = min(max(30,steering),145)       #  Limit PID steering
    print("PI Straight")
    return steering

def decide_turn_path():
    # Deciding the turn path based on start position of turn and obstacle seen on the corner
    global yaw, total_error, turns, turning, turn_obs
    global prev_obs, turn_forward, prev_steering, prev_turns
    current_obs = nearest_obstacle()
    print(f'Current obstacle: {current_obs}')
    speed = 200
    steering = prev_steering
    path = 'Straight'
    x = current_obs[0][0]
    y = current_obs[0][1]
    w = current_obs[0][2]
    h = current_obs[0][3]
    colour = current_obs[1]
    if turning:target_yaw = (turn_dir*(turns+1)*90 + 360) % 360
    else: target_yaw = (turn_dir*turns*90 + 360) % 360
    error = target_yaw - yaw
    if error > 180: error = error - 360
    elif error < -180: error = error + 360
    print(f"Target yaw = {target_yaw}, error = {error}")

    if start_pos == 'outer':            # Starting turn near the outer wall
        # TODO Fix this
        if turn_obs[1] == '' and colour != '': turn_obs = current_obs  
        if abs(error) > 80 and turning:
            if turn_dir == 1: steering = 155
            elif turn_dir == -1: steering = 10
            speed = 195
        elif abs(error) > 48 and turning:
            if turn_dir == 1: steering = 165
            elif turn_dir == -1: steering = 3
            speed = 195    
        elif colour == 'green' and y > 40 and x < 1260: 
            path = 'LEFT'
            steering -= (1200-x) * 0.1
        elif colour == 'red' and y > 40 and (x + w) > 80: 
            path = 'RIGHT'
            steering += x*0.09
        else:
            steering = pi_control(error)    
        if abs(error) < 20 and ((left_dist < 20 and turn_obs[1]=='green') or (right_dist < 20 and turn_obs[1]=='red') or turn_obs[1]=='') : 
            turning = False
            turns += 1
    
    # TODO no obstacle
    elif start_pos == 'inner':              # Starting turn near the inner wall
        if turn_obs[1] == '' and colour != '': turn_obs = current_obs
        if turn_obs[1] == 'red' and turning:
            if turn_dir == -1:
                if turn_forward == 0: 
                    turn_forward = 1
                    steering = 149
                if turn_forward == 1 and abs(90-error)>20: 
                    steering = 3
                    turn_forward = 2
            elif turn_dir == 1: steering = 165
        elif turn_obs[1] == 'green' and turning:
            if turn_dir == 1:
                if turn_forward == 0: 
                    turn_forward = 1
                    steering = 15
                if turn_forward == 1 and abs(error-90)>20: 
                    steering = 165
                    turn_forward = 2
            elif turn_dir == -1: steering = 3
        elif turning and turn_obs[1] == '':
            # TODO When no obstacle
            pass
        elif not turning:
            steering = pi_control(error)

        if turning and ((abs(error)<10 and turn_forward == 2) or (abs(error)<25 and turn_forward == 0)):       # End of turn determination
            turning = False
            turn_forward = 0
            turns += 1
            logging.info("Turn completed")
            print("\n\n\tTurn completed\n")

        # Go back if had to turn closer to inner wall and need to change lanes again due to different colour
        if prev_turns < turns:
            if current_obs[1] != turn_obs[1] and current_obs[1] != '' and turn_obs[1] != '':
                print("\n\tGoing back post turn\n\n") 
                backward(225,90,13,True)
                logging.info("Going back")
            else:
                print("\n\tNo need to go back post turn completion\n\n") 
                logging.info("Not going back")
            turn_obs = [(0,0,0,0),'']

    print(f"Turn obs- {turn_obs}")
    print(f"Turn value: {turn_forward}")
    if turn_obs[1]!='' and abs(error)<25:   # Check if obstacle has been passed
        print("\n\n\n\tObstacle passed!\n\n\n")
    prev_obs, prev_steering, prev_turns = current_obs, steering, turns
    return path, speed, steering

def run():
    global frame, hsv_frame, video_out, hsv_roi
    global yaw, distance, left_dist, front_dist, right_dist, back_dist, turning, turns, start_dist, prev_turns
    if front_dist < 100: backward(200,90,97-front_dist,True)
    while True:        
        frame = picam2.capture_array()  # Read a frame from the camera
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert  frame to HSV format
        hsv_roi = hsv_frame[350:720, 0:1280]        # Region of interest is only the bottom half
        
        frame_processed, red_obs, green_obs = process_frame() # Process frame for obstacles

        # Decide navigation based on obstacle detection
        path_action, speed, steering = decide_turn_path()
        #if (distance - start_dist) > 125: break # Stop after turn
        if prev_turns == 1: break
        # Camera feed and analysis display
        cv2.putText(frame_processed, f"Speed {speed} Steering {steering}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2)
        cv2.imshow("Obstacle Detection", frame_processed)
        video_out.write(frame_processed)

        drive_data(speed, steering)
        print(f"Steering: {steering}")

        if cv2.waitKey(1) & 0xFF == ord('q'):   # Manual kill switch
            break

try:
    picam2.configure(config)
    picam2.start_preview()
    led()
    picam2.start()
    drive_data(0,90)
    start_dist = distance
    turning = True
    run()

finally:
    drive_data(0,90)            # Stop robot
    video_out.write(picam2.capture_array())
    video_out.release()
    picam2.stop_preview()       # Close camera
    picam2.stop()
    cv2.destroyAllWindows()
    led(1.5)                    # Blink LED
    GPIO.cleanup()