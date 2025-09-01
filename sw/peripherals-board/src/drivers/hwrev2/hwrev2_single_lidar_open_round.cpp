/**
 * @brief Implementation of Single Lidar Open Round drive algorithm
 * @author Pranav
 */

#include "hwrev2_single_lidar_open_round.hpp"
#include <utility/imumaths.h>

hw_rev_2_SingleLidarOpenRound::hw_rev_2_SingleLidarOpenRound(VehicleConfig cfg){
  _config = cfg;
}

void hw_rev_2_SingleLidarOpenRound::init(ILogger* logger) {

  VehicleData vehicleData;
  _debugLogger = logger;
  _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::init()", _debugLogger->INFO, "Initialising drive algorithm");
  front_start_dist = vehicleData.lidar[0];
  right_start_dist = vehicleData.lidar[90];
  left_start_dist = vehicleData.lidar[270];
  speed = 225;                      // Initial speed
  VehicleCommand{.targetSpeed = speed, .targetYaw = 90}; // Set initial speed, steering  

}

VehicleCommand hw_rev_2_SingleLidarOpenRound::drive(VehicleData vehicleData){
  
  VehicleCommand command;

  // Getting basic data
  yaw = vehicleData.orientation.x;
  distance = - vehicleData.encoderPosition / 43;    // Negative for Pranav's version of robot
  // Get data from LiDARs
  front_lidarDist = vehicleData.lidar[0];
  left_lidarDist = vehicleData.lidar[270];
  right_lidarDist = vehicleData.lidar[90];

  // Determine turn direction based on side LiDARs
  if (turnDir == 0){
    if (left_lidarDist - right_lidarDist > 100){ 
      turnDir = -1; // Turning to left
      threshold = 95;
    }
    else if (left_lidarDist - right_lidarDist < -100){ 
      turnDir = 1; // Turning to right
      threshold = 75; 
    }  
  }


  float error = targetYaw - yaw;
  if(error > 180) error -= 360;
  else if(error < -180) error += 360;

  // Logic while turning and to stop turning
  if (turning){
    if ((abs(error) <= 12 && turnDir == 1) || (abs(error) <= 9 && turnDir == -1)) {   // Return to straight after turning for ~83.5°
        speed = 300;
        turning = false;
        encoderValue = 0;
        distance = 0;
        turns += 1;
        pos = 90; // Reset servo position
        targetYaw -= turns * turnDir * 0.75;
        command.targetYaw = pos;
        _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Stopping turn TY:" + String(targetYaw) + " deg Yaw" + String(yaw) + " deg");
    }
    else{
      if (turnDir == 1) pos = 90 + 75;
      else pos = 90 - 88;
    }
  }

  // Checking to turn, not turning anyways.
  else if (front_lidarDist <= threshold && abs(error) < 15 && ((turns == 0 && (left_lidarDist + right_lidarDist) > 120) || (turns != 0))){
    turning = true;
    targetYaw = ((turnDir * (turns + 1) * 90) + 360) % 360;
    if (turnDir == 1) pos = 90 + 80; // Set servo position for turning
    else if (turnDir == -1) pos = 90 - 84;

    if (turns == 0 && front_lidarDist < 70 && (left_lidarDist + right_lidarDist) > 120 && backward == 0){
      // Go back for first turn in case of extended wall
      if (turnDir == -1){
        /*if (left_start_dist < 20 && right_start_dist < 50){ 
          backward = 10;
          threshold += 5;
        }
        else*/ backward = 25;
      }
      else if (turnDir == 1){
        /*if (left_start_dist < 50 && right_start_dist < 20){ 
          backward = 7;
          threshold += 5;
        }
        else*/ backward = 15;
      }
      back_start = distance;
      turning = false;
    }
    else if (turns == 0 && back_start != 0){
      speed = 300;
    }
    else speed = 275;
    
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Turning; Target " + String(targetYaw) + "  ;Front " + String(front_lidarDist));
  }

  // Not turning - Gyro straight follower
  else{
    speed = 250;
    float correction = 0;
    totalError += error;                            // Used for integral control

    if (error > 0) correction = error * 3.3 - totalError * 0.001; // correction to the right
    else if (error < 0) correction = error * 3.2 - totalError * 0.001; // correction to the left

    if (correction > PID_TURN_LIMIT_ABS) correction = PID_TURN_LIMIT_ABS;
    else if (correction < -PID_TURN_LIMIT_ABS) correction = -PID_TURN_LIMIT_ABS;
    pos = 90 + correction; // Set servo position based on correction

  } 

  // Check is 3 rounds are completed
  if (turns == 12 && front_lidarDist > 120 && front_lidarDist < 170){
    completed = true;
    speed = 0; // Stop the vehicle
    pos = 90; // Reset servo position
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Completed 3 rounds");
  }

  //if(turnDir == -1 && pos < 100 && pos > 80 && pos!=90) pos -= 10;

  if(turns == 0){
    if (turning) speed = 300;
    else speed = 225; 
  }
  
  if (backward!=0 && gone_back == false){
    speed = -190;
    pos = 90;
  }
  if ((backward!=0) && ((back_start - distance) > (backward - 5)) && gone_back == false){
    backward = 0;
    back_start = 0;
    pos = 90;
    speed = 250;
    if (turnDir == 1) turns += 1;
  }

  command.targetSpeed = speed;
  // This sets servo position not yaw since this system is currently on direct control
  // Ref .hpp file with line 19 - `bool isDirectControl() override { return true; }`
  command.targetYaw = int(pos);

  _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Speed " + String(speed) + "  ;Steering " + String(pos));

  return command;
}