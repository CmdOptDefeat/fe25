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
  
  _debugLogger = logger;
  _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::init()", _debugLogger->INFO, "Initialising drive algorithm");

  speed = 180;                      // Initial speed
  VehicleCommand{.targetSpeed = speed, .targetYaw = 90}; // Set initial speed, steering  

}

VehicleCommand hw_rev_2_SingleLidarOpenRound::drive(VehicleData vehicleData){
  
  VehicleCommand command;

  // Getting basic data
  yaw = vehicleData.orientation.x;
  distance = vehicleData.encoderPosition / 43;
  // Get data from LiDARs
  front_lidarDist = vehicleData.lidar[0];
  left_lidarDist = vehicleData.lidar[270];
  right_lidarDist = vehicleData.lidar[90];

  // Determine turn direction based on side LiDARs
  if (turnDir == 0){
    if (left_lidarDist - right_lidarDist > 100) turnDir = -1; // Turning to left
    else if (left_lidarDist - right_lidarDist < -100) turnDir = 1; // Turning to right   
  }


  float error = targetYaw - yaw;
  if(error > 180) error -= 360;
  else if(error < -180) error += 360;

  // Logic while turning and to stop turning
  if (turning){
    if (abs(error) <= 6.5f){   // Return to straight after turning for ~83.5°
        speed = 300;
        turning = false;
        encoderValue = 0;
        distance = 0;
        turns += 1;
        pos = 90; // Reset servo position
        targetYaw -= turns * turnDir * 1;
        command.targetYaw = pos;
        _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Stopping turn TY:" + String(targetYaw) + " deg Yaw" + String(yaw) + " deg");   
    }
    else if(turns == 0){
      if(turnDir == 1) pos = 170;
      else pos = 0;
    }    
    else if (abs(error) > 10 && abs(error) < 75){ // 75° to 10°
        pos = 90 + (60 - (90-abs(error))/2.5f) * turnDir; // Set servo position for turning
        _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "steering" + String(pos) + " " + String(yaw));
    }
    else if(abs(error) > 75) pos = 90 + turnDir * 70; //90° to 75°
    else pos = 90 + (turnDir * 20);   //10° to 6.5°
  }


  // Checking to turn
  if (!turning && front_lidarDist <= threshold && abs(error) < 15 && ((turns == 0 && (left_lidarDist + right_lidarDist) > 120) || (turns != 0))){
    if (turns == 0){
      speed = 200;
      if (front_lidarDist < 75){
        // Go back for first turn in case of extended wall
        speed = 200;
      }
    }
    else if (turns == 11) speed = 210;
    else speed = 225;
    turning = true;
    if (turnDir == 1) pos = 90 + 60; // Set servo position for turning
    else if (turnDir == -1) pos = 90 - 65;
    targetYaw = ((turnDir * (turns + 1) * 90) + 360) % 360;
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Turning; Target " + String(targetYaw) + "  ;Front " + String(front_lidarDist));
  }

  // Not turning - Gyro straight follower
  else if(!turning){
    speed = 225;
    float correction = 0;
    totalError += error;                            // Used for integral control

    if (error > 0) correction = error * 3.3 - totalError * 0.001; // correction to the right
    else if (error < 0) correction = error * 3.2 - totalError * 0.001; // correction to the left

    if (correction > PID_TURN_LIMIT_ABS) correction = PID_TURN_LIMIT_ABS;
    else if (correction < -PID_TURN_LIMIT_ABS) correction = -PID_TURN_LIMIT_ABS;
    pos = 90 + correction; // Set servo position based on correction

  } 

  // Check is 3 rounds are completed
  if (turns == 12){
    completed = true;
    speed = 0; // Stop the vehicle
    pos = 90; // Reset servo position
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Completed 3 rounds");
  }

  if(turnDir == -1 && pos < 100 && pos > 80) pos -= 20;

  if(turns == 0){
    if (turning) speed = 200;
    else speed = 180; 
  }

  command.targetSpeed = speed;
  // This sets servo position not yaw since this system is currently on direct control
  // Ref .hpp file with line 19 - `bool isDirectControl() override { return true; }`
  command.targetYaw = int(pos);

  return command;

}