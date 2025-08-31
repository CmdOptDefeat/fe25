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
  
  speed = 1024;                      // Initial speed
  VehicleCommand{.targetSpeed = speed, .targetYaw = 90}; // Set initial speed, steering  

}

VehicleCommand hw_rev_2_SingleLidarOpenRound::drive(VehicleData vehicleData){
  
  VehicleCommand command;

  // Getting basic data
  yaw = vehicleData.orientation.x;
  distance = vehicleData.encoderPosition / 43;
  
  // Stopping turn logic
  //float difference = -turnDir * (targetYaw - yaw);
  //if (difference > 180) difference = 360 - difference;
  //else if (difference < -180) difference = 360 + difference;

  float difference = targetYaw - yaw;

  if(difference > 180){
    difference -= 360;
  }
  else if(difference < -180){
    difference += 360;
  }

  if (turning == true){
    if (abs(difference) <= 6.5f){   // Return to straight after turning for ~89°
        speed = 400;
        turning = false;
        encoderValue = 0;
        distance = 0;
        turns += 1;
        pos = 90; // Reset servo position
        targetYaw -= turns * turnDir * 1;
        command.targetYaw = pos;
        _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Stopping turn TY:" + String(targetYaw) + " deg Yaw" + String(yaw) + " deg");   
    }
    else if (abs(difference) > 10 && abs(difference) < 75){ // Continue turning
        pos = 90 + (60 - (90-abs(difference))/2.5f) * turnDir; // Set servo position for turning
        _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "steering" + String(pos) + " " + String(yaw));
    }
    else if(abs(difference) > 75){

      pos = 90 + turnDir * 70;

    }
    else{

      pos = 90 + (turnDir * 20);

    }

  }

  // Get data from LiDARs
  front_lidarDist = vehicleData.lidar[0];
  left_lidarDist = vehicleData.lidar[270];
  right_lidarDist = vehicleData.lidar[90];

  // Checking to turn
  if ((turning == false) and (left_lidarDist + right_lidarDist > 120)){ 
    speed = 1024;
    if (turns == 11) speed = 400; // Slow down for final turn
    turning = true;
    if (turnDir == 1) pos = 90 + 60; // Set servo position for turning
    else if (turnDir == -1) pos = 90 - 65;

  /*
    targetYaw = yaw + turnDir * 90;
    if (targetYaw > 360) targetYaw = targetYaw - 360;
    else if (targetYaw < 0) targetYaw = 360 + targetYaw;

    //if ((turns % 4 == 0) and (turnDir == -1)) targetYaw = 270; // First turn in anticlockwise

    if (targetYaw > 75 && targetYaw < 105) targetYaw = 90;
    else if (targetYaw > 165 && targetYaw < 195) targetYaw = 180;
    else if (targetYaw > 255 && targetYaw < 290) targetYaw = 270;
    else if (targetYaw > 345 or targetYaw < 15) targetYaw = 0;
    
    */
    targetYaw = (((turns + 1) * (90 * turnDir)) + 360) % 360;

    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Start turn " + String(targetYaw) + " deg " + String(front_lidarDist));

  }

  // Not turning - Gyro straight follower
  if(turning == false){
    speed = 1024;
    correction = 0;
    error = round(targetYaw - yaw);

    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
    totalError += error;                            // Used for integral control

    if (error > 0) correction = error * 2.3 - totalError * 0.001; // correction to the right
    else if (error < 0) correction = error * 2.2 - totalError * 0.001; // correction to the left

    if (correction > PID_TURN_LIMIT_ABS) correction = PID_TURN_LIMIT_ABS;
    else if (correction < -PID_TURN_LIMIT_ABS) correction = -PID_TURN_LIMIT_ABS;
    pos = 90 + correction; // Set servo position based on correction

  } 

  if (turnDir == 0){ // Determine turn direction based on side LiDARs
    if (left_lidarDist - right_lidarDist > 100){ 
        turnDir = -1; // Turning to left
        threshold = 78;
    }
    else if (left_lidarDist - right_lidarDist < -100){ 
        turnDir = 1; // Turning to right
        threshold = 74;
    }
  }

  //if ((turnDir == 0) && (left_lidarDist > left_startDist + 60)) turnDir = -1; // Turning to left
  //else if ((turnDir == 0) && (right_lidarDist > right_startDist + 60)) turnDir = 1; // Turning to right
  if (turns == 12){
    completed = true;
    speed = 0; // Stop the vehicle
    pos = 90; // Reset servo position
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Completed 3 rounds");
  }

  if(turnDir == -1 && pos != 90){

    pos -= 20;

  }

  command.targetSpeed = speed;
  // This sets servo position not yaw since this system is currently on direct control
  // Ref .hpp file with line 19 - bool isDirectControl() override { return true; }
  command.targetYaw = int(pos);

  return command;

}