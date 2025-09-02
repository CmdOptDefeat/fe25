#include "hwrev2_wall_following_open_round.hpp"

hw_rev_2_WallFollowingOpenRound::hw_rev_2_WallFollowingOpenRound(VehicleConfig cfg){

  _cfg = cfg;

}

void hw_rev_2_WallFollowingOpenRound::init(ILogger *logger){

  _logger = logger;

}

VehicleCommand hw_rev_2_WallFollowingOpenRound::drive(VehicleData data){

  _data = data;

  _cmd.targetYaw = _rightWallController();

  _cmd.targetSpeed = 300;

  return _cmd;

}

int16_t hw_rev_2_WallFollowingOpenRound::_leftWallController(){

  int16_t distanceFromWall = _data.lidar[270];  

  double error = _leftWallTarget - distanceFromWall;

  double output = error * _leftWallP;

  double servoOutput = 90 + output;
  servoOutput = constrain(servoOutput, 20, 160);

  return servoOutput;

}

int16_t hw_rev_2_WallFollowingOpenRound::_rightWallController(){

  int16_t distanceFromWall = _data.lidar[90];  

  double error = _rightWallTarget - distanceFromWall;

  double output = error * _rightWallP;

  double servoOutput = 90 + -output;
  servoOutput = constrain(servoOutput, 20, 160);

  return servoOutput;

}