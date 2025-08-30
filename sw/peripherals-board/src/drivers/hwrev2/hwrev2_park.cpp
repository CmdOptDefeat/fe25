#include <hwrev2_park.hpp>

hw_rev_2_ParkAlgorithm::hw_rev_2_ParkAlgorithm(VehicleConfig cfg){
  _config = cfg;
}

void hw_rev_2_ParkAlgorithm::init(ILogger *logger){

  _logger = logger;

}

VehicleCommand hw_rev_2_ParkAlgorithm::drive(VehicleData data){

  _data = data;

  switch(_state){

    case STRAIGHT0:
      _straight0();
      break;

    case TURN0:
      _turn0();
      break;

    case STRAIGHT1:
      _straight1();
      break;

    case STRAIGHT2:
      _straight2();
      break;

    case TURN1:
      _turn1();
      break;

  }

  return _cmd;

}

void hw_rev_2_ParkAlgorithm::_straight0(){

  if(_data.roundDirectionCW){

    if(_data.lidar[0] >= 100){
      _cmd.targetYaw = 90;
      _cmd.targetSpeed = 0;
      _state = TURN0;
      return;
    }

    
  }
  else{

    if(_data.lidar[0] >= 60){
      _cmd.targetYaw = 90;
      _cmd.targetSpeed = 0;
      _state = TURN0;
      return;
    }

  }

  _cmd.targetYaw = 90;
  _cmd.targetSpeed = -_absBaseSpeed;

}

void hw_rev_2_ParkAlgorithm::_turn0(){

  if(_data.roundDirectionCW){

    if(_data.orientation.x >= 90){

      _cmd.targetYaw = 90;
      _cmd.targetSpeed = 0;
      _state = STRAIGHT1;
      return;
    }

  }
  else{

    if(_data.orientation.x <= 270){

      _cmd.targetYaw = 90;
      _cmd.targetSpeed = 0;
      _state = STRAIGHT1;
      return;

    }

  }

  
  if(_data.roundDirectionCW){
    _cmd.targetYaw = MAX_LEFT_TURN;
  }
  else{
    _cmd.targetYaw = MAX_RIGHT_TURN;
  }

  _cmd.targetSpeed = -_absTurnSpeed;
  

}

void hw_rev_2_ParkAlgorithm::_straight1(){

  if(_data.lidar[180] <= 5){

    _cmd.targetYaw = 90;
    _cmd.targetSpeed = 0;
    _state = STRAIGHT2;
    return;

  }

  _cmd.targetSpeed = -_absBaseSpeed;
  _cmd.targetYaw = 90;

}

void hw_rev_2_ParkAlgorithm::_straight2(){

  if(_data.lidar[0] <= 30){

    _cmd.targetYaw = 90;
    _cmd.targetSpeed = 0;
    _state = TURN1;
    return;

  }

  _cmd.targetSpeed = _absBaseSpeed;
  _cmd.targetYaw = 90;

}

void hw_rev_2_ParkAlgorithm::_turn1(){

  if(_data.roundDirectionCW){

    _cmd.targetSpeed = -_absTurnSpeed;
    _cmd.targetYaw = MAX_RIGHT_TURN;

  }
  else{

    _cmd.targetSpeed = -_absTurnSpeed;
    _cmd.targetYaw = MAX_LEFT_TURN;

  }

}

bool hw_rev_2_ParkAlgorithm::isFinished(){

  return false;

}