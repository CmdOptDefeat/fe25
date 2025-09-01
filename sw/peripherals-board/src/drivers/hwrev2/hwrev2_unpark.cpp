#include <hwrev2_unpark.hpp>

hw_rev_2_UnparkAlgorithm::hw_rev_2_UnparkAlgorithm(VehicleConfig cfg){
 
  _config = cfg;

}

void hw_rev_2_UnparkAlgorithm::init(ILogger *logger){

  _logger = logger;
  
}

VehicleCommand hw_rev_2_UnparkAlgorithm::drive(VehicleData data){

  _data = data;

  switch(_state){

    case TURN0:
      _turn0();
      break;

    case TURN1:
      _turn1();
      break;

    case TURN2:
      _turn2();
      break;

    case TURN3:
      _turn3();
      break;

    case TURN4:
      _turn4();
      break;

    default:
      break;

  }

  return _cmd;

}

void hw_rev_2_UnparkAlgorithm::_turn0(){

  if(_data.lidar[180] <= 2){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN1;
    return;

  }

  _cmd.targetYaw = 90;
  _cmd.targetSpeed = -_absTurnSpeed;

}

void hw_rev_2_UnparkAlgorithm::_turn1(){

  if(_data.lidar[0] <= 2){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN2;
    return;

  }

  _cmd.targetYaw = _data.roundDirectionCW ? MAX_RIGHT_TURN : MAX_LEFT_TURN;
  _cmd.targetSpeed = _absTurnSpeed;

}

void hw_rev_2_UnparkAlgorithm::_turn2(){

  if(_data.lidar[180] <= 3){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN3;
    return;

  }

  _cmd.targetYaw = _data.roundDirectionCW ? MAX_LEFT_TURN : MAX_RIGHT_TURN;
  _cmd.targetSpeed = -_absTurnSpeed;

}

void hw_rev_2_UnparkAlgorithm::_turn3(){

  if(_data.lidar[0] <= 3){
    _cmd.targetSpeed = 0;
    _state = TURN4;
    return;
  }

  _cmd.targetYaw = _data.roundDirectionCW ? MAX_RIGHT_TURN : MAX_LEFT_TURN;
  _cmd.targetSpeed = _absTurnSpeed;

}

void hw_rev_2_UnparkAlgorithm::_turn4(){

  if(_data.lidar[180] <= 3){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    return;

  }

  _cmd.targetYaw = _data.roundDirectionCW ? MAX_LEFT_TURN : MAX_RIGHT_TURN;
  _cmd.targetSpeed = -_absTurnSpeed;

}


bool hw_rev_2_UnparkAlgorithm::isFinished(){

  return false;

}