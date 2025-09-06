#include <hwrev2_unpark.hpp>

hw_rev_2_UnparkAlgorithm::hw_rev_2_UnparkAlgorithm(VehicleConfig cfg)
{

  _config = cfg;
}

void hw_rev_2_UnparkAlgorithm::init(ILogger *logger)
{

  _logger = logger;
  _state = STRAIGHT0;

}

VehicleCommand hw_rev_2_UnparkAlgorithm::drive(VehicleData data)
{

  _data = data;

  switch (_state)
  {

  case STRAIGHT0:
    _straight0();
    break;

  case TURN0:
    _turn0();
    break;

    case TURN1:
    _turn1();
    break;

  case STOP:
    _stop();
    break;

  default:
    break;
  }

  return _cmd;
}

void hw_rev_2_UnparkAlgorithm::_straight0(){

  if(_data.lidar[0] <= 6){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN0;
    return;

  }

  _cmd.targetSpeed = _absBaseSpeed;
  _cmd.targetYaw = 90;

}

void hw_rev_2_UnparkAlgorithm::_turn0(){

  if(_data.lidar[180] <= 6){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN1;
    return;

  }

  _cmd.targetSpeed = -_absTurnSpeed;
  _cmd.targetYaw = _data.roundDirectionCW ? MAX_LEFT_TURN : MAX_RIGHT_TURN;

}

void hw_rev_2_UnparkAlgorithm::_turn1(){

  if(_data.roundDirectionCW){

     if(_data.orientation.x >= 90){

      _cmd.targetSpeed = 0;
      _cmd.targetYaw = 90;
      _state = STOP;
      return;

     }

  }
  else{

     if(_data.orientation.x <= 270){

      _cmd.targetSpeed = 0;
      _cmd.targetYaw = 90;
      _state = STOP;
      return;

     }

  }

  _cmd.targetSpeed = _absTurnSpeed;
  _cmd.targetYaw = _data.roundDirectionCW ? MAX_RIGHT_TURN : MAX_LEFT_TURN;

}

void hw_rev_2_UnparkAlgorithm::_stop(){

  _cmd.targetSpeed = 0;
  _cmd.targetYaw = 90;

}