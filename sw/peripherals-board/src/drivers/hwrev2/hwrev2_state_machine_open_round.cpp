#include <hwrev2_state_machine_open_round.hpp>

hw_rev_2_StateMachineOpenRound::hw_rev_2_StateMachineOpenRound(VehicleConfig config){

  _cfg = config;

}

void hw_rev_2_StateMachineOpenRound::init(ILogger *logger){

  _logger = logger;

}

VehicleCommand hw_rev_2_StateMachineOpenRound::drive(VehicleData vehicleData){

  _data = vehicleData;

  switch(_state){

    case GET_START_DIST:
      _getStartDist();
      break;

    case INITIAL_STRAIGHT:
      _initialStraight();
      break;

  }


  _cmd.targetYaw = 180;
  _cmd.targetSpeed = 300;
  return _cmd;

}

void hw_rev_2_StateMachineOpenRound::_getStartDist(){

  _frontStartDist = _data.lidar[0];
  _state = INITIAL_STRAIGHT;

}
void hw_rev_2_StateMachineOpenRound::_initialStraight(){

  if(_data.lidar[0] <= _frontProbeThreshold){
 
    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    return;

  }

  _cmd.targetYaw = 90;
  _cmd.targetYaw = _absBaseSpeed;

}