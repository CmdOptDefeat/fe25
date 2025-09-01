#include <hwrev2_state_machine_open_round.hpp>

hw_rev_2_StateMachineOpenRound::hw_rev_2_StateMachineOpenRound(VehicleConfig config){

  _cfg = config;

}

void hw_rev_2_StateMachineOpenRound::init(ILogger *logger){

  _logger = logger;
  _turnPID = new PID(&_pidVehicleYaw, &_pidOutput, &_pidAdjustedTargetYaw, _cfg.controlConfig.steeringP, _cfg.controlConfig.steeringI, _cfg.controlConfig.steeringD, DIRECT);
  _turnPID->SetMode(AUTOMATIC);
  _turnPID->SetOutputLimits(_cfg.controlConfig.minSteeringPIDCommand, _cfg.controlConfig.maxSteeringPIDCommand);

}

VehicleCommand hw_rev_2_StateMachineOpenRound::drive(VehicleData vehicleData){

  _data = vehicleData;
/*
  switch(_state){

    case GET_START_DIST:
      _getStartDist();
      break;

    case INITIAL_STRAIGHT:
      _initialStraight();
      break;

  }
*/

  _drivePID(0);
  return _cmd;

}

void hw_rev_2_StateMachineOpenRound::_getStartDist(){

  _frontStartDist = _data.lidar[0];
  _state = INITIAL_STRAIGHT;

}
void hw_rev_2_StateMachineOpenRound::_initialStraight(){

  if(_data.lidar[0] <= _frontTurnThreshold){
 
    _cmd.targetSpeed = 0;
    return;

  }

  _drivePID(0);

}

double hw_rev_2_StateMachineOpenRound::getShortestAngleError(double target, double current) {

  double error = target - current;

  if(error > 180){
    error -= 360;
  }

  else if(error < -180){
    error += 360;
  }

  return error;

}

void hw_rev_2_StateMachineOpenRound::_drivePID(double targetAngle){

  _pidVehicleYaw = _data.orientation.x;
  _pidAdjustedTargetYaw = targetAngle;

  _pidYawError = getShortestAngleError(_pidAdjustedTargetYaw, _pidVehicleYaw);

  if(_data.orientation.x - _pidAdjustedTargetYaw > 180){
    _pidAdjustedTargetYaw += 360;
  }  
  else if(_data.orientation.x - _pidAdjustedTargetYaw < -180){
    _pidAdjustedTargetYaw -= 360;
  }

  _turnPID->Compute();

  _cmd.targetYaw = _pidOutput;
  _cmd.targetSpeed = _absBaseSpeed;

}