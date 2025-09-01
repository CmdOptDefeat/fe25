#include <hwrev2_state_machine_open_round.hpp>

hw_rev_2_StateMachineOpenRound::hw_rev_2_StateMachineOpenRound(VehicleConfig config){

  _cfg = config;

}

void hw_rev_2_StateMachineOpenRound::init(ILogger *logger){

  _logger = logger;
  _state = GET_START_DIST;

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

    case GET_ROUND_DIR:
      _getRoundDir();
      break;

    case INITIAL_MOVE_BACKWARD:
      _initialMoveBackward();
      break;

    case STRAIGHT_0:
      _straight0();
      break;

    case STRAIGHT_90:
      _straight90();
      break;

    case STRAIGHT_180:
      _straight180();
      break;

    case STRAIGHT_270:
      _straight270();
      break;

  }

  return _cmd;

}

void hw_rev_2_StateMachineOpenRound::_getStartDist(){

  _frontStartDist = _data.lidar[0];
  _state = INITIAL_STRAIGHT;

  _cmd.targetSpeed = 0;
  _cmd.targetYaw = 90;

}

void hw_rev_2_StateMachineOpenRound::_initialStraight(){

  if(_data.lidar[0] <= _frontProbeThreshold){
 
    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = GET_ROUND_DIR;
    return;

  }

  _cmd.targetYaw = _steeringController(_data.orientation.x, 0);
  _cmd.targetSpeed = _absSlowSpeed;

}

void hw_rev_2_StateMachineOpenRound::_getRoundDir(){

  uint16_t leftDist = _data.lidar[270];
  uint16_t rightDist = _data.lidar[90];

  _roundDirCW = rightDist > leftDist;

  _logger->sendMessage("hw_rev_2_StateMachineOpenRound::_getRoundDir", _logger->INFO, "Left distance: " + String(leftDist) + ", right distance: " + String(rightDist) + ", _roundDirCW: " + String(_roundDirCW));

  _cmd.targetYaw = 90;
  _cmd.targetSpeed = 0;
  _state = INITIAL_MOVE_BACKWARD;

}

void hw_rev_2_StateMachineOpenRound::_initialMoveBackward(){

  if(_data.lidar[0] >= _frontTurnThreshold){
    
    _cmd.targetYaw = 90;
    _cmd.targetSpeed = 0;
    _state = STRAIGHT_0;
    return;

  }

  _cmd.targetSpeed = -_absSlowSpeed;
  _cmd.targetYaw = 90;

}

void hw_rev_2_StateMachineOpenRound::_straight0(){

  if(_data.lidar[0] <= _frontTurnThreshold && _inDegreeRange(_data.orientation.x, 0, turnMargin)){
    _state = _roundDirCW ? STRAIGHT_90 : STRAIGHT_270;
    _numTurns++;
    return;
  }

  _cmd.targetYaw = _steeringController(_data.orientation.x, 0);  

  _speedCheck();
  _stopCheck();

}

void hw_rev_2_StateMachineOpenRound::_straight90(){

  if(_data.lidar[0] <= _frontTurnThreshold && _inDegreeRange(_data.orientation.x, 90, turnMargin)){
    _state = _roundDirCW ? STRAIGHT_180 : STRAIGHT_0;
    _numTurns++;
    return;
  }

  _cmd.targetYaw = _steeringController(_data.orientation.x, 90);  
  _cmd.targetSpeed = _absBaseSpeed;

  _speedCheck();
  _stopCheck();

}

void hw_rev_2_StateMachineOpenRound::_straight180(){

  if(_data.lidar[0] <= _frontTurnThreshold && _inDegreeRange(_data.orientation.x, 180, turnMargin)){
    _state = _roundDirCW ? STRAIGHT_270 : STRAIGHT_90;
    _numTurns++;
    return;
  }

  _cmd.targetYaw = _steeringController(_data.orientation.x, 180);  
  _cmd.targetSpeed = _absBaseSpeed;

  _speedCheck();
  _stopCheck();

}

void hw_rev_2_StateMachineOpenRound::_straight270(){

  if(_data.lidar[0] <= _frontTurnThreshold && _inDegreeRange(_data.orientation.x, 270, turnMargin)){
    _state = _roundDirCW ? STRAIGHT_0 : STRAIGHT_180;
    _numTurns++;
    return;
  }

  _cmd.targetYaw = _steeringController(_data.orientation.x, 270);  
  _cmd.targetSpeed = _absBaseSpeed;

  _speedCheck();
  _stopCheck();

}

int16_t hw_rev_2_StateMachineOpenRound::_steeringController(double current, double target){

  double error = target - current;
  double proportionalOutput;
  double integralOutput;
  double totalOutput;
  double servoOutput;
  static double errorSum;

  if(error > 180){
    error -= 360;
  }
  else if(error < -180){
    error += 360;
  }

  proportionalOutput = error * _kP;
  integralOutput = errorSum * _kI;

  totalOutput = proportionalOutput + integralOutput;

  servoOutput = 90 + totalOutput;
  servoOutput =  constrain(servoOutput, 20, 160);

  return servoOutput;

}

bool hw_rev_2_StateMachineOpenRound::_inDegreeRange(double degree, double target, double range){

  degree = fmod(fmod(degree, 360.0) + 360.0, 360.0);
  target = fmod(fmod(target, 360.0) + 360.0, 360.0);
  double diff = abs(degree - target);

  if(diff > 180){
    diff = 360 - diff;
  }

  return diff <= range;

}

void hw_rev_2_StateMachineOpenRound::_stopCheck(){

  if(_numTurns >= 12 && _inDegreeRange(_data.orientation.x, 0, turnMargin)){  

    if(_data.lidar[0] < _frontStartDist){

      _cmd.targetSpeed = 0;
      _cmd.targetYaw = 90;
      _state = COMPLETED;

    }

  }

}

void hw_rev_2_StateMachineOpenRound::_speedCheck(){

  if(_numTurns == 12){  

    _cmd.targetSpeed = _absBaseSpeed;

  }
  else{

    _cmd.targetSpeed = _absBaseSpeed;

  }

}
