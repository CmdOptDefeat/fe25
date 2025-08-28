#include <hwrev2_unpark.hpp>

hw_rev_2_UnparkAlgorithm::hw_rev_2_UnparkAlgorithm(VehicleConfig cfg){
 
  _config = cfg;

}

void hw_rev_2_UnparkAlgorithm::init(ILogger *logger){

  _logger = logger;
  _state = TURN0;   // round direction is figured out by main, so skip FIND_ROUND_DIRECTION

}

VehicleCommand hw_rev_2_UnparkAlgorithm::drive(VehicleData data){

  _data = data;
  _roundDirCW = _data.roundDirectionCW;   // driver was written before round dir field was added
                                          // to main, set private round dir variable
  switch(_state){

    case FIND_ROUND_DIRECTION:
      _findRoundDir();
      break;

    case TURN0:
      _turn0();
      _logger->sendMessage("hw_rev_2_UnparkAlgorithm::drive", _logger->INFO, "Turn 0");
      break;

    case TURN1:
      _turn1();
      _logger->sendMessage("hw_rev_2_UnparkAlgorithm::drive", _logger->INFO, "Turn 1");
      break;

    case TURN2:
      _turn2();
      _logger->sendMessage("hw_rev_2_UnparkAlgorithm::drive", _logger->INFO, "Turn 2");
      break;

    case TURN3:
      _turn3();
      _logger->sendMessage("hw_rev_2_UnparkAlgorithm::drive", _logger->INFO, "Turn 3");
      break;

    case TURN4:
      _turn4();
      _logger->sendMessage("hw_rev_2_UnparkAlgorithm::drive", _logger->INFO, "Turn 4");
      break;

    case STOP:
      _stop();
      _logger->sendMessage("hw_rev_2_UnparkAlgorithm::drive", _logger->INFO, "STOP");
      break;

    default:
      break;

  }

  return _cmd;

}

/**
 * @brief Find out direction of round based on difference in side LiDAR data, move on to next step
 */
void hw_rev_2_UnparkAlgorithm::_findRoundDir(){

  _cmd.targetYaw = 90;
  _cmd.targetSpeed = 0;

  uint16_t leftLidarDist = _data.lidar[270];
  uint16_t rightLidarDist = _data.lidar[90];

  if(leftLidarDist - rightLidarDist > 10){
    _roundDirCW = false;
  }
  else{
    _roundDirCW = true;
  }

  _logger->sendMessage("hw_rev_2_UnparkAlgorithm::_findRoundDir()", _logger->INFO, "Left LiDAR: " + String(leftLidarDist) + ", right LiDAR: " + String(rightLidarDist));
  _logger->sendMessage("hw_rev_2_UnparkAlgorithm::_findRoundDir()", _logger->INFO, String(_roundDirCW));

  _state = TURN0;

}

void hw_rev_2_UnparkAlgorithm::_turn0(){

  if(_data.lidar[180] <= 3){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN1;

  }

  else{

    _cmd.targetSpeed = -_absBaseSpeed;

    if(_roundDirCW){
      _cmd.targetYaw = 0;
    }

    else{
      _cmd.targetYaw = 180;
    }

  }

}

void hw_rev_2_UnparkAlgorithm::_turn1(){

  if(_data.lidar[180] >= 22){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN2;

  }

  if(_roundDirCW){

    _cmd.targetSpeed = _absBaseSpeed;
    _cmd.targetYaw = 180;

  }
  else{
    
    _cmd.targetSpeed = _absBaseSpeed;
    _cmd.targetYaw = 0;

  }

}

void hw_rev_2_UnparkAlgorithm::_turn2(){

  if(_data.lidar[180] <= 5){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = TURN3;

  }

  if(_roundDirCW){

    _cmd.targetSpeed = -_absBaseSpeed;
    _cmd.targetYaw = 0;

  }
  else{
    
    _cmd.targetSpeed = -_absBaseSpeed;
    _cmd.targetYaw = 180;

  }

}

void hw_rev_2_UnparkAlgorithm::_stop(){

  _cmd.targetSpeed = 0;
  _cmd.targetYaw = 90;

}

void hw_rev_2_UnparkAlgorithm::_turn3(){

  if(_roundDirCW){
    if(_data.orientation.x >= 90){
      _cmd.targetSpeed = 0;
      _cmd.targetYaw = 90;
      _state = TURN4;
    } 
  }

  else{
    if(_data.orientation.x <= 270){
      _cmd.targetSpeed = 0;
      _cmd.targetYaw = 90;
      _state = TURN4;
    } 
  }

  if(_roundDirCW){

    _cmd.targetSpeed = _absBaseSpeed;
    _cmd.targetYaw = 180;

  }
  else{
    
    _cmd.targetSpeed = _absBaseSpeed;
    _cmd.targetYaw = 0;

  }

}

void hw_rev_2_UnparkAlgorithm::_turn4(){

  if(_data.lidar[180] <= 5){

    _cmd.targetSpeed = 0;
    _cmd.targetYaw = 90;
    _state = STOP;

  }

  _cmd.targetSpeed = -_absBaseSpeed;
  _cmd.targetYaw = 90;

}

bool hw_rev_2_UnparkAlgorithm::isFinished(){

  return _state == STOP;

}