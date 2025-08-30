#include <hwrev2_rf24_communication.hpp>

hw_rev_2_RF24Communication::hw_rev_2_RF24Communication(VehicleConfig cfg){

  _config = cfg;

  _radio = new RF24(_config.pinConfig.nrfCE, _config.pinConfig.nrfCS);

}

void hw_rev_2_RF24Communication::init(ILogger *logger){

  _logger = logger;

  if(!_radio->begin(&SPI1)){

    _logger->sendMessage("hw_rev_2_RF24Communication::init", _logger->ERROR, "RF24 init failed");

  }
  else{

    _logger->sendMessage("hw_rev_2_RF24Communication::init", _logger->ERROR, "RF24 init failed");

  }

  _radio->setPALevel(RF24_PA_HIGH);
  _radio->setDataRate(RF24_2MBPS);
  _radio->setAutoAck(true);
  _radio->enableAckPayload();
  _radio->setAddressWidth(5);

}

VehicleCommand hw_rev_2_RF24Communication::update(VehicleData data, VehicleCommand cmd){

  VehicleCommand returnCommand;

  hwrev2_rf24_telem_block1 telemBlock1;
  hwrev2_rf24_cmd_block1 cmdBlock1;

  telemBlock1.oriX              = data.orientation.x;
  telemBlock1.lidarLeft         = data.lidar[270];
  telemBlock1.lidarFront        = data.lidar[0];
  telemBlock1.lidarRight        = data.lidar[90];
  telemBlock1.lidarBack         = data.lidar[180];
  telemBlock1.commandedSpeed    = cmd.targetSpeed;
  telemBlock1.commandedSteer    = cmd.targetYaw;
  telemBlock1.millis            = millis();
  telemBlock1.roundDirectionCW  = data.roundDirectionCW;

  _radio->openWritingPipe(TLM_PIPE_0);
  _radio->stopListening();
  
  if(!_radio->write(&telemBlock1, sizeof(telemBlock1))){
    _logger->sendMessage("hw_rev_2_RF24Communication::update", _logger->ERROR, "RF24 write failed. Code execution will continue.");
  }
  else{
    _logger->sendMessage("hw_rev_2_RF24Communication::update", _logger->INFO, "Successfully sent telemetry over RF24.");
  }

  /* // Command receive code is disabled, don't have time to debug

  uint8_t ackPipeNum;
  if(_radio->available(&ackPipeNum)){

    _radio->read(&cmdBlock1, sizeof(cmdBlock1));
    returnCommand.targetSpeed = cmdBlock1.targetSpeed;
    returnCommand.targetYaw = cmdBlock1.targetYaw;
    _logger->sendMessage("hw_rev_2_RF24Communication::update", _logger->INFO, "Successfully received command over RF24.");

  }
  else{

    returnCommand.targetSpeed = 0;
    returnCommand.targetYaw = 90;
    _logger->sendMessage("hw_rev_2_RF24Communication::update", _logger->ERROR, "RF24 command read failed. Code execution will continue.");

  }

  */

  return returnCommand;
  
}