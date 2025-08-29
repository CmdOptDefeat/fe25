#include <hwrev2_rf24_communication.hpp>

hw_rev_2_RF24Communication::hw_rev_2_RF24Communication(VehicleConfig cfg){

  _config = cfg;

  _radio = new RF24(_config.pinConfig.nrfCE, _config.pinConfig.nrfCS);

}

void hw_rev_2_RF24Communication::init(ILogger *logger){

  _logger = logger;

  _radio->begin(&SPI1);
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
  _radio->write(&telemBlock1, sizeof(telemBlock1));

  uint8_t ackPipeNum;
  if(_radio->available(&ackPipeNum)){

    _radio->read(&cmdBlock1, sizeof(cmdBlock1));
    returnCommand.targetSpeed = cmdBlock1.targetSpeed;
    returnCommand.targetYaw = cmdBlock1.targetYaw;

  }
  else{
    returnCommand.targetSpeed = 0;
    returnCommand.targetYaw = 90;
  }

  return returnCommand;
  
}