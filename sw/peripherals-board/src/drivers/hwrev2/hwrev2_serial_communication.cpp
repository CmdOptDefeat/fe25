#include <hwrev2_serial_communication.hpp>

hw_rev_2_SerialCommunication *commClassPtr = nullptr;

hw_rev_2_SerialCommunication::hw_rev_2_SerialCommunication(VehicleConfig cfg){

  _config = cfg;
  commClassPtr = this;

}

void hw_rev_2_SerialCommunication::init(ILogger *logger){

  _logger = logger;
  _serialTask = new SchedulerTask(_serialCallbackWrapper, 10);  // 10 ms : 100 hz
  Serial.begin(115200);

}

VehicleCommand hw_rev_2_SerialCommunication::update(VehicleData data, VehicleCommand cmd){

  _data = data;
  _displayCmd = cmd;

  _parseSerialInput();

  return _cmd;
  
}

void hw_rev_2_SerialCommunication::_sendFormattedData(VehicleData data){

  const char seperator = ',';

  String message;

  message += String(data.orientation.x);
  message += seperator;
  message += String(data.orientation.y);
  message += seperator;
  message += String(data.orientation.z);
  message += seperator;

  message += String(data.acceleration.x);
  message += seperator;
  message += String(data.acceleration.y);
  message += seperator;
  message += String(data.acceleration.z);
  message += seperator;

  message += String(data.angularVelocity.x);
  message += seperator;
  message += String(data.angularVelocity.y);
  message += seperator;
  message += String(data.angularVelocity.z);
  message += seperator;

  message += String(data.lidar[0]);
  message += seperator;
  message += String(data.lidar[90]);
  message += seperator;
  message += String(data.lidar[180]);
  message += seperator;
  message += String(data.lidar[270]);
  message += seperator;

  message += String(data.speed);
  message += seperator;
  message += String(data.encoderPosition);
  message += seperator;  

  message += String(data.imuCalib);
  message += seperator;
  message += String(data.gyroCalib);
  message += seperator;
  message += String(data.accelCalib);
  message += seperator;
  message += String(data.magCalib);
  message += seperator;

  message += String(_displayCmd.targetSpeed);
  message += seperator;
  message += String(_displayCmd.targetYaw);
  message += seperator;

  message += String(data.roundDirectionCW);
  message += seperator;

  message += String(data.instruction);
  message += seperator;

  message += '\n';
  Serial.print(message);

}

void hw_rev_2_SerialCommunication::_serialCallback(){

  _sendFormattedData(_data);
  _parseSerialInput();

}

void hw_rev_2_SerialCommunication::_serialCallbackWrapper(){
  commClassPtr->_serialCallback();
}

void hw_rev_2_SerialCommunication::_parseSerialInput(){

  if(Serial.available()){

    String rawString = Serial.readStringUntil('\n');

    if(!_isLegalCommand(rawString)){
      
      _logger->sendMessage("hw_rev_2_SerialCommunication::_parseSerialInput", _logger->ERROR, "Invalid command string read: " + rawString);
      return;

    }
    else{

      int comma1Pos = rawString.indexOf(',');
      int comma2Pos = rawString.indexOf(',', comma1Pos + 1);

      _cmd.targetSpeed = rawString.substring(0, comma1Pos).toInt();
      _cmd.targetYaw   = rawString.substring(comma1Pos + 1, comma2Pos) .toInt();
      _cmd.instruction = (VehicleInstruction)(rawString.substring(comma2Pos + 1).toInt());  // cast int to VehicleInstruction

      _sendFormattedData(_data);

    }

  }

}

bool hw_rev_2_SerialCommunication::_isLegalCommand(String inputString){

  int commaCount = 0;

  for(int i = 0; i < inputString.length(); i++){

    if(inputString.charAt(i) == ','){
      commaCount++;
    }

  }

  return commaCount == 2;

}