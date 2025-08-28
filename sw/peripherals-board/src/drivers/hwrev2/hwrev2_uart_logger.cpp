/**
 * @brief Implementation for hwrev2 debug logger over UART
 * @author DIY Labs
 */

#include <Arduino.h>
#include "hwrev2_uart_logger.hpp"

hw_rev_2_UARTLogger::hw_rev_2_UARTLogger(VehicleConfig cfg){

  _config = cfg;
  _baudRate = _config.constantsConfig.debugSerialBaudRate;

}

void hw_rev_2_UARTLogger::init(){

  Serial1.begin(_baudRate);

}

void hw_rev_2_UARTLogger::sendMessage(String sender, LogType type, String message){

  String log;

  log += "[";
  log += sender;
  log += " : ";
  log += _stringFromType(type);
  log += " : ";
  log += millis();
  log += "]   ";
  
  log += message;

  Serial1.println(log);

}

void hw_rev_2_UARTLogger::sendString(String string){

  Serial1.print(string);

}

String hw_rev_2_UARTLogger::_stringFromType(LogType type){

  String typeString;

  switch(type){

    case INFO:
      typeString = "INFO";  
      break;

    case WARNING:
      typeString = "WARN";  
      break;

    case ERROR:
      typeString = "ERR";  
      break;

  }

  return typeString;

}

void hw_rev_2_UARTLogger::addKillHandler(void (*funcptr)()){

  _killCallback = funcptr;

}

void hw_rev_2_UARTLogger::addRebootHandler(void (*funcptr)()){

  _rebootCallback = funcptr;

}

void hw_rev_2_UARTLogger::addBootselHandler(void (*funcptr)()){

  _bootselCallback = funcptr;

}

void hw_rev_2_UARTLogger::handleInput(){

  if(Serial1.available()){

    String rawCommand = Serial1.readString();
    sendMessage("hw_rev_2_UARTLogger::handleInput", INFO, "Received command " + rawCommand);

    if(rawCommand  == "kill"){
      sendMessage("hw_rev_2_UARTLogger::handleInput", INFO, "Parsed kill command");
      if(_killCallback != nullptr){_killCallback();}
      return;
    }

    if(rawCommand  == "reboot"){
      sendMessage("hw_rev_2_UARTLogger::handleInput", INFO, "Parsed reboot command");
      if(_rebootCallback != nullptr){_rebootCallback();}
      return;
    }

    if(rawCommand  == "bootsel"){
      sendMessage("hw_rev_2_UARTLogger::handleInput", INFO, "Parsed bootsel command");
      if(_bootselCallback != nullptr){_bootselCallback();}
      return;
    }

    sendMessage("hw_rev_2_UARTLogger::handleInput", INFO, "Unable to parse command " + rawCommand);    

  }

}