#pragma once

#include <Arduino.h>

class ILogger{
public:  

  enum LogType{

    INFO,
    WARNING,
    ERROR

  };

  virtual ~ILogger() = default;

  virtual void init() = 0;
  virtual void sendMessage(String sender, LogType type, String message);
  virtual void sendString(String string);
  virtual void addKillHandler(void (*)());
  virtual void addRebootHandler(void (*)());
  virtual void addBootselHandler(void (*)());
  virtual void handleInput();

};