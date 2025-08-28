/**
 * @brief Header for hwrev2 debug logger over UART
 * @author DIY Labs
 */

 #pragma once

 #include <ILogger.hpp>
 #include <config.hpp>

class hw_rev_2_UARTLogger : public ILogger{

public:
  hw_rev_2_UARTLogger(VehicleConfig cfg);
  void init() override;
  void sendMessage(String sender, LogType type, String message) override;
  void sendString(String string) override;
  void addKillHandler(void (*funcptr)()) override;
  void addRebootHandler(void (*)()) override;
  void addBootselHandler(void (*)()) override;  
  void handleInput() override;

private:
  VehicleConfig _config;
  uint32_t _baudRate;
  String _stringFromType(LogType type);
  void (*_killCallback)();
  void (*_rebootCallback)();
  void (*_bootselCallback)();

};