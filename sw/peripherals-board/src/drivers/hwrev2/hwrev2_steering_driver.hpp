/**
 * @brief Headerfor hwrev2 steering driver
 * @author DIY Labs
 */

#pragma once

#include <interfaces/ISteeringDriver.hpp>
#include <managers/config.hpp>
#include <Arduino.h>
#include <Servo.h>

class hw_rev_2_SteeringDriver : public ISteeringDriver{
public:
  hw_rev_2_SteeringDriver(VehicleConfig cfg);

  virtual void init(ILogger* logger) override;
  virtual void steer(int steeringAngle) override;  

private:
  VehicleConfig _config;
  Servo _steeringServo;
  ILogger* _logger;
  const float _ratio = 1;
  const int _SERVO_MIN = 0;
  const int _SERVO_CENTER = 90;
  const int _SERVO_MAX = 180;
  const int _COMMAND_MIN = -90;
  const int _COMMAND_CENTER = 0;
  const int _COMMAND_MAX = 90;
};