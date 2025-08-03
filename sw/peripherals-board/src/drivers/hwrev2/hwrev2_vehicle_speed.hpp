#pragma once

#include <config.hpp>
#include <ISensor.hpp>
#include <sensordata.hpp>
#include <RotaryEncoder.h>

class hw_rev_2_VehicleSpeed : public ISensor{
public:
  hw_rev_2_VehicleSpeed(VehicleConfig cfg);
  void init() override;
  SensorData update() override;
  void _encoderISR();

private:
  VehicleConfig _config;
  RotaryEncoder *_encoder;
  long _encoderPosition; 
  long _prevEncoderPosition; 
  unsigned long lastUpdateTime = 0;
  long prevEncoderPosition = 0;
  uint16_t _speed; 
  uint16_t _ticksPerCM;

};