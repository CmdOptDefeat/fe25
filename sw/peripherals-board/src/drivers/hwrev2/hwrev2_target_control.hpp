#pragma once

#include <config.hpp>
#include <IMotorDriver.hpp>
#include <ISteeringDriver.hpp>
#include <ITargetControl.hpp>

class hw_rev_2_TargetControl : public ITargetControl{
public:
  hw_rev_2_TargetControl
(VehicleConfig cfg);
  void init(IMotorDriver* motorDriver, ISteeringDriver* steeringDriver) override;
  void targetControl(VehicleCommand cmd) override;

private:
  VehicleConfig _config;
  IMotorDriver* _motorDriver;
  ISteeringDriver* _steeringDriver;

};