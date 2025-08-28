#pragma once

#include <IDriveAlgorithm.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>

class hw_rev_2_ParkAlgorithm: public IDriveAlgorithm{

public:
  hw_rev_2_ParkAlgorithm(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData data) override;
  bool isDirectControl() override {return true;}

  enum hw_rev_2_park_state{

    STRAIGHT0,
    TURN0,
    STRAIGHT1,
    STRAIGHT2,
    TURN1,
    STRAIGHTEN_OUT
  
  };

private:
  VehicleConfig _config;
  ILogger *_logger;
  VehicleData _data;
  VehicleCommand _cmd;
  hw_rev_2_park_state _state;
  const int16_t _absBaseSpeed = 200;
  void _straight0();
  void _turn0();
  void _straight1();
  void _straight2();
  void _turn1();
  void _straightenOut();

};