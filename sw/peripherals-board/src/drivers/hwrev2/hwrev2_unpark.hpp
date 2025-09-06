#pragma once

#include <IDriveAlgorithm.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>

enum hw_rev_2_unpark_state{

  STRAIGHT0,
  TURN0,
  TURN1,
  STOP

};

class hw_rev_2_UnparkAlgorithm: public IDriveAlgorithm{

public:
  hw_rev_2_UnparkAlgorithm(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData data) override;
  bool isDirectControl() override {return true;}
  bool isFinished() override {return _state == STOP;};

private:
  VehicleConfig _config;
  ILogger *_logger;
  hw_rev_2_unpark_state _state;
  VehicleData _data;
  VehicleCommand _cmd;
  const int16_t _absBaseSpeed = 70;
  const int16_t _absTurnSpeed = 80;
  const int16_t MAX_LEFT_TURN = 5;
  const int16_t MAX_RIGHT_TURN = 175;

  void _straight0();
  void _turn0();
  void _turn1();
  void _stop();

};