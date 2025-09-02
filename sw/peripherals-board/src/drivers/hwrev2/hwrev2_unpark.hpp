#pragma once

#include <IDriveAlgorithm.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>

enum hw_rev_2_unpark_state{

  TURN0,
  TURN1,
  TURN2,
  TURN3,
  TURN4


};

class hw_rev_2_UnparkAlgorithm: public IDriveAlgorithm{

public:
  hw_rev_2_UnparkAlgorithm(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData data) override;
  bool isDirectControl() override {return true;}
  bool isFinished() override;

private:
  VehicleConfig _config;
  ILogger *_logger;
  hw_rev_2_unpark_state _state;
  VehicleData _data;
  VehicleCommand _cmd;
  const int16_t _absBaseSpeed = 80;
  const int16_t _absTurnSpeed = 250;
  const int16_t MAX_LEFT_TURN = 10;
  const int16_t MAX_RIGHT_TURN = 170;
  int turn_dir = 0;
  void _turn0();
  void _turn1();
  void _turn2();
  void _turn3();
  void _turn4();

};