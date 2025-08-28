#pragma once

#include <IDriveAlgorithm.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>

enum hw_rev_2_unpark_state{

  FIND_ROUND_DIRECTION,
  TURN0,
  TURN1,
  TURN2,
  TURN3,
  TURN4,
  STOP,

};

class hw_rev_2_UnparkAlgorithm: public IDriveAlgorithm{

public:
  hw_rev_2_UnparkAlgorithm(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData data) override;
  bool isDirectControl() override {return true;}
  bool isFinished() override;
  void _findRoundDir();
  void _turn0();
  void _turn1();
  void _turn2();
  void _turn3();
  void _turn4();
  void _stop();

private:
  VehicleConfig _config;
  ILogger *_logger;
  hw_rev_2_unpark_state _state;
  VehicleData _data;
  VehicleCommand _cmd;
  const int16_t _absBaseSpeed = 200;

  bool _roundDirCW;  // true if clockwise, false if counterclockwise

};