#pragma once
#include <IDriveAlgorithm.hpp>
#include <vehicledata.hpp>
#include <vehiclecommand.hpp>
#include <config.hpp>

class hw_rev_2_WallFollowingOpenRound: public IDriveAlgorithm{

public:
  hw_rev_2_WallFollowingOpenRound(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData data) override;
  bool isFinished() override {return false;}
  bool isDirectControl() override {return true;}

private:
  VehicleConfig _cfg;
  VehicleData _data;
  VehicleCommand _cmd;
  ILogger *_logger;

  int16_t _leftWallController();
  int16_t _rightWallController();

  const double _leftWallP = 2.5;
  const double _rightWallP = 2.5;
  const double _leftWallTarget = 10;
  const double _rightWallTarget = 10;
  const int16_t _absWallBaseSpeed = 300;

};