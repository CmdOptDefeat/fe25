#pragma once

#include <IDriveAlgorithm.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <ILogger.hpp>
#include <config.hpp>
#include <PID_v1.h>

class hw_rev_2_StateMachineOpenRound: public IDriveAlgorithm{

public:

  enum DriveState{

    GET_START_DIST,
    INITIAL_STRAIGHT,
    GET_ROUND_DIR

  };

  hw_rev_2_StateMachineOpenRound(VehicleConfig config);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData vehicleData) override;
  bool isDirectControl() override {return true;}
  bool isFinished() override {return false;}  

private:

  ILogger* _logger;
  VehicleConfig _cfg;
  VehicleCommand _cmd;
  VehicleData _data;
  DriveState _state;
  PID *_turnPID;

  double _pidTargetYaw;
  double _pidYawError;
  double _pidAdjustedTargetYaw;
  double _pidVehicleYaw;
  double _pidOutput;

  int16_t _frontStartDist;

  const int16_t _frontTurnThreshold = 50;
  const int16_t _absBaseSpeed = 200;

  void _drivePID(double targetAngle);
  void _getStartDist();
  void _initialStraight();
  double getShortestAngleError(double target, double current);

};