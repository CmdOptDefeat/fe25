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

  // Variables determined while driving
  int16_t _frontStartDist;                  // Distance from front LiDAR when algorithm is started. Used to stop the vehicle in the same section it started in
  bool roundDirCW;                          // True if round direction is clockwise, false if counterclockwise.
  
  // Driving constants
  const int16_t _frontTurnThreshold = 50;   // When the front LiDAR distance is <= this value, the vehicle starts turning
  const int16_t _frontProbeThreshold = 30;  // For the first straight section, the vehicle mvoes forward until the front LiDAR distance is <= this value, then gets round direction.
  const int16_t _absBaseSpeed = 300;        // General driving speed
  const int16_t _absTurnSpeed = 400;        // General turning speed;
  const int16_t _absSlowSpeed = 200;        // Driving speed for the final straight section

  void _getStartDist();
  void _initialStraight();

};