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
    GET_ROUND_DIR,
    INITIAL_MOVE_BACKWARD,
    STRAIGHT_0,
    STRAIGHT_90,
    STRAIGHT_180,
    STRAIGHT_270,
    COMPLETED


  };

  hw_rev_2_StateMachineOpenRound(VehicleConfig config);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData vehicleData) override;
  bool isDirectControl() override {return true;}
  bool isFinished() override {return _state == COMPLETED;}  

private:

  ILogger* _logger;
  VehicleConfig _cfg;
  VehicleCommand _cmd;
  VehicleData _data;
  DriveState _state;

  // Variables determined while driving
  int16_t _frontStartDist;                  // Distance from front LiDAR when algorithm is started. Used to stop the vehicle in the same section it started in
  bool _roundDirCW;                         // True if round direction is clockwise, false if counterclockwise.
  uint8_t _numTurns;                        // How many turns the vehicle has performed. Used to count rounds;
  
  // Driving constants
  const int16_t _frontTurnThreshold = 100;   // When the front LiDAR distance is <= this value, the vehicle starts turning
  const int16_t _frontProbeThreshold = 40;  // For the first straight section, the vehicle mvoes forward until the front LiDAR distance is <= this value, then gets round direction.
  const int16_t _absBaseSpeed = 1024;        // General driving speed
  const int16_t _absSlowSpeed = 150;        // Slower general driving speed.
  const double _kP = 2;                     // Proportional constant for steering controller
  const double _kI = 0.01;                  // Integral constant for steering controller
  const int turnMargin = 5;                 // The vehicle orientation must be within +- turnMargin of the target yaw before the vehicle considers turning

  void _getStartDist();
  void _initialStraight();
  void _getRoundDir();
  void _initialMoveBackward();
  int16_t _steeringController(double current, double target);
  void _straight0();
  void _straight90();
  void _straight180();
  void _straight270();
  bool _inDegreeRange(double degree, double target, double range);
  void _stopCheck();
  void _speedCheck();

};