/**
 * @brief Header for Single Lidar Open Round drive algorithm
 * @author Pranav
 */

#pragma once
#include <IDriveAlgorithm.hpp>
#include <ILogger.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>

class hw_rev_2_SingleLidarOpenRound: public IDriveAlgorithm{

public:
    hw_rev_2_SingleLidarOpenRound(VehicleConfig cfg);
    void init(ILogger* logger) override;
    VehicleCommand drive(VehicleData vehicleData) override;
    bool isDirectControl() override {return true;}
    bool isFinished() override {return completed;}      

private:
    VehicleConfig _config;
    VehicleData vehicleData;
    ILogger *_debugLogger;

    // About driving
    int dir = 1;
    int16_t speed = 175;  // Motor PWM speed 
    int lastEncoded = 0; 
    long encoderValue = 0;
    float distance = 0.0;

    // About turning
    int turns = 0;
    bool turning = false;
    int turnDir = 0;           // 1 for clockwise, -1 for counterclockwise. Not known at start
    
    uint8_t pos = 90;          // variable to store the servo position  

    // About IMU
    float yaw;
    float targetYaw = 0;

    unsigned long startMillis;
    unsigned long currentMillis;

    // About Lidar
    int threshold = 100;         // The distance at which the robot should start turning
    int16_t front_lidarDist;
    int16_t left_lidarDist;
    int16_t right_lidarDist;

    // About Gyro straight follower
    int correction = 0;
    float error = 0;
    float totalError = 0;            // Used for integral control

    bool completed = false;          // Indicates if the 3 rounds are completed

    const int PID_TURN_LIMIT_ABS = 55;

};