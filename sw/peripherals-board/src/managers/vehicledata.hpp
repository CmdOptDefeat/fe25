/**
 * @brief Struct for storing data collected from sensors
 * @author DIY Labs
 */

#pragma once

#include "vec3f.hpp"
#include <cstdint>
#include "vehicleinstruction.hpp"

struct VehicleData{

  Vec3f orientation = {0, 0, 0};
  Vec3f acceleration = {0, 0, 0};
  Vec3f angularVelocity = {0, 0, 0};
  uint16_t lidar[360] = {0};
  int16_t speed = 0;
  long encoderPosition = 0;
  
  uint8_t imuCalib = 0;
  uint8_t gyroCalib = 0;
  uint8_t accelCalib = 0;
  uint8_t magCalib = 0;

  bool roundDirectionCW = false; // true if clockwise, false if counterclockwise

  VehicleInstruction instruction = NO_INSTRUCTION;

};