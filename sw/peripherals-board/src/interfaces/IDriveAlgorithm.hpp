/**
 * @brief Interface for drive algorithms
 * @author DIY Labs
 */

#pragma once
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <ILogger.hpp>

class IDriveAlgorithm{
public:  

  virtual ~IDriveAlgorithm() = default;

  virtual void init(ILogger* logger) = 0;
  virtual VehicleCommand drive(VehicleData vehicleData) = 0;
  virtual bool isDirectControl() = 0;
  virtual bool isFinished() = 0;

};