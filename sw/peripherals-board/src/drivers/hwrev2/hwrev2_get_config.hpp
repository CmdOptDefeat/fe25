/**
 * @brief Implements function to get configuration struct
 * @author DIY Labs
 */

#pragma once

#include <managers/config.hpp>

/**
 * @brief Returns vehicle configuration struct for hardware revision 2
 * @return Vehicle configuration struct for hardware revision 2
 */
VehicleConfig hwrev2_getConfig(){

  VehicleConfig cfg;

  cfg.pinConfig.motorDriverDirA             = 26;
  cfg.pinConfig.motorDriverDirB             = 27;
  cfg.pinConfig.motorDriverPWM              = 28;
  cfg.pinConfig.motorDriverStandby          = 15;
  cfg.pinConfig.steeringServo               = 2;

  cfg.pinConfig.motorEncoderA               = 13;
  cfg.pinConfig.motorEncoderB               = 14;

  cfg.pinConfig.uart0RX                     = 1;
  cfg.pinConfig.uart0TX                     = 0;

  cfg.pinConfig.nrfCE                       = 8;
  cfg.pinConfig.nrfCS                       = 9;
  cfg.pinConfig.spi1MISO                    = 12;
  cfg.pinConfig.spi1MOSI                    = 11;
  cfg.pinConfig.spi1SCK                     = 10;

  cfg.pinConfig.i2c0SDA                     = 4;
  cfg.pinConfig.i2c0SCL                     = 5;

  cfg.pinConfig.uart0RX                     = 1;
  cfg.pinConfig.uart0TX                     = 0;

  cfg.pinConfig.lidarMotorPWM               = 29; // Actually the start button now

  cfg.pinConfig.rgbLed                      = 16;

  cfg.limitsConfig.maxForwardSpeed          = 500;
  cfg.limitsConfig.maxForwardSpeed          = 500;

  cfg.constantsConfig.ticksPerCM            = 40;
  cfg.constantsConfig.debugSerialBaudRate   = 115200;
  cfg.constantsConfig.rpiSerialBaudRate     = 115200;

  cfg.addressConfig.bnoAddr                 = 0x28;
  cfg.addressConfig.leftLidarAddr           = 0x10;
  cfg.addressConfig.frontLidarAddr          = 0x20;
  cfg.addressConfig.rightLidarAddr          = 0x30;
  cfg.addressConfig.backLidarAddr           = 0x40;

  cfg.controlConfig.steeringP               = 5.0;
  cfg.controlConfig.steeringI               = 0.5;
  cfg.controlConfig.steeringD               = 0.5;

  cfg.controlConfig.speedP                  = 1.0;
  cfg.controlConfig.speedI                  = 0.1;
  cfg.controlConfig.speedD                  = 0.1;

  cfg.controlConfig.maxSteeringPIDCommand   = 200.0;
  cfg.controlConfig.minSteeringPIDCommand   = -200.0;

  cfg.controlConfig.maxSteeringAngle        = 45.0; 

  return cfg;
  
}