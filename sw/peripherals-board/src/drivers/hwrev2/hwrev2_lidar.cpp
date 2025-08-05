/**
 * @brief Implementation of hwrev2 lidar driver
 * @author DIY Labs
 */

#include "hwrev2_lidar.hpp"
#include <Adafruit_BNO055.h>

hw_rev_2_lidar::hw_rev_2_lidar(VehicleConfig cfg){
  
  _config = cfg;

}

void hw_rev_2_lidar::init(){

  _lidar = new TFLI2C();

}

SensorData hw_rev_2_lidar::update(){

  SensorData data;
  data.sensorDataType = SENSOR_LIDAR;


  _lidar->getData(data.lidar[270], _config.addressConfig.leftLidarAddr);
  _lidar->getData(data.lidar[0], _config.addressConfig.frontLidarAddr);
  _lidar->getData(data.lidar[90], _config.addressConfig.rightLidarAddr);

  return data;
 
}