/**
 * @brief Preprocessor directives for vehicle driver configuration
 * @note This file must be included after all necessary hardware includes
 * @author DIY Labs
 */

#pragma once

/**
 * @warning HARDWARE REVISION 1 DRIVERS ARE INCOMPLETE, BUGGY, OR MISSING!!
 */
#if defined(VEHICLE_DRIVERSET_HWREV1)

#include <hwrev1_includes.h>
#define VEHICLE_DRIVER_MOTOR hw_rev_1_MotorDriver
#define VEHICLE_DRIVER_STEERING hw_rev_1_SteeringDriver
#define VEHICLE_DRIVER_TARGET_CONTROL hw_rev_1_TargetControl
#define VEHICLE_GET_CONFIG hwrev1_getConfig()

#elif defined(VEHICLE_DRIVERSET_HWREV2)

#include <hwrev2_includes.h>
#define VEHICLE_DRIVER_IMU hw_rev_2_imu
#define VEHICLE_DRIVER_LIDAR hw_rev_2_lidar
#define VEHICLE_DRIVER_SPEED hw_rev_2_VehicleSpeed
#define VEHICLE_DRIVER_MOTOR hw_rev_2_MotorDriver
#define VEHICLE_DRIVER_STEERING hw_rev_2_SteeringDriver
#define VEHICLE_DRIVER_TARGET_CONTROL hw_rev_2_TargetControl
#define VEHICLE_DRIVER_REMOTE_COMMUNICATION hw_rev_2_RF24Communication
#define VEHICLE_DRIVER_SERIAL_COMMUNICATION hw_rev_2_SerialCommunication
//#define VEHICLE_DRIVER_ROS_COMMUNICATION hw_rev_2_ROSCommunication
#define VEHICLE_DRIVER_DEBUG_LOG hw_rev_2_UARTLogger
#define VEHICLE_DRIVER_RGB_LED hwrev2_RGBLED
#define VEHICLE_GET_CONFIG hwrev2_getConfig()

#define VEHICLE_DRIVER_OPEN_ROUND_ALGORITHM hw_rev_2_SingleLidarOpenRound
#define VEHICLE_DRIVER_UNPARK_ALGORITHM hw_rev_2_UnparkAlgorithm
#define VEHICLE_DRIVER_PARK_ALGORITHM hw_rev_2_ParkAlgorithm

#endif