// TODO: Figure out a clean way of switching between target control sources, ICommunication and IDriveAlgorithm, combine?k
// Maybe implement radio and serial as drive algorithms, how to pass comm drivers to them?

/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"

// COMPETITION_OPEN_ROUND, COMPETITION_OBSTACLE_ROUND
//#define COMPETITION_OPEN_ROUND
#define COMPETITION_OBSTACLE_ROUND
#define VEHICLE_DRIVERSET_HWREV2                        // HWREV2 **NOTE** HWREV1 DRIVERS ARE INCOMPLETE, BUGGY, OR MISSING!!
#define VEHICLE_SW_STATUS "DEV"                         // String containing status of software. Printed over debug port
#define VEHICLE_SW_NAME "Serial Integration Tests"      // String containing name of software. Printed over debug port

#include <driverconfig.hpp>                             // **NOTE** All config #defines must be before this include
#include <SensorManager.hpp>


VEHICLE_DRIVER_IMU bno(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_LIDAR lidar(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_SPEED speed(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_MOTOR motor(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_STEERING steering(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_TARGET_CONTROL targetControl(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_OPEN_ROUND_ALGORITHM openRoundAlgorithm(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_UNPARK_ALGORITHM unparkAlgorithm(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_PARK_ALGORITHM parkAlgorithm(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_REMOTE_COMMUNICATION remoteCommunication(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_SERIAL_COMMUNICATION serialCommunication(VEHICLE_GET_CONFIG);
//VEHICLE_DRIVER_ROS_COMMUNICATION rosCommunication(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_DEBUG_LOG debugLogger(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_RGB_LED rgbLED(VEHICLE_GET_CONFIG);
SensorManager sensorManager(VEHICLE_GET_CONFIG);


enum CoreControlState{

  GET_ORIENTATION,
  UNPARK,
  DRIVE_FROM_PI,
  PARK,
  SAFE

};

VehicleCommand coreVehicleCommand;
VehicleCommand coreSerialCommand;
VehicleData coreVehicleData;
VehicleInstruction coreVehicleInstructionToPi;
VehicleInstruction coreVehicleInstructionFromPi;
CoreControlState coreControlState;
bool coreRoundDirCW;

void coreGetOrientation();
void coreUnpark();
void coreDriveFromPi();
void corePark();
void coreSafe();
void coreOpenRound();
void coreRunStateMachine();


void debugPrintVehicleData(VehicleData data, VehicleCommand cmd);
void debugLogHeader();
void debugLogDataCommand(VehicleData data, VehicleCommand cmd);
void debugProbeI2CAddr(byte addr);
void debugFailureBlink();
void debugKillBlink();
void debugKillCallback();
void debugRebootCallback();
void debugBootselCallback();

/**
 * @brief Initialises sensor manager, target controller, and drive algorithm
 */
void setup(){

  debugLogger.init();  
  debugLogger.addKillHandler(debugKillCallback);
  debugLogger.addBootselHandler(debugBootselCallback);
  debugLogger.addRebootHandler(debugRebootCallback);

  debugLogHeader();

  rgbLED.init(&debugLogger);
  rgbLED.limitBrightness(10);
  rgbLED.setStaticColor(rgbLED.BLUE);

  SPI1.setSCK(VEHICLE_GET_CONFIG.pinConfig.spi1SCK);
  SPI1.setRX(VEHICLE_GET_CONFIG.pinConfig.spi1MISO);
  SPI1.setTX(VEHICLE_GET_CONFIG.pinConfig.spi1MOSI);
  SPI1.begin();

  Serial1.setRX(VEHICLE_GET_CONFIG.pinConfig.uart0RX);
  Serial1.setTX(VEHICLE_GET_CONFIG.pinConfig.uart0TX);  

  Wire.setSCL(VEHICLE_GET_CONFIG.pinConfig.i2c0SCL);
  Wire.setSDA(VEHICLE_GET_CONFIG.pinConfig.i2c0SDA);
  Wire.begin();

  debugLogger.sendMessage("setup()", debugLogger.INFO, "Finished setting communication pins for SPI1, UART1, I2C0");

  sensorManager.init(&debugLogger);

  unparkAlgorithm.init(&debugLogger);
  parkAlgorithm.init(&debugLogger);

  if(!sensorManager.addSensor(&bno)){

    debugLogger.sendMessage("setup()", debugLogger.ERROR, "Failed to add BNO055 driver to sensor manager. Going into infinite loop.");
    
    while(true){
      debugFailureBlink();
    }

  }

  if(!sensorManager.addSensor(&lidar)){

    debugLogger.sendMessage("setup()", debugLogger.ERROR, "Failed to add TFLuna driver to sensor manager. Going into infinite loop.");
    
    while(true){
      debugFailureBlink();
    }

  }

  if(!sensorManager.addSensor(&speed)){

    debugLogger.sendMessage("setup()", debugLogger.ERROR, "Failed to add motor encoder driver to sensor manager. Going into infinite loop.");
    
    while(true){
      debugFailureBlink();
    }

  }
  
  debugLogger.sendMessage("setup()", debugLogger.INFO, "Finished adding drivers to sensor manager");
  rgbLED.setStaticColor(rgbLED.GREEN);

  targetControl.init(&motor, &steering, &debugLogger);

  remoteCommunication.init(&debugLogger);
  serialCommunication.init(&debugLogger);

  coreControlState = GET_ORIENTATION;

}

/**
 * @brief Reads data from sensors, runs state machine, commands output devices
 * @author DIY Labs
 * @note Consider removing debug updates to save loop time for competition, probably not required
 */
void loop(){

  // Handle any serial input over debug port (disable for competition compile?)
  debugLogger.handleInput();

  // Get fresh data from sensors (manually set round direction bool based on what we've found out before)
  coreVehicleData = sensorManager.update();
  coreVehicleData.roundDirectionCW = coreRoundDirCW;
  coreVehicleData.instruction = coreVehicleInstructionToPi;

  // Update state machine
  coreRunStateMachine();

  // Command motors and steering (fixed to directControl regardless of drive command's isDirectControl return; hack since all algorithms written assume direct output)
  targetControl.directControl(coreVehicleCommand, coreVehicleData);

  // Log latest data and drive commands over debug serial port
  debugLogDataCommand(coreVehicleData, coreVehicleCommand);

  // Send latest data and drive commands over nRF24L01+ radio; not using any commands the telemetry adapter sends
  remoteCommunication.update(coreVehicleData, coreVehicleCommand); 

  // Update RPi on communication, set core instruction
  coreSerialCommand = serialCommunication.update(coreVehicleData, coreVehicleCommand);
  coreSerialCommand.instruction = coreVehicleInstructionFromPi;

}

/**
 * @brief Function for printing all collected vehicle data without names for use with SerialPlot
 * @note Do not modify order or add name print to variables 
 * @author DIY Labs
 */
void debugPrintVehicleData(VehicleData data, VehicleCommand cmd){

  Serial.print("Yaw: ");
  Serial.print(data.orientation.x);
  /*Serial.print(" Pitch: ");
  Serial.print(-data.orientation.y);
  Serial.print(" Roll: ");
  Serial.print(data.orientation.z);
  
  Serial.print(" Accel X: ");
  Serial.print(data.acceleration.x);
  Serial.print(" Accel Y: ");
  Serial.print(data.acceleration.y);
  Serial.print(" Accel Z: ");
  Serial.print(data.acceleration.z);
  
  Serial.print(" Angular X: ");
  Serial.print(data.angularVelocity.x);
  /*Serial.print(" Angular Y: ");
  Serial.print(data.angularVelocity.y);
  Serial.print(" Angular Z: ");
  Serial.println(data.angularVelocity.z);*/
  
  Serial.print(" Encoder: ");
  Serial.print(data.encoderPosition);
  Serial.print(" Distance: ");
  Serial.print(data.encoderPosition / 43);
  Serial.print(" Speed: ");
  Serial.println(data.speed);

  Serial.print(" Lidar Left: ");
  Serial.print(data.lidar[270]);
  Serial.print(" Front: ");
  Serial.print(data.lidar[0]);
  Serial.print(" Right: ");
  Serial.print(data.lidar[90]);
  Serial.print(" ");

  Serial.print(" Target Servo ");
  Serial.print(cmd.targetYaw);
  Serial.println();

}

/**
 * @brief Prints a data frame over debug port, frame contains vehicle data and  active vehicle command
 * @author DIY Labs
 */
void debugLogDataCommand(VehicleData data, VehicleCommand cmd){

  debugLogger.sendMessage("debugLogDataCommand()", debugLogger.INFO, "Data frame start");

  debugLogger.sendString(String(data.orientation.x));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.orientation.y));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.orientation.z));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.acceleration.x));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.acceleration.y));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.acceleration.z));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.angularVelocity.x));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.angularVelocity.y));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.angularVelocity.z));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.encoderPosition));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.speed));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.lidar[0]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[90]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[180]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[270]));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(cmd.targetSpeed));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(cmd.targetYaw));
  debugLogger.sendString(", ");

  debugLogger.sendString("\n");

  debugLogger.sendMessage("debugLogDataCommand()", debugLogger.INFO, "Data frame end");

}

/**
 * @brief Prints sw versions over debug port
 * @author DIY Labs
 */
void debugLogHeader(){

  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "CTRL+ALT+DEFEAT Peripherals Board Debug Port");
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Software status: " + String(VEHICLE_SW_STATUS));
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Software name: " + String(VEHICLE_SW_NAME));  
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Compiled on: " + String(__DATE__) + " at: " + String(__TIME__));
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Pico SDK version: " + String(PICO_SDK_VERSION_STRING));

}

/**
 * @note Sometimes doesn't return a success, even when device works. Don't have time to debug it now. Check data stream over debug to ensure sensors are alive.
 */
void debugProbeI2CAddr(byte addr){
    
  Wire.beginTransmission(addr);
  byte err = Wire.endTransmission();

  String errStr;

  switch(err){
    case 0:
      errStr = "Success";
      break;

    case 1:
      errStr = "Data too long";
      break;

    case 2:
      errStr = "NACK on transmit of address";
      break;

    case 3:
      errStr = "NACK on transmit of data";
      break;

    case 4:
      errStr = "Other error";
      break;

    case 5:
      errStr = "Timeout";
      break;

    default:
      break;
    
  };

  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Error for address " + String(addr, HEX) + " is " + errStr);

  delay(5);

}

/**
 * @brief Blinks onboard LED amber (BLOCKING!)
 */
void debugFailureBlink(){

  rgbLED.limitBrightness(50);
  rgbLED.setStaticColor(rgbLED.AMBER);
  delay(100);
  rgbLED.setStaticColor(rgbLED.BLACK);
  delay(100);

}


void debugKillBlink(){

  rgbLED.limitBrightness(50);
  rgbLED.setStaticColor(rgbLED.CYAN);
  delay(50);
  rgbLED.setStaticColor(rgbLED.BLACK);
  delay(1000);

}

void debugKillCallback(){

  debugLogger.sendMessage("debugKillCallback", debugLogger.INFO, "Kill callback called. Entering safe state.");

  coreSafe();

  while(true){
    debugKillBlink();
  }

}

void debugRebootCallback(){

  rp2040.reboot();

}

void debugBootselCallback(){

  rp2040.rebootToBootloader();

}

void coreGetOrientation(){

  uint16_t leftDist = coreVehicleData.lidar[270];
  uint16_t rightDist = coreVehicleData.lidar[90];

  coreRoundDirCW = rightDist > leftDist;

  coreControlState = UNPARK;  // Move on to unparking after figuring out direction

  debugLogger.sendMessage("coreGetOrientation", debugLogger.INFO, "Set coreRoundDirCW to " + String(coreRoundDirCW ? "true" : "false"));  // not sure why +ing a set string and a ternary returns only the ternary eval, wrapping ternary in String() fixes it

}

void coreUnpark(){

  if(unparkAlgorithm.isFinished()){
    
    coreControlState = DRIVE_FROM_PI;
    return;

  }

  coreVehicleCommand = unparkAlgorithm.drive(coreVehicleData);

}

void coreDriveFromPi(){

  if(coreSerialCommand.instruction == RPI_RP2040_START_PARKING){
    
    coreControlState = PARK;
    return;

  }

  coreVehicleInstructionToPi = RP2040_RPI_START_OBSTACLE_NAVIGATION;
  coreVehicleCommand.targetSpeed = coreSerialCommand.targetSpeed;
  coreVehicleCommand.targetYaw = coreSerialCommand.targetYaw;

}

void corePark(){

  if(parkAlgorithm.isFinished()){

    coreControlState = SAFE;
    return;

  }

  coreVehicleCommand = unparkAlgorithm.drive(coreVehicleData);

}

void coreSafe(){

  motor.driveMotor(0, false);
  motor.disarmMotor();
  steering.steer(90);

}

void coreOpenRound(){

  coreVehicleCommand = openRoundAlgorithm.drive(coreVehicleData);

}

void coreRunStateMachine(){

  switch(coreControlState){

    case GET_ORIENTATION:
      coreGetOrientation();
      break;

    case UNPARK:
      coreUnpark();
      break;

    case DRIVE_FROM_PI:
      coreDriveFromPi();
      break;

    case PARK:
      corePark();
      break;

    case SAFE:
      coreSafe();
      break;

    default:
      coreSafe();   // Something has gone terribly wrong if we didn't match with any of the cases before us, so STOP!
      break;

  };

}

/*
enum CoreControlState{

  GET_ORIENTATION,
  UNPARK,
  DRIVE_FROM_PI,
  PARK,
  SAFE

}; */