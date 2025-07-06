#include <Arduino.h>
#include "pindefs.h"
#include "SensorManager/SensorManager.h"
#include "CommManager/CommManager.h"
#include "drivers/MPU6050.h"
#include "drivers/BMP085.h"
#include "drivers/TCS34725.h"
#include "drivers/TFLuna.h"
#include <Servo.h>

Servo lidarServo;
Servo steeringServo;

DriverMPU6050 sensorMPU6050(&Wire1);
DriverBMP085 sensorBMP085(&Wire1);
DriverTCS34725 sensorTCS34725;
DriverTFLuna sensorTFLuna;
SensorManager sensorManager;


void driveFromRX();
void driveMotor(int throttle);
void updateEncoder();

volatile int encoderPos;
volatile int prevEncoderPos;

void setup(){

  Serial.begin();

  lidarServo.attach(PIN_LIDAR_SERVO);
  steeringServo.attach(PIN_STEERING_SERVO);

  Wire1.setSCL(PIN_I2C1_SCL);
  Wire1.setSDA(PIN_I2C1_SDA);
  Wire.setSCL(PIN_I2C0_SCL);
  Wire.setSDA(PIN_I2C0_SDA);

  sensorManager.addSensor(&sensorMPU6050);
  sensorManager.addSensor(&sensorBMP085);
  sensorManager.addSensor(&sensorTCS34725);
  sensorManager.addSensor(&sensorTFLuna);
  sensorManager.initializeSensors();

  pinMode(PIN_RX_CH1, INPUT);
  pinMode(PIN_RX_CH2, INPUT);

  pinMode(PIN_TB6612_BIN1, OUTPUT);
  pinMode(PIN_TB6612_BIN2, OUTPUT);
  pinMode(PIN_TB6612_PWMB, OUTPUT);
  pinMode(PIN_TB6612_STBY, OUTPUT);

  pinMode(PIN_MOTOR_ENCA, INPUT);
  pinMode(PIN_MOTOR_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCA), updateEncoder, CHANGE);

}

void loop(){

  driveFromRX();
  Serial.println("Encoder Position: " + String(encoderPos));

}

void driveFromRX(){

  int16_t steering = pulseIn(PIN_RX_CH1, HIGH);
  int16_t throttle = pulseIn(PIN_RX_CH2, HIGH);

  steering = constrain(map(steering, 1000, 2000, 0, 180), 0, 180);
  throttle = constrain(map(throttle, 1000, 2000, -100, 100), -100, 100);

  steeringServo.write(steering);
  driveMotor(throttle);

  Serial.println("Steering: " + String(steering) + ", Throttle: " + String(throttle));

}

void driveMotor(int throttle){

  digitalWrite(PIN_TB6612_STBY, HIGH); 

  if(throttle > 0){

    digitalWrite(PIN_TB6612_BIN1, HIGH);
    digitalWrite(PIN_TB6612_BIN2, LOW);
    analogWrite(PIN_TB6612_PWMB, map(throttle, 0, 1024, 0, 255));

  }
  else{

    digitalWrite(PIN_TB6612_BIN1, LOW);
    digitalWrite(PIN_TB6612_BIN2, HIGH);
    analogWrite(PIN_TB6612_PWMB, map(-throttle, 0, 1024, 0, 255));

  }

}

void updateEncoder(){

  int MSB = digitalRead(PIN_MOTOR_ENCA); 
  int LSB = digitalRead(PIN_MOTOR_ENCB); 

  int encoded = (MSB << 1) | LSB;
  int sum = (prevEncoderPos << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPos--;

  prevEncoderPos = encoded;

}