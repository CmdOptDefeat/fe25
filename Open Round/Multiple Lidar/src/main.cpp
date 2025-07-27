#include <Arduino.h>

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <TFLI2C.h>
#include <SoftwareSerial.h>

const byte rxPin = 9;
const byte txPin = 11;

// Set up a new SoftwareSerial object
SoftwareSerial mySerial (rxPin, txPin);
TFLI2C lidar;
Servo myservo;

// Pin definitions
const int encoder1 = 2;
const int encoder2 = 3;
const int startBtn = 5;
const int motor = 12;
const int motorDir = 10;

// About driving
int dir = 1;
int lastEncoded = 0; 
long encoderValue = 0;
float distance = 0.0;

// About turning
int turns = 0;
bool turning = false;
int turnDir = 0;           // 1 for clockwise, -1 for counterclockwise

// About IMU
Adafruit_BNO055 bno = Adafruit_BNO055(0x28);
float yaw;
int targetYaw = 0;
int startYaw = 0;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 200;

int pos = 90;    // variable to store the servo position  

// About Lidar
int threshold = 60;
int16_t front_lidarDist;
int16_t left_lidarDist;
int16_t right_lidarDist;
int16_t front_startDist = 50;
int16_t left_startDist = 40;
int16_t right_startDist = 40;
int stopDist = 5;

// About Gyro straight follower
int correction = 0;
float error = 0;
float totalError = 0;             // Used for integral control
float lateral_error = 0;          // Used for lateral correction
float width = 80;                // Width of track as seen by Lidars

// Function declarations
void updateEncoder();
void forward(int pwm);
void backward(int pwm);
void stop();
void steer(int angle);
void checkYaw();
void pStraight();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mySerial.begin(9600);
  Wire.begin(); 
  myservo.attach(9);
  myservo.write(90);

  pinMode(startBtn, INPUT_PULLUP);

  pinMode(motor, OUTPUT); 
  pinMode(motorDir, OUTPUT); 
  pinMode(encoder1, INPUT_PULLUP); 
  pinMode(encoder2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, CHANGE);
  
  while (true){
    int pinValue = digitalRead(startBtn);
    mySerial.println("Waiting");
    if(pinValue != 1){
      Serial.println("Started");
      break;
    }
  }
  delay(1500);

  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  bno.setExtCrystalUse(true);
  Serial.println("Initializing...");
  
  lidar.getData(left_startDist, 0x30);
  lidar.getData(right_startDist, 0x10);
  lidar.getData(front_startDist, 0x20);
  Serial.print("Start Distance: ");
  Serial.println(front_startDist);
  if (front_startDist > 140) stopDist = 0;
  else stopDist = 80;
  
  forward(240);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  sensors_event_t event; 
  bno.getEvent(&event);

  yaw = event.orientation.x;

  distance = encoderValue / 45;
  
  lidar.getData(left_lidarDist, 0x30);
  lidar.getData(right_lidarDist, 0x10);
  lidar.getData(front_lidarDist, 0x20); 
  
  currentMillis = millis();
  if (currentMillis - startMillis >= period){
    startMillis = currentMillis;
    
    //Serial.print(" Yaw:");
    mySerial.print(yaw);
    mySerial.print(',');
    //Serial.print("Front:");
    mySerial.print(front_lidarDist);
    mySerial.print(',');
    //Serial.print("Left:");
    mySerial.print(left_lidarDist);
    mySerial.print(',');
    //Serial.print("Right:");
    mySerial.println(right_lidarDist);
    
    /*Serial.print(" Target Yaw:");
    Serial.print(targetYaw);
    
    Serial.print(" Encoder Value:");
    Serial.print(encoderValue);

    Serial.print(" Distance:");
    Serial.print(distance);

    Serial.print(" Direction:");
    Serial.print(dir);

    Serial.print(" Turns:");
    Serial.println(turns);*/
  }

  checkYaw();
  
  if ((front_lidarDist < threshold) && (turning == false) && (turns == 0 or distance > 100)){
    steer(90);
    turning = true;
  }
  if ((turnDir == 0) && (left_lidarDist > left_startDist + 50)) turnDir = -1; // Turning to left
  else if ((turnDir == 0) && (right_lidarDist > right_startDist + 50)) turnDir = 1; // Turning to right
  
  if (turns == 12 && distance >= stopDist){ 
    stop();
    myservo.write(90);
    while (true){}
  }

  pStraight();
  delay(0.5);
}

// put function definitions here:
void updateEncoder(){
  if(dir == -1) encoderValue --;
  if(dir == 1) encoderValue ++;
}

void forward(int pwm){
  dir = 1;
  digitalWrite(motorDir,HIGH);
  analogWrite(motor,pwm);
}

void backward(int pwm){
  dir = -1;
  digitalWrite(motorDir,LOW);
  analogWrite(motor,pwm);
}

void stop(){
  digitalWrite(motor,0);
  digitalWrite(motorDir,LOW);
}

void steer(int angle){
  myservo.write(90 + turnDir*42);
  forward(225);
  startYaw = yaw;
  targetYaw = yaw + angle*turnDir;
  if (targetYaw > 360) targetYaw = targetYaw - 360;
  if (targetYaw > 75 && targetYaw < 105) targetYaw = 90;
  else if (targetYaw > 165 && targetYaw < 195) targetYaw = 180;
  else if (targetYaw > 255 && targetYaw < 285) targetYaw = 270;
  else if (targetYaw > 345 or targetYaw < 15) targetYaw = 0;
}

void checkYaw(){
  float difference = targetYaw - yaw;
  if (abs(difference) <= 6 && turning == true){
    myservo.write(90);
    forward(275);
    turning = false;
    encoderValue = 0;
    distance = 0;
    turns ++;
  }
}

void pStraight(){
  if(turning == false){
    correction = 0;
    lateral_error = 0;
    error = round(targetYaw - yaw);
    width = right_lidarDist + left_lidarDist;
    if (width > 80) width = 80;
    lateral_error = right_lidarDist - left_lidarDist; // Calculate lateral error based on lidar readings
    
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;                     // Normalize error to [-180, 180]
    totalError += error;                                            // Accumulate error for integral control
    
    if (error > 0) correction = error * 2.3 - totalError * 0.001; // correction to the right based on yaw
    else if (error < 0) correction = error * 2.2 - totalError * 0.001; // correction to the left based on yaw

    if (lateral_error > 0) correction += lateral_error * 0.55; // correction to left based on lateral error
    else if (lateral_error < 0) correction += lateral_error * 0.5; // correction to right based on lateral error

    if (correction > 45) correction = 45;
    else if (correction < -45) correction = -45;

    myservo.write(90 + correction);
    //Serial.print("Correction: ");
    //Serial.print(correction);
  }
}