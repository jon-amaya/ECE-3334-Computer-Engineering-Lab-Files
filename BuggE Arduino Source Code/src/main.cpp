#include <Arduino.h>
#include <VNH3SP30.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>


//ultrasonic modules and distance to trigger stop
#define SONAR_NUM 3      
#define MAX_DISTANCE 12 

//VNH2SP30 motor driver pin definitions
VNH3SP30 motorFrontRight;
VNH3SP30 motorFrontLeft;
VNH3SP30 motorBackRight;
VNH3SP30 motorBackLeft;

const int motorFrontRightPWM = 6;
const int motorFrontRightEnableA = 30;
const int motorFrontRightEnableB = 31;

const int motorFrontLeftPWM = 4;
const int motorFrontLeftEnableA = 26;
const int motorFrontLeftEnableB = 27;

const int motorBackRightPWM = 8;
const int motorBackRightEnableA = 34;
const int mototBackRightEnableB = 35;

const int motorBackLeftPWM = 2;
const int motorBackLeftEnableA = 22;
const int motorBackLeftEnableB = 23;


const int forwardSpeed =400;
const int reverseSpeed =-400;
const int turnSpeed = 300;



void motorForward()
{
  motorFrontRight.setSpeed(-400);
  motorFrontLeft.setSpeed(-400);
  motorBackRight.setSpeed(-400);
  motorBackLeft.setSpeed(-400);
}

void motorBackward()
{
  motorFrontRight.setSpeed(150);
  motorBackRight.setSpeed(150);
  motorFrontLeft.setSpeed(150);
  motorBackLeft.setSpeed(150);
}

void motorStop()
{
  motorFrontRight.setSpeed(0);
  motorFrontLeft.setSpeed(0);
  motorBackRight.setSpeed(0);
  motorBackLeft.setSpeed(0);
}

void motorleft()
{
  motorFrontRight.setSpeed(-400);
  motorBackRight.setSpeed(-400);
  motorFrontLeft.setSpeed(400);
  motorBackLeft.setSpeed(400);
}

void motorright()
{
  motorFrontRight.setSpeed(400);
  motorBackRight.setSpeed(400);
  motorFrontLeft.setSpeed(-400);
  motorBackLeft.setSpeed(-400);
}

void steerRight()
{
  motorFrontRight.setSpeed(200);
  motorBackRight.setSpeed(200);
  motorFrontLeft.setSpeed(-400);
  motorBackLeft.setSpeed(-400);
}

void setup()
{
  motorFrontRight.begin(motorFrontRightPWM, motorFrontRightEnableA, motorFrontRightEnableB); // Motor 1 object connected through specified pins
  motorFrontLeft.begin(motorFrontLeftPWM, motorFrontLeftEnableA, motorFrontLeftEnableB);
  motorBackRight.begin(motorBackRightPWM, motorBackRightEnableA, mototBackRightEnableB);
  motorBackLeft.begin(motorBackLeftPWM, motorBackLeftEnableA, motorBackLeftEnableB);
  Serial.begin(115200);
}

void loop()
{

}