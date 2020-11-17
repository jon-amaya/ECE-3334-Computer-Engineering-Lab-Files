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

//ultrasonic sensor sweeping 
boolean sonarPing = false;
unsigned long currentPingMillis=0;
unsigned long previousPingMillis =0;
const long pingIntervaL = 200;


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


const int positiveSpeed =400;
const int negativeSpeed =-400;


TinyGPSPlus gps;
unsigned long distanceToDestination;
long double destinationLatitude;
long double destinationLongitude;
unsigned int satelliteCount = 0;
unsigned long lastUpdateTime = 0;

int increment = 0; //???


//compass axis
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345).;


int *compassX = 0;
int *compassY = 0 ;
int *compassZ = 0;
const int compassOffset = 10;
int *compassHeading;
int *compassHeadingOne;
int *compassHeadingTwo;
int *roverCycle;



NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(1, 2, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(3, 4, MAX_DISTANCE), 
  NewPing(5, 6, MAX_DISTANCE)
};

int getPing(){
  int average = 0;
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar[i].ping_cm());
    Serial.print("cm ");
    average+= sonar[i].ping_in();
  }
  return average/SONAR_NUM;

}

void updateGPS(){
  while (Serial1.available()>0){
    gps.encode(Serial1.read());
  }
}

void updateCompass(){
  
  sensors_event_t event;
  mag.getEvent(&event);
  
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x));
  // Normalize to 0-360
  if (heading < 0){ 
    heading +=2*M_PI;
    *compassHeading = int(heading*180/M_PI);
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  Serial.println(*compassHeading);
}

void roverSetup()
{
    Serial.println("Searching for Satellites "); 
      
  while (satelliteCount <= 4)                         // Wait until x number of satellites are acquired before starting main loop
  {                                  
    updateGPS();                                         // Update gps data
    satelliteCount = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired       
  }
    /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
    
   updateGPS();
   updateCompass();

}


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
  Serial.begin(115200);
  Serial1.begin(9600); //Serial1 for GPS
  //set pins for ultrasonics
  motorFrontRight.begin(motorFrontRightPWM, motorFrontRightEnableA, motorFrontRightEnableB); // Motor 1 object connected through specified pins
  motorFrontLeft.begin(motorFrontLeftPWM, motorFrontLeftEnableA, motorFrontLeftEnableB);
  motorBackRight.begin(motorBackRightPWM, motorBackRightEnableA, mototBackRightEnableB);
  motorBackLeft.begin(motorBackLeftPWM, motorBackLeftEnableA, motorBackLeftEnableB);
  roverSetup();
}

void loop()
{
updateCompass();
updateGPS();

distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng())
}