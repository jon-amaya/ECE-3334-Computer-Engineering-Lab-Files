#include <Arduino.h>
#include <VNH3SP30.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <LSM303.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>
#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <ThingerTinyGSM.h>
//ultrasonic modules and distance to trigger stop
#define SONAR_NUM 3      
#define MAX_DISTANCE 12 


//ultrasonic sensor sweeping 
boolean sonarPing = false;
unsigned long currentPingMillis=0;
unsigned long previousPingMillis =0;
const long pingIntervaL = 200;

const char *cardinal;
double coursetoKey;
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


const int positiveSpeed =150

;
const int negativeSpeed =-150

;


TinyGPSPlus gps;
unsigned long distanceToDestination;
long double destinationLatitude = 33.584168;
long double destinationLongitude = -101.874182;
double currentLatitude;
double currentLongitude;
unsigned int satelliteCount = 0;
unsigned long lastUpdateTime = 0;




int increment = 0; //???

#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <ThingerTinyGSM.h>
//Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345).;

//ThingerIO Credentials
#define USERNAME "ttuRover"
#define DEVICE_ID "Arduino"
#define DEVICE_CREDENTIAL "raider"

// use your own APN config
#define APN_NAME "epc.tmobile.com"
#define APN_USER ""
#define APN_PSWD ""

// set your cad pin (optional)
#define CARD_PIN ""

ThingerTinyGSM thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL, Serial2);

LSM303 compass;
int *compassX = 0;
int *compassY = 0 ;
int *compassZ = 0;
const int compassOffset = 10;
float compassHeading;
int *compassHeadingOne;
int *compassHeadingTwo;
int *roverCycle;

float currentCompass;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(1, 2, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(3, 4, MAX_DISTANCE), 
  NewPing(5, 6, MAX_DISTANCE)
};

int getPing(){
  int average = 0;
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(150
    ); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
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
compass.read();


Serial.print("Compass Reading: ");



 /* 
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


  */
}

void roverSetup()
{
    Serial.println("Searching for Satellites "); 
      
  while (satelliteCount <= 4)                         // Wait until x number of satellites are acquired before starting main loop
  {                                  
    updateGPS();                                         // Update gps data
    satelliteCount = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired       
  }
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-610, -440, -539};
  compass.m_max = (LSM303::vector<int16_t>){+509, +526, +434};
    
   updateGPS();
   updateCompass();

}


void motorForward()
{
  motorFrontRight.setSpeed(150
  );
  motorFrontLeft.setSpeed(150
  );
  motorBackRight.setSpeed(150

  );
  motorBackLeft.setSpeed(150

  );
}

void motorBackward()
{
  motorFrontRight.setSpeed(-150
  );
  motorBackRight.setSpeed(-150
  );
  motorFrontLeft.setSpeed(-150
  );
  motorBackLeft.setSpeed(-150
  );
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
  motorFrontRight.setSpeed(-150

  );
  motorBackRight.setSpeed(-150

  );
  motorFrontLeft.setSpeed(150

  );
  motorBackLeft.setSpeed(150

  );
}

void motorright()
{
  motorFrontRight.setSpeed(150

  );
  motorBackRight.setSpeed(150

  );
  motorFrontLeft.setSpeed(-150

  );
  motorBackLeft.setSpeed(-150

  );
}

void steerRight()
{
  motorFrontRight.setSpeed(150
  );
  motorBackRight.setSpeed(150
  );
  motorFrontLeft.setSpeed(-150

  );
  motorBackLeft.setSpeed(-150

  );
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

void setup()
{
  Serial.begin(250000);
  Serial1.begin(9600); //Serial1 for GPS 
  Serial2.begin(57600);
  Dabble.begin(9600);      //Enter baudrate of your bluetooth.Connect bluetooth on Bluetooth port present on evive.

  //set pins for ultrasonics
  motorFrontRight.begin(motorFrontRightPWM, motorFrontRightEnableA, motorFrontRightEnableB); // Motor 1 object connected through specified pins
  motorFrontLeft.begin(motorFrontLeftPWM, motorFrontLeftEnableA, motorFrontLeftEnableB);
  motorBackRight.begin(motorBackRightPWM, motorBackRightEnableA, mototBackRightEnableB);
  motorBackLeft.begin(motorBackLeftPWM, motorBackLeftEnableA, motorBackLeftEnableB);
  roverSetup();

  // Serial for AT commands (can be higher with HW Serial, or even lower in SW Serial)
  // set APN
  thing.setAPN(APN_NAME, APN_USER, APN_PSWD);

  // resource input example (i.e, controlling a digitalPin);
  pinMode(LED_BUILTIN, OUTPUT);
  thing["led"] << digitalPin(LED_BUILTIN);
  thing["millis"] >> outputValue(millis());
  thing["compass"] >> outputValue(compassHeading);
  thing["compassGPS"] >> outputValue(currentCompass);
  thing["courseVariation"] >> outputValue(coursetoKey);
  thing["cardinal"] >> outputValue(cardinal);

  thing["location"] >> [](pson & out) { 
 
    out["lat"] = currentLatitude;
    out["lon"] = currentLongitude;
  };
  // more
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-610, -440, -539};
  compass.m_max = (LSM303::vector<int16_t>){+509, +526, +434};  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-610, -440, -539};
  compass.m_max = (LSM303::vector<int16_t>){+509, +526, +434};
}

void loop()
{  
  thing.handle();
  
    smartDelay(10);
    currentLongitude = gps.location.lng();
    currentLatitude = gps.location.lat();
    
  
  //motorBackward();
  //motorStop();
  updateGPS();
  compass.read();
  compassHeading = compass.heading();

  currentCompass=gps.course.deg();
  Serial.println(gps.course.deg()); // Course in degrees (double)

  
   coursetoKey =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      destinationLatitude, 
      destinationLongitude);
  //Serial.println(compassHeading);
 cardinal = TinyGPSPlus::cardinal(coursetoKey);
    Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
  Serial.print("KeyPressed: ");
  if (GamePad.isUpPressed())
  {
    motorForward();
    Serial.print("UP");
  }

  if (GamePad.isDownPressed())
  {
    motorBackward();
    Serial.print("DOWN");
  }

  if (GamePad.isLeftPressed())
  {
    motorleft();
    Serial.print("Left");
  }

  if (GamePad.isRightPressed())
  {
    motorright();
    Serial.print("Right");
  }

  if (GamePad.isSquarePressed())
  {
    motorStop();
    Serial.print("Square");
  }

  if (GamePad.isCirclePressed())
  {
    Serial.print("Circle");
  }

  if (GamePad.isCrossPressed())
  {
    Serial.print("Cross");
  }

  if (GamePad.isTrianglePressed())
  {
    Serial.print("Triangle");
  }

  if (GamePad.isStartPressed())
  {
    Serial.print("Start");
  }

  if (GamePad.isSelectPressed())
  {
    Serial.print("Select");
  }
  Serial.print('\t');
/*motorBackward();
Serial.println(gps.location.lat(),7);
compass.read();
*compassHeading = compass.heading();
*compassHeading+=5.47;
currentLatitude= gps.location.lat();
currentLongitude=gps.location.lng();
Serial.print("Compass Reading: ");
Serial.println(*compassHeading);
//distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng())
   gps.encode(ss.read());
    motorStop();

    if (millis() - lastUpdateTime >= 3000)
    {
      lastUpdateTime = millis();
      Serial.println(String(lastUpdateTime));

      unsigned long distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LATITUDE, TARGET_LONGITUE);
      double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LATITUDE, TARGET_LONGITUE);
      const char *cardinalDestination = TinyGPSPlus::cardinal(courseToDestination);
      int heading = (int)(360 + courseToDestination - gps.course.deg()) % 360;

      /*sensors_event_t event;
  mag.getEvent(&event);
  float Pi = 3.14159;
  // Calculate the angle of the vector y,x
  float heading = ((atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi)+5.88;
  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  delay(500);
*/
      Serial.print("Lat: ");
      Serial.println(gps.location.lat());
      Serial.println("Long: ");
      Serial.println(gps.location.lng());
      Serial.print("Course to destination: ");
      Serial.println(courseToDestination);
      Serial.println("Current Course: ");
      Serial.print(gps.course.deg());
      Serial.print("Cardinal Direction: ");
      Serial.println(cardinalDestination);
      Serial.print("Adjustment needed: ");
      Serial.println(heading);
      Serial.print("Speed in kmph: ");
      Serial.println(gps.speed.kmph());

      if (distanceToDestination <= 1)
      {
        Serial.println("Destination Reached");
        exit(1);                                                                                                                                                                                  
      }

      Serial.print("DISTANCE: ");
      Serial.print(distanceToDestination);
      Serial.println(" meters to go.");
      Serial.print("INSTRUCTION: ");

      if (gps.speed.kmph() < .1)
      {
        Serial.print("Head ");
        Serial.print(cardinalDestination);
        Serial.println(".");
        return;
      }

      if (heading >= 345 || heading < 15)
      {
        motorStop();
        delay(1000);
        moveForward();
      }
      else if (heading >= 315 && heading < 345)
      {
        Serial.println("Veer slightly to the left.");
        motorStop();
        delay(1000);
        moveLeftLow();
      }
      else if (heading >= 15 && heading < 45)
      {
        Serial.println("Veer slightly to the right.");
        motorStop();
        delay(1000);
        moveRightLow();
      }
      else if (heading >= 255 && heading < 315)
      {
        Serial.println("Turn to the left.");
        motorStop();
        delay(1000);
        moveLeft();
      }
      else if (heading >= 45 && heading < 105)
      {
        Serial.println("Turn to the right.");
        motorStop();
        delay(1000);
        moveRight();
      }
      else
      {
        Serial.println("Turn completely around.");
        motorStop();
        delay(1000);
        moveRight();
        delay(3000);
        motorStop();
      }
*/}