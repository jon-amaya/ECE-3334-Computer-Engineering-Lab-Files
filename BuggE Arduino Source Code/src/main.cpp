#include <Arduino.h>

#include <VNH3SP30.h>

VNH3SP30 MotorFR;    // define control object for 1 motor
VNH3SP30 MotorFL;
VNH3SP30 MotorBR;
VNH3SP30 MotorBL;

// motor pins
#define MFR_PWM 6    // pwm pin motor
#define MFR_INA 30   // control pin INA
#define MFR_INB 31    // control pin INB

#define MFL_PWM 4
#define MFL_INA 26
#define MFL_INB 27

#define MBR_PWM 8
#define MBR_INA 34
#define MBR_INB 35

#define MBL_PWM 2
#define MBL_INA 22
#define MBL_INB 23

void forward() {
  MotorFR.setSpeed(-400);
  MotorFL.setSpeed(-400);
  MotorBR.setSpeed(-400);
  MotorBL.setSpeed(-400);
}

void backward() {
  MotorFR.setSpeed(150);
  MotorBR.setSpeed(150);
  MotorFL.setSpeed(150);
  MotorBL.setSpeed(150);
}

void stopping() {
  MotorFR.setSpeed(0);
  MotorFL.setSpeed(0);
  MotorBR.setSpeed(0);
  MotorBL.setSpeed(0);
}

void left() {
  MotorFR.setSpeed(-400);
  MotorBR.setSpeed(-400);
  MotorFL.setSpeed(400);
  MotorBL.setSpeed(400);
}

void right() {
  MotorFR.setSpeed(400);
  MotorBR.setSpeed(400);
  MotorFL.setSpeed(-400);
  MotorBL.setSpeed(-400);
}

void steerRight() {
  MotorFR.setSpeed(200);
  MotorBR.setSpeed(200);
  MotorFL.setSpeed(-400);
  MotorBL.setSpeed(-400);
}

void setup() {
  MotorFR.begin(MFR_PWM, MFR_INA, MFR_INB);    // Motor 1 object connected through specified pins 
  MotorFL.begin(MFL_PWM, MFL_INA, MFL_INB);
  MotorBR.begin(MBR_PWM, MBR_INA, MBR_INB);
  MotorBL.begin(MBL_PWM, MBL_INA, MBL_INB);
  Serial.begin(115200);
}

void loop() {
  forward();
  delay(8000);
  stopping();
  delay(250);
  //right();
  steerRight();
  delay(150);
  stopping();
  delay(250);

}