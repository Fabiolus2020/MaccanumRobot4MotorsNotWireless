/*
  For use with the Adafruit Motor Shield v2
  ---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// Define Joystick Connections
#define joyY A0
#define joyX A1

// Define Joystick Values - Start at 512 (middle position) this might change depending on batteries and motor type.
int joyposX = 512;
int joyposY;

// Set initial motor speed at 0
int motorspeed1 = 0;
int motorspeed2 = 0;
int motorspeed3 = 0;
int motorspeed4 = 0;

// Set button for diagonal mode drive, upon pushing button specific direction is iniated.
#define buttonSideWayRight 0
#define buttonSideWayLeft 1
#define buttonForwardRight 2
#define buttonForwardLeft 3
#define buttonBackwardRight 4
#define buttonBackwardLeft 5

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  //AFMS.begin();  // create with the default frequency 1.6KHz
  AFMS.begin(4000);  // OR with a different frequency, say 4KHz

  pinMode(buttonSideWayRight, INPUT_PULLUP);
  pinMode(buttonSideWayLeft, INPUT_PULLUP);
  pinMode(buttonForwardRight, INPUT_PULLUP);
  pinMode(buttonForwardLeft, INPUT_PULLUP);
  pinMode(buttonBackwardRight, INPUT_PULLUP);
  pinMode(buttonBackwardLeft, INPUT_PULLUP);
}

void loop() {
  uint8_t i;
  uint8_t e;

  joyposX = analogRead(joyX);
  joyposY = analogRead(joyY);
  
  if (joyposX < 485)//Going Forward
  {
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(FORWARD);
    motorspeed1 = map(joyposX, 485, 0, 0, 255);
    motorspeed2 = map(joyposX, 485, 0, 0, 255);
    motorspeed3 = map(joyposX, 485, 0, 0, 255);
    motorspeed4 = map(joyposX, 485, 0, 0, 255);
    myMotor1->setSpeed(motorspeed1);
    myMotor2->setSpeed(motorspeed2);
    myMotor3->setSpeed(motorspeed3);
    myMotor4->setSpeed(motorspeed4);
  }
  else if (joyposX > 600)//Going Backward
  {
    myMotor1->run(BACKWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(BACKWARD);
    myMotor4->run(BACKWARD);
    motorspeed1 = map(joyposX, 600, 1023, 0, 255);
    motorspeed2 = map(joyposX, 600, 1023, 0, 255);
    motorspeed3 = map(joyposX, 600, 1023, 0, 255);
    motorspeed4 = map(joyposX, 600, 1023, 0, 255);
    myMotor1->setSpeed(motorspeed1);
    myMotor2->setSpeed(motorspeed2);
    myMotor3->setSpeed(motorspeed3);
    myMotor4->setSpeed(motorspeed4);
  }
  else if (joyposY > 600)//Going left
  {
    myMotor1->run(BACKWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(BACKWARD);
    myMotor4->run(FORWARD);
    motorspeed1 = map(joyposY, 600, 1023, 0, 255);
    motorspeed2 = map(joyposY, 600, 1023, 0, 255);
    motorspeed3 = map(joyposY, 600, 1023, 0, 255);
    motorspeed4 = map(joyposY, 600, 1023, 0, 255);
    myMotor1->setSpeed(motorspeed1);
    myMotor2->setSpeed(motorspeed2);
    myMotor3->setSpeed(motorspeed3);
    myMotor4->setSpeed(motorspeed4);
  }
  else if (joyposY < 470)//Going right
  {
    myMotor1->run(FORWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(BACKWARD);
    motorspeed1 = map(joyposY, 470, 0, 0, 255);
    motorspeed2 = map(joyposY, 470, 0, 0, 255);
    motorspeed3 = map(joyposY, 470, 0, 0, 255);
    motorspeed4 = map(joyposY, 470, 0, 0, 255);
    myMotor1->setSpeed(motorspeed1);
    myMotor2->setSpeed(motorspeed2);
    myMotor3->setSpeed(motorspeed3);
    myMotor4->setSpeed(motorspeed4);
  }
  else
  {
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
  }
  if (digitalRead(buttonSideWayRight) == LOW) {
    myMotor1->run(BACKWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(BACKWARD);
    myMotor1->setSpeed(150);
    myMotor2->setSpeed(150);
    myMotor3->setSpeed(150);
    myMotor4->setSpeed(150);
  }
  if (digitalRead(buttonSideWayLeft) == LOW) {
    myMotor1->run(FORWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(BACKWARD);
    myMotor4->run(FORWARD);
    myMotor1->setSpeed(150);
    myMotor2->setSpeed(150);
    myMotor3->setSpeed(150);
    myMotor4->setSpeed(150);
  }
  else if (digitalRead(buttonForwardRight) == LOW) {
    myMotor1->run(FORWARD);
    //myMotor2->run(BACKWARD);
    //myMotor3->run(BACKWARD);
    myMotor4->run(FORWARD);
    myMotor1->setSpeed(150);
    // myMotor2->setSpeed(255);
    //myMotor3->setSpeed(255);
    myMotor4->setSpeed(150);
  }
  else if (digitalRead(buttonForwardLeft) == LOW) {
    //myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
    //myMotor4->run(FORWARD);
    //myMotor1->setSpeed(150);
    myMotor2->setSpeed(150);
    myMotor3->setSpeed(150);
    // myMotor4->setSpeed(150);
  }
  else if (digitalRead(buttonBackwardRight) == LOW) {
    myMotor1->run(BACKWARD);
    //myMotor2->run(FORWARD);
    //myMotor3->run(FORWARD);
    myMotor4->run(BACKWARD);
    myMotor1->setSpeed(150);
    // myMotor2->setSpeed(255);
    //myMotor3->setSpeed(255);
    myMotor4->setSpeed(150);
  }
  else if (digitalRead(buttonBackwardLeft) == LOW) {
    // myMotor1->run(BACKWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(BACKWARD);
    //myMotor4->run(BACKWARD);
    //myMotor1->setSpeed(150);
    myMotor2->setSpeed(150);
    myMotor3->setSpeed(150);
    //myMotor4->setSpeed(150);
  }
  else
  {
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
  }
}
