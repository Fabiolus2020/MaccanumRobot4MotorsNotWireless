/*
  This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
  It won't work with v1.x motor shields! Only for the v2's with built in PWM
  control

  For use with the Adafruit Motor Shield v2
  ---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ezButton.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// Define Joystick Connections
#define joyY A0
#define joyX A1

// Define Joystick Values - Start at 512 (middle position) this might change depending on batteries and motor type.
int joyposX = 512;
//int joyposX;
int joyposY;
int modeCount;

//defining joystick button using ezbutton library
ezButton modeButton(6);  // create ezButton object that attach to pin 7;

// Set initial motor speed at 0
int motorspeedModeOne = 0;
int motorspeedModeTwo = 0;
//int motorspeed2 = 0;
//int motorspeed3 = 0;
//int motorspeed4 = 0;

// Set button for diagonal mode drive, upon pushing button specific direction is iniated.
#define buttonSideWayRight 0
#define buttonSideWayLeft 1
#define buttonForwardRight 2
#define buttonForwardLeft 3
#define buttonBackwardRight 4
#define buttonBackwardLeft 5

//Define potentiometer
#define Pot A2
int potValue = 0;

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

  //The below I use for debounce issue using the EZbutton library, had to adjust for the switch button on thumbjoystick
  modeButton.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void loop() {
  uint8_t i;
  //uint8_t e;

  modeButton.loop(); // MUST call the loop() function first
  //from EZbutton library to keep count of buttons count. Works great!
  unsigned long modeButtonCount = modeButton.getCount();
  Serial.println(modeCount);
  //for some reason released works better then IsPressed
  if (modeButton.isReleased()) {

    if (modeButtonCount >= 3) modeButton.resetCount();

    for (i = 1; i < modeButtonCount; i++) {
      Serial.println(modeCount);
    }
    modeCount = modeButtonCount;
  }

  //reading the analog pins and assigning them to variables
  joyposX = analogRead(joyX);
  joyposY = analogRead(joyY);
  potValue = analogRead(Pot);


  //Serial.println(joyposX);
  // Serial.print("PosX");
  //  Serial.println(joyposX);
  // Serial.print("PosY");
  // Serial.println(potValue);

  //mapping the potentiometer for mode two, this will be applied to both joystick and buttons and control PWM 0-255
  // motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
  //Serial.println(motorspeedModeTwo);




  if (joyposX < 485)//Going Forward
  {
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(FORWARD);

    if (modeCount == 1)
    {
      motorspeedModeOne = map(joyposX, 485, 0, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (modeCount == 2)
    {
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
    else {
      myMotor1->setSpeed(0);
      myMotor2->setSpeed(0);
      myMotor3->setSpeed(0);
      myMotor4->setSpeed(0);
    }
  }


  if (joyposX > 600)//Going Backward
  {
    myMotor1->run(BACKWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(BACKWARD);
    myMotor4->run(BACKWARD);
    if (modeCount == 1)
    {
      motorspeedModeOne = map(joyposX, 600, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }

  else if (modeCount == 2)
  {
    motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
    myMotor1->setSpeed(motorspeedModeTwo);
    myMotor2->setSpeed(motorspeedModeTwo);
    myMotor3->setSpeed(motorspeedModeTwo);
    myMotor4->setSpeed(motorspeedModeTwo);
  }
      else {
      myMotor1->setSpeed(0);
      myMotor2->setSpeed(0);
      myMotor3->setSpeed(0);
      myMotor4->setSpeed(0);
    }
  }


  if (joyposY > 600)//Going left
  {
    myMotor1->run(BACKWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(BACKWARD);
    myMotor4->run(FORWARD);
    if (modeCount == 1)
    {
      motorspeedModeOne = map(joyposY, 600, 1023, 0, 255);
      // motorspeed2 = map(joyposY, 600, 1023, 0, 255);
      // motorspeed3 = map(joyposY, 600, 1023, 0, 255);
      // motorspeed4 = map(joyposY, 600, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
 
  else if (modeCount == 2)
  {
    motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
    myMotor1->setSpeed(motorspeedModeTwo);
    myMotor2->setSpeed(motorspeedModeTwo);
    myMotor3->setSpeed(motorspeedModeTwo);
    myMotor4->setSpeed(motorspeedModeTwo);
  }
      else {
      myMotor1->setSpeed(0);
      myMotor2->setSpeed(0);
      myMotor3->setSpeed(0);
      myMotor4->setSpeed(0);
    }
 }
  if (joyposY < 470)//Going right
  {
    myMotor1->run(FORWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(BACKWARD);
    if (modeCount == 1)
    {
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);

      // motorspeed2 = map(joyposY, 470, 0, 0, 255);
      // motorspeed3 = map(joyposY, 470, 0, 0, 255);
      //  motorspeed4 = map(joyposY, 470, 0, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }

  else if (modeCount == 2)
  {
    motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
    myMotor1->setSpeed(motorspeedModeTwo);
    myMotor2->setSpeed(motorspeedModeTwo);
    myMotor3->setSpeed(motorspeedModeTwo);
    myMotor4->setSpeed(motorspeedModeTwo);
  }
      else {
      myMotor1->setSpeed(0);
      myMotor2->setSpeed(0);
      myMotor3->setSpeed(0);
      myMotor4->setSpeed(0);
    }
  }
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);

}
