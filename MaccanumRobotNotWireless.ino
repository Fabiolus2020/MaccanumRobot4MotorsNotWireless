#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ezButton.h>

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
//int joyposX;
int joyposY;
int modeCount;

//defining joystick button followed by declaring variables and build mechanical state machine to keep count of joystick and have 3 modes
const int buttonPin  = 6;
int buttonState      = 0;     // current state of the button
int lastButtonState  = 0;     // previous state of the button
int modeState         = 0;     // remember current led state
int stateNum;

// Set initial motor speed at 0
int motorspeedModeOne = 0;
int motorspeedModeTwo = 0;

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

  pinMode(buttonPin, INPUT_PULLUP);  // initialize the button pin as a input

  pinMode(buttonSideWayRight, INPUT_PULLUP);
  pinMode(buttonSideWayLeft, INPUT_PULLUP);
  pinMode(buttonForwardRight, INPUT_PULLUP);
  pinMode(buttonForwardLeft, INPUT_PULLUP);
  pinMode(buttonBackwardRight, INPUT_PULLUP);
  pinMode(buttonBackwardLeft, INPUT_PULLUP);

  //The below I use for debounce issue using the EZbutton library, had to adjust for the switch button on thumbjoystick
  //  modeButton.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void loop() {
  uint8_t i;

  // read the pushbutton input pin
  buttonState = digitalRead(buttonPin);
  // check if the button is pressed or released
  // by comparing the buttonState to its previous state, this is the best I could get to being bulletproof and no debouning effect using joystick
  if (buttonState != lastButtonState) {

    // change the state of the led when someone pressed the button
    if (buttonState == 0) {
      stateNum++;
      if (stateNum > 2) stateNum = 0;
    }

    // remember the current state of the button
    lastButtonState = buttonState;
  }
  //reading the analog pins and assigning them to variables
  joyposX = analogRead(joyX);
  joyposY = analogRead(joyY);
  potValue = analogRead(Pot);

  if (joyposX < 485)//Going Forward
  {
    if (stateNum == 1)
    {
      myMotor1->run(FORWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(FORWARD);
      motorspeedModeOne = map(joyposX, 485, 0, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)
    {
      myMotor1->run(FORWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(FORWARD);
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  if (joyposX > 600)//Going Backward
  {
    if (stateNum == 1)
    {
      myMotor1->run(BACKWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(BACKWARD);
      motorspeedModeOne = map(joyposX, 600, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)//Going Backward
    {
      myMotor1->run(BACKWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(BACKWARD);
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  if (joyposY > 600)//Going left
  {
    if (stateNum == 1)
    {
      myMotor1->run(BACKWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(FORWARD);
      motorspeedModeOne = map(joyposY, 600, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)//Going left
    {
      myMotor1->run(BACKWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(FORWARD);
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  if (joyposY < 470)//Going right
  {
    if (stateNum == 1)
    {
      myMotor1->run(FORWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(BACKWARD);
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);
      // motorspeed2 = map(joyposY, 470, 0, 0, 255);
      // motorspeed3 = map(joyposY, 470, 0, 0, 255);
      //  motorspeed4 = map(joyposY, 470, 0, 0, 255);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)//Going right
    {
      myMotor1->run(FORWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(BACKWARD);
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      // motorspeed2 = map(joyposY, 470, 0, 0, 255);
      // motorspeed3 = map(joyposY, 470, 0, 0, 255);
      //  motorspeed4 = map(joyposY, 470, 0, 0, 255);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  if (digitalRead(buttonSideWayRight) == LOW) {
    if (stateNum == 1)
    {
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);
      myMotor1->run(BACKWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(BACKWARD);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)
    {
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->run(BACKWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(BACKWARD);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  if (digitalRead(buttonSideWayLeft) == LOW) {
    if (stateNum == 1)
    {
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);
      myMotor1->run(FORWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(FORWARD);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)
    {
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->run(FORWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(FORWARD);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  if (digitalRead(buttonForwardRight) == LOW) {
    if (stateNum == 1)
    {
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);
      myMotor1->run(FORWARD);
      myMotor4->run(FORWARD);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)
    {
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->run(FORWARD);
      myMotor4->run(FORWARD);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  else if (digitalRead(buttonForwardLeft) == LOW) {
    if (stateNum == 1)
    {
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)
    {
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      // myMotor4->run(BACKWARD);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
    }
  }
  else if (digitalRead(buttonBackwardRight) == LOW) {
    if (stateNum == 1)
    {
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor2->setSpeed(motorspeedModeOne);
      myMotor3->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)
    {
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor2->setSpeed(motorspeedModeTwo);
      myMotor3->setSpeed(motorspeedModeTwo);
    }
  }
  else if (digitalRead(buttonBackwardLeft) == LOW) {
    if (stateNum == 1)
    {
      motorspeedModeOne = map(joyposY, 470, 0, 0, 255);
      myMotor1->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor1->setSpeed(motorspeedModeOne);
      myMotor4->setSpeed(motorspeedModeOne);
    }
    else if (stateNum == 2)
    {
      motorspeedModeTwo = map(potValue, 0, 1023, 0, 255);
      myMotor1->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor1->setSpeed(motorspeedModeTwo);
      myMotor4->setSpeed(motorspeedModeTwo);
    }
  }
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
}
