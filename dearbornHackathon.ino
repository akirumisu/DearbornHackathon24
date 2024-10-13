/* Simulate a car trunk closing */
// The hinge is a gear stepper motor from Seeed Studios
// The button to close/open the trunk is a Grove Buttons
// The light sensor to detect something in the way is a Grove Light Sensor

#include <Stepper.h>

// Grove Base Shield v2 Inputs
int button = 2;      //D2 input
int lightSensor = 3; //D3 input
// Grove Input Values
int buttonState;
float lightSensorValue;

// Boolean represented if the button was just pressed this loop
bool isButtonPressed;
bool isButtonReleased;

// Stepper Motor Using The Stepper Library
int stepsPerRotation = 32;
int minMotorSpeed = 30;
int maxMotorSpeed = 200;
int rpmIncrease = 1;
int stepsInSequence = 1;
Stepper motor = Stepper(stepsPerRotation, 8, 9, 10, 11);

int sign;
int rpm;

// enum representing the state of the system
enum possibleStates {CLOSED, OPEN, OPENING, CLOSING};
possibleStates systemState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //begin output

  // Set inputs
  pinMode(button, INPUT);
  pinMode(lightSensor, INPUT);

  motor.setSpeed(minMotorSpeed);

  // Initialize variables
  systemState = CLOSED;
  isButtonPressed = false;
  isButtonReleased = true;
  sign = 0;
  rpm = 0;

  delay(10);
}

void updateButtonAndLights() {
  buttonState = digitalRead(button); //get the button's value
  lightSensorValue = digitalRead(lightSensor); //get the lightSensor's value

  if (buttonState == 1 && isButtonPressed) {
    isButtonPressed = false;
  }
  else if (buttonState == 1 && isButtonReleased) {
    isButtonPressed = true;
    isButtonReleased = false;
    Serial.println("Button Pressed");
  }
  else if (buttonState == 0) {
    isButtonPressed = false;
    isButtonReleased = true;
  }

  if (lightSensorValue == 0) {
    Serial.println("Light Sensor Triggered");
    sign = 0;
  }
}

void rotateStepperMotor(int i) {
  if (sign == 0) {
    return;
  }

  motor.setSpeed(rpm);

  motor.step(i*sign);

  updateButtonAndLights();
  if (isButtonPressed) {
    sign = 0;
    return;
  }

  if (rpm < maxMotorSpeed) { rpm+=rpmIncrease; }

  rotateStepperMotor(stepsInSequence*sign);
}

void loop() {
  // put your main code here, to run repeatedly:
  updateButtonAndLights();

  if (isButtonPressed) {
    switch (systemState) {
      case OPEN:
        //
      case OPENING:
        systemState = OPENING;
        Serial.println("Opening");
        sign = 1;
        rpm = minMotorSpeed;
        rotateStepperMotor(stepsInSequence);
        break;
      case CLOSED:
        //
      case CLOSING:
        systemState = CLOSING;
        Serial.println("Closing");
        sign = -1;
        rpm = minMotorSpeed;
        rotateStepperMotor(stepsInSequence);
        break;
    }
  }
}
