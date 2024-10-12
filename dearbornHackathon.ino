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
int stepsPerRotation = 4096;
const int STEPPER_MOTOR_INTERVAL = 1; //in ms
Stepper motor = Stepper(stepsPerRotation, 9, 8, 11, 10);

// enum representing the state of the system
enum possibleStates {CLOSED, OPEN, OPENING, CLOSING};
possibleStates systemState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //begin output

  // Set inputs
  pinMode(button, INPUT);
  pinMode(lightSensor, INPUT);

  // Initialize variables
  systemState = CLOSED;
  isButtonPressed = false;
  isButtonReleased = true;

  motor.setSpeed(120);

  delay(100);
}

// Checks to see if the button was just pressed this loop
// If the button was held down for > 1 cycle, isButtonPressed = false
void updateButton() {
  buttonState = digitalRead(button); //get the button's value

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
}

void rotateStepperMotor() {
  static uint32_t tThing;
  uint32_t tNow = millis();

  //if(not time to do thing)
  if((tNow - tThing) < STEPPER_MOTOR_INTERVAL) {
    return;
  }

  //save time for next check
  tThing = tNow;

  motor.step(1);
}

void open() {
  systemState = OPENING;
  rotateStepperMotor();
}

void close() {
  systemState = CLOSING;
  rotateStepperMotor();
}

void loop() {
  // put your main code here, to run repeatedly:
  lightSensorValue = digitalRead(lightSensor); //get the lightSensor's value

  updateButton();

  if (isButtonPressed) {
    switch (systemState) {
      case CLOSED:
        open();
        Serial.println("Opening");
        break;
      case OPEN:
        close();
        Serial.println("Closing");
        break;
      case OPENING:
        close();
        Serial.println("Closing");
        break;
      case CLOSING:
        open();
        Serial.println("Opening");
        break;
    }
  }
}
