// For single motor instance
#include <L298N.h>

// Pin definitions for the encoder
const int encoderPinA = 2;  // A phase of the encoder
const int encoderPinB = 3;  // B phase of the encoder

// Variables for tracking encoder position
volatile long encoderPosition = 0; // The position based on encoder
volatile int lastEncoded = 0;

// With Enable pin to control speed
L298N myMotor(5, 8, 9);

// Motor control variables
int maxRPM = 45;  // Speed of the motor (0-255)

// Grove Base Shield v2 inputs
int button = 6; // D6 input

// Button state tracking
int buttonState;
bool isButtonPressed = false;
bool isButtonReleased = true;

// Encoder min, encoder max
int minRotation = 0;
int maxRotation = 300;

int minLumens = 110;

const int ledPin1 = 10;
const int ledPin2 = 7;

// Enum representing the state of the system
enum possibleStates {CLOSED, OPEN, OPENING, CLOSING};
possibleStates systemState;

void setup() {
  Serial.begin(9600);

  myMotor.setSpeed(maxRPM);

  // Set up encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH); // Enable pull-up resistors
  digitalWrite(encoderPinB, HIGH); // Enable pull-up resistors

  // Attach interrupts for the encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
}

void loop() {
  Serial.print("Encoder position: ");
  Serial.println(encoderPosition); // Print the encoder position

  checkInputs();

  if (isButtonPressed) {
    switch (systemState) {
      case OPEN:
        //
      case OPENING:
        systemState = CLOSING;
        Serial.println("Closing");
        runMotor(-1);
        break;
      case CLOSED:
        //
      case CLOSING:
        systemState = OPENING;
        Serial.println("Opening");
        runMotor(1);
        break;
    }
  }
}

void checkInputs() {
  if (encoderPosition <= minRotation && systemState == CLOSING) {
    myMotor.stop();
  } else if (encoderPosition >= maxRotation && systemState == OPENING) {
    myMotor.stop();
  }

  if (analogRead(A3) < minLumens) {
    Serial.println("Light Sensor Triggered");
    if (systemState == OPENING) {
      myMotor.stop();
    }
    systemState = OPENING;
    myMotor.forward();
    return;
  }

  buttonState = digitalRead(button);

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

void runMotor(int sign) {
  if (sign == 1) {
    myMotor.forward();
  }
  else if (sign == -1) {
    myMotor.backward();
  }
  else {
    myMotor.stop();
    return;
  }

  for (int i=0; i<maxRPM; i++) {
    myMotor.setSpeed(i);
    delay(1);
  }

  checkInputs();
}

// Interrupt service routine to update encoder position
void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // Most significant bit
  int LSB = digitalRead(encoderPinB); // Least significant bit

  int encoded = (MSB << 1) | LSB;     // Convert the 2 pin value to an integer
  int sum = (lastEncoded << 2) | encoded; // Calculate the difference

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPosition++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPosition--;
  }

  lastEncoded = encoded;  // Store the previous state
}