#include <Adafruit_MotorShield.h>

// Pin definitions for the encoder
const int encoderPinA = 2;  // A phase of the encoder
const int encoderPinB = 3;  // B phase of the encoder

// Variables for tracking encoder position
volatile long encoderPosition = 0; // The position based on encoder
volatile int lastEncoded = 0;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Use port M3 for the motor
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);

// Motor control variables
int maxRPM = 30;  // Speed of the motor (0-255)

// Grove Base Shield v2 inputs
int button = 6; // D6 input

// Button state tracking
int buttonState;
bool isButtonPressed = false;
bool isButtonReleased = true;

// Encoder min, encoder max
int minRotation = 0;
int maxRotation = 300;

// Enum representing the state of the system
enum possibleStates {CLOSED, OPEN, OPENING, CLOSING};
possibleStates systemState;

void setup() {
  Serial.begin(9600);           // Set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test with encoder!");

  // Initialize the motor shield
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);  // Stop here if Motor Shield is not found
  }
  Serial.println("Motor Shield found.");

  // Set up encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH); // Enable pull-up resistors
  digitalWrite(encoderPinB, HIGH); // Enable pull-up resistors

  // Attach interrupts for the encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void loop() {
  checkInputs();

  Serial.print("Encoder position: ");
  Serial.println(encoderPosition);  // Print the encoder position

  if (encoderPosition <= minRotation && systemState != OPENING) {
    runMotor(0);
    systemState = CLOSED;
  }
  else if (encoderPosition >= maxRotation && systemState != CLOSING) {
    runMotor(0);
    systemState = OPEN;
  }

  if (isButtonPressed) {
    switch (systemState) {
      case OPEN:
        //
      case OPENING:
        systemState = CLOSING;
        Serial.println("Closing");
        runMotor(1);
        break;
      case CLOSED:
        //
      case CLOSING:
        systemState = OPENING;
        Serial.println("Opening");
        runMotor(-1);
        break;
    }
  }
}

void checkInputs() {
  if (analogRead(A0) < 100) {
    Serial.println("Light Sensor Triggered");
    myMotor->run(RELEASE);
    return true;
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
    myMotor->run(FORWARD);
  }
  else if (sign == -1) {
    myMotor->run(BACKWARD);
  }
  else {
    myMotor->run(RELEASE);
    return;
  }

  for (int i=0; i<maxRPM; i+=5) {
    myMotor->setSpeed(i);
    delay(1);
  }

  checkInputs();
  if (isButtonPressed == 1) {
    myMotor->run(RELEASE);
    return;
  }

  delay(10); // Wait before starting next cycle
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