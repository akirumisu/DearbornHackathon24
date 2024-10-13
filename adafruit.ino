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
int motorSpeed = 150;  // Speed of the motor (0-255)

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

  // Set initial motor speed and direction
  myMotor->setSpeed(motorSpeed);
  myMotor->run(FORWARD);
}

void loop() {
  Serial.print("Encoder position: ");
  Serial.println(encoderPosition);  // Print the encoder position

  // Run motor forward and reverse, adjusting speed
  runMotorWithAcceleration();
}

void runMotorWithAcceleration() {
  uint8_t i;

  Serial.println("Running motor forward...");
  myMotor->run(FORWARD);
  for (i = 0; i < 255; i++) {
    myMotor->setSpeed(i);
    delay(10);
  }
  for (i = 255; i != 0; i--) {
    myMotor->setSpeed(i);
    delay(10);
  }

  Serial.println("Running motor backward...");
  myMotor->run(BACKWARD);
  for (i = 0; i < 255; i++) {
    myMotor->setSpeed(i);
    delay(10);
  }
  for (i = 255; i != 0; i--) {
    myMotor->setSpeed(i);
    delay(10);
  }

  myMotor->run(RELEASE);  // Stop the motor
  delay(1000);            // Wait before starting next cycle
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
