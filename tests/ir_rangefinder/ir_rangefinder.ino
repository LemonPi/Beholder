#include <Servo.h>

constexpr auto IR_RANGEFINDER = A15;

// Arm
constexpr auto ARM_PIN = 5;
constexpr auto CLAW_PIN = 4;

Servo _armServo;  // create servo object to control arm
int _armPosition;
Servo _clawServo;  // create servo object to control claw
int _clawPosition;

// Servo positions.
static constexpr int ARM_DOWN = 170;
static constexpr int ARM_SEARCH_POSITION = ARM_DOWN - 5;
static constexpr int ARM_UP = 75 + 50; // Accomodating for IR rangefinder.
static constexpr int CLAW_CLOSED = 35;
static constexpr int CLAW_OPENED = 120;

// the setup routine runs once when you press reset:
void setup() {
  _clawServo.attach(CLAW_PIN);
  _armServo.attach(ARM_PIN);
  _armPosition = ARM_SEARCH_POSITION;
  _clawPosition = CLAW_OPENED;
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
}

// the loop routine runs over and over again forever:
void loop() {
  _armServo.write(_armPosition);
  _clawServo.write(_clawPosition);

  float sensorValue = float(analogRead(A15));
  // print out the value you read:
  Serial.println(sensorValue);
  delay(1000);        // delay in between reads for stability
}
