#include <NewPing.h>

constexpr auto TRIGGER_PIN = 51;
constexpr auto ECHO_PIN = 50;

#define MAX_DISTANCE 200
constexpr auto MAX_DIST_MM = 200;

#define SONAR_NUM 3 // Number or sensors.

unsigned int cm[SONAR_NUM]; // Store ping distances.

NewPing sonar[SONAR_NUM] = { // Sensor object array.
    NewPing(53, 52, MAX_DIST_MM), // right
    NewPing(51, 50, MAX_DIST_MM), // front
    NewPing(49, 48, MAX_DIST_MM)  // left
};

void setup() { Serial.begin(9600); }

unsigned long lastPrintTime = 0;
unsigned long lastPingTime = 0;
void loop() {
  const auto currentTime = millis();
  if (currentTime - lastPingTime > 50) {
    lastPingTime = currentTime;
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      sonar[i].ping_timer(echoCheck);
    }
    // oneSensorCycle();
  }
  if (currentTime - lastPrintTime > 1000) {
    lastPrintTime = currentTime;
    oneSensorCycle();
  }
  // every second print distances for all
  // The rest of your code would go here.
}

void echoCheck() { // If ping echo, set distance to array.
	  for (uint8_t i = 0; i < SONAR_NUM; ++i) {
	    if (sonar[i].check_timer()) {
	      cm[i] = sonar[i].ping_result * 10 / US_ROUNDTRIP_CM;
	    }
	  }
}

void oneSensorCycle() { // Do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}