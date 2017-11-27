#include <SoftwareSerial.h>

constexpr long BL_RX = 15;
constexpr long BL_TX = 14;

// constexpr long BL_RX = 2;
// constexpr long BL_TX = 4;

unsigned long txTimes[256] = {0};
unsigned long lastGroupTxTime = 0;

//SoftwareSerial EEBlue(BL_RX, BL_TX); // RX | TX
#define EEBlue Serial3

void setup() {
	pinMode(BL_RX, INPUT);
	pinMode(BL_TX, OUTPUT);
  Serial.begin(9600);
  Serial.println("Reading from bluetooth");
  EEBlue.begin(9600);
}

void loop() {

  // Feed any data from bluetooth to Terminal.
  if (EEBlue.available()) {
    const auto rxTime = millis();
    const auto c = EEBlue.read();

    // echo char to serial
    Serial.write(c);
    Serial.println();
    Serial.println(rxTime - txTimes[c]);

    if (lastGroupTxTime != 0) {
      Serial.print(rxTime - lastGroupTxTime);
      Serial.println(" (group)");
      lastGroupTxTime = 0;
    }
  }

  // Feed all data from termial to bluetooth
  if (Serial.available()) {
    const auto txTime = millis();
    const auto c = Serial.read();

    EEBlue.write(c);

    // start latency check on each char
    txTimes[c] = txTime;

    if (lastGroupTxTime == 0) {
      lastGroupTxTime = txTime;
    }
  }
}
