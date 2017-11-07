#include <SoftwareSerial.h>

constexpr long BL_RX = 10;
constexpr long BL_TX = 11;

SoftwareSerial EEBlue(BL_RX, BL_TX); // RX | TX
void setup()
{
   pinMode(9, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH);

  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  EEBlue.begin(38400);  // HC-05 default speed in AT command more
 
}
 
void loop()
{
 
  // Feed any data from bluetooth to Terminal.
  if (EEBlue.available()) {
    Serial.write(EEBlue.read());
  }
 
  // Feed all data from termial to bluetooth
  if (Serial.available()) {
    EEBlue.write(Serial.read());
  }
}