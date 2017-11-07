#include <SoftwareSerial.h>

constexpr long BL_RX = 10;
constexpr long BL_TX = 11;

SoftwareSerial EEBlue(BL_RX, BL_TX); // RX | TX
void setup()
{
  Serial.begin(9600);
  Serial.println("Reading from bluetooth");
  EEBlue.begin(9600);  
 
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