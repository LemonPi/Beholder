#include <SoftwareSerial.h>

#include "serialize.h"

constexpr long BL_RX = 5;
constexpr long BL_TX = 6;

unsigned long txTimes[256] = {0};
unsigned long lastGroupTxTime = 0;

SoftwareSerial EEBlue(BL_RX, BL_TX); // RX | TX
void setup() {
  Serial.begin(9600);
  Serial.println("Bluetooth ready");
  EEBlue.begin(9600);
}

struct pose_update {
  float d;                    // 4 bytes
  float dh;                   // 4 bytes
};


struct sensor_struct {
  uint32_t ts;                // 4 bytes
  uint32_t state;             // 4 bytes
  pose_update dx;             // 8 bytes
  uint32_t range;        // 4 bytes
  uint32_t right;        // 4 bytes
  uint32_t left;         // 4 bytes

  static constexpr auto PACKET_SIZE = sizeof(uint32_t)*5 + sizeof(float)*2;
  void serializeObj(uint8_t* buffer, uint8_t& index) {
    serialize(buffer, index, ts);
    serialize(buffer, index, state);
    serialize(buffer, index, dx.d);
    serialize(buffer, index, dx.dh);
    serialize(buffer, index, range);
    serialize(buffer, index, right);
    serialize(buffer, index, left);
  }
};

union sensor_packet {
  sensor_struct sensor_data;
  byte byte_data[sensor_struct::PACKET_SIZE];

};

sensor_packet packet;
float x;
uint32_t pack_num = 1;
unsigned int last_packet = millis();

byte serialBuffer[sensor_struct::PACKET_SIZE];

void loop() {
    unsigned long long now = millis();
    if(now - last_packet >= 100) {
      packet.sensor_data.ts = pack_num;
      packet.sensor_data.state = 0;
      packet.sensor_data.dx.d = 0;
      packet.sensor_data.dx.dh = 0;
      packet.sensor_data.range = 500;
      packet.sensor_data.right = 100;
      packet.sensor_data.left = 100;
      EEBlue.write(0xA1);

//      uint8_t index = 0;
//      packet.sensor_data.serializeObj(serialBuffer, index);
      
      // int sent = EEBlue.write(packet.byte_data, PACKET_SIZE);
//      if (index != sensor_struct::PACKET_SIZE) {
//        Serial.println("WHATTTTT");
//        Serial.print(index);
//        Serial.print(" vs ");
//        Serial.println(sensor_struct::PACKET_SIZE);
//      }
//      index = 0;
//      uint32_t tester = 5;
//      serialize(serialBuffer, index, tester);
//       const auto sentBytes = EEBlue.write(serialBuffer, index);
//      Serial.print(serialBuffer[0]);
//      Serial.print(serialBuffer[1]);
//      Serial.print(serialBuffer[2]);
//      Serial.print(serialBuffer[3]);
//      Serial.print(serialBuffer[4]);
//      Serial.print(serialBuffer[5]);
//      Serial.print(serialBuffer[6]);
//      Serial.print(serialBuffer[7]);
//      for (uint8_t i = 0; i < index; ++i) {
//        Serial.print(serialBuffer[i]);
//        Serial.print(" ");
//      }
//      Serial.println();
//      
//      pack_num++;
      last_packet = now;
    }
//    if (EEBlue.available()) {
//      const auto rxTime = millis();
//      const auto c = EEBlue.read();
//      Serial.println(rxTime - last_packet);
//    }
}
