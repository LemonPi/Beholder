#include <SoftwareSerial.h>

constexpr long BL_RX = 10;
constexpr long BL_TX = 11;

unsigned long txTimes[256] = {0};
unsigned long lastGroupTxTime = 0;

SoftwareSerial EEBlue(BL_RX, BL_TX); // RX | TX
void setup() {
  Serial.begin(9600);
  Serial.println("Bluetooth ready");
  EEBlue.begin(9600);
  EEBlue.write(0xA2);
}

struct pose_update {
  float d;                    // 4 bytes
  float dh;                   // 4 bytes  
};

struct sensor_struct {
  uint32_t ts;                // 4 bytes
  uint32_t state;             // 4 bytes
  pose_update dx;             // 8 bytes
  unsigned long range;        // 4 bytes
  unsigned long right;        // 4 bytes
  unsigned long left;         // 4 bytes
};

union sensor_packet {
  sensor_struct sensor_data;
  byte byte_data[sizeof(sensor_struct)];
};

sensor_packet packet;
float x;
uint32_t pack_num = 1;
unsigned int last_packet = millis();


void loop() {
    unsigned long long now = millis();
    if(now - last_packet >= 1000) {
      packet.sensor_data.ts = pack_num;
      packet.sensor_data.state = 0;
      packet.sensor_data.dx.d = 0;
      packet.sensor_data.dx.dh = 0;
      packet.sensor_data.range = 500;
      packet.sensor_data.right = 100;
      packet.sensor_data.left = 100;
      EEBlue.write(0xA1);
      
      int sent = EEBlue.write(packet.byte_data, sizeof(sensor_struct));      
      pack_num++;
      last_packet = now;
    }
    if (EEBlue.available()) {
      const auto rxTime = millis();
      const auto c = EEBlue.read();
      Serial.println(rxTime - last_packet);
    }
}
