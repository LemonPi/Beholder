
void setup() {
    // start serial port at 9600 bps:
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  }
struct sensor_struct {
  unsigned long ts;           // 4 bytes
  float d;                    // 4 bytes
  float dh;                   // 4 bytes
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

void loop() {
    packet.sensor_data.ts = millis();
    packet.sensor_data.d = 0;
    packet.sensor_data.dh = 0;
    packet.sensor_data.range = 500;
    packet.sensor_data.right = 100;
    packet.sensor_data.left = 100;
    Serial.write(0xA1);
    Serial.write(packet.byte_data, sizeof(sensor_struct));
    delay(100);
}
