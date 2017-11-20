
void setup() {
    // start serial port at 9600 bps:
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  }
struct sensor_struct {
  unsigned long ts;   // 4 bytes
  float range;        // 4 bytes
  float right;        // 4 bytes
  float left;         // 4 bytes
  float colour;       // 4 bytes
};

union sensor_packet {
  sensor_struct sensor_data;
  byte byte_data[sizeof(sensor_struct)];
};

sensor_packet packet;
float x;

void loop() {
    packet.sensor_data.ts = millis();
    packet.sensor_data.range = 0.5;
    packet.sensor_data.right = 0.15;
    packet.sensor_data.left = 0.15;
    packet.sensor_data.colour = 1.0;
    Serial.write(0xA1);
    Serial.write(packet.byte_data, sizeof(sensor_struct));
    delay(100);
}