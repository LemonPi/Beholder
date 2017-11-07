
char* buffer;

void setup() {
    buffer = (char*)malloc(20);
    // start serial port at 9600 bps:
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  }

unsigned int ts = 0;
float sensor1 = 0.0;
float sensor2 = 0.0;
float sensor3 = 0.0;
float sensor4 = 0.0;



void loop() {
    unsigned int* int_buffer = (unsigned int*)buffer;
    int_buffer[0] = ts;

    float* f_buffer = (float*)buffer + sizeof(ts);
    f_buffer[0] = sensor1;
    f_buffer[1] = sensor2;
    f_buffer[2] = sensor3;
    f_buffer[3] = sensor4;
    // Serial.write(buffer, 20);
    Serial.println(*((long*)buffer));
    // Serial.flush();
    delay(1000);
}