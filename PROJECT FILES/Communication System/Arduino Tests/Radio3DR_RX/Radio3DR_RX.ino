#define BAUDRATE 57600
int incomingByte = 0;

void setup() {
  Serial.begin(BAUDRATE);
}
void loop() {
  if (Serial.available()) {
    Serial.print("Data Received: ");
    Serial.println(Serial.read(), DEC);
    delay(500);
  }
}
