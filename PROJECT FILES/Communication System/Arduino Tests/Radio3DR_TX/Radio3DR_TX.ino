#define BAUDRATE 57600

char CTSpin = 7;
int byteSent = 0;

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(CTSpin, OUTPUT);
  digitalWrite(CTSpin, HIGH);
}
void loop() {
//    if (byteSent <= 255)
//    {
////      Serial.print("Command Sent: ");
////      Serial.println(Serial.write(byteSent), BIN);
//      Serial.println(byteSent, DEC);
//    }
//   if (byteSent == 255)
//   {
////      Serial.print("Command Sent: ");
////      Serial.println(Serial.write(byteSent), BIN);
//      Serial.println(byteSent, DEC);
//      byteSent = 0;     
//   }
//   byteSent = byteSent + 1;

Serial.write(byteSent);
   Serial.print(byteSent);

   Serial.print(byteSent);
   Serial.write("\n");
   delay(1000);
   byteSent++;
}
