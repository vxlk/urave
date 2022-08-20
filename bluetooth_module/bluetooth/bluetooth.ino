#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(3,2); // TX && RX

//Listening to:


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Bluetooth.begin(36400);
}

void loop() {
  if (Bluetooth.available()) {
    Bluetooth.write(Serial.read());
  }
}
