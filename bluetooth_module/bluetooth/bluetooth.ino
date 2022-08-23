#include <BluetoothSerial.h>

//https://randomnerdtutorials.com/esp32-bluetooth-classic-arduino-ide/
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


BluetoothSerial btSerial;


void setup() {
  Serial.begin(115200); // this is most likely the wrong baud rate
  btSerial.begin("U-Rave-Bluetooth");
}

void loop() {
    btSerial.write(255);
}
