

// UART2 BONES
// If you are getting an error finding this,
// it comes with the esp32 package.
// C:\Users\small\Documents\ArduinoData\packages\esp32\hardware\esp32\2.0.4\cores\esp32
// on my disk
#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // use UART2
HardwareSerial SerialPort_Debug(0); // Able to print to this one

/**
 * Once the collection on command is received,
 * this data structure be polled once every 64 sampling points (1.28s)
 */
typedef struct
{
uint8_t header;
int8_t acdata[64]; //Heartbeat data
uint8_t heartrate; //heart rate
uint8_t spo2; //blood oxygen
uint8_t bk; //Micro loop
uint8_t rsv[8]; //Retain data
} Real_Time_Packet;

//const size_t BUFFER_SIZE = 76;
const size_t BUFFER_SIZE = 85; // I am leaving some extra room in the buffer...
uint8_t buffer[BUFFER_SIZE];

void readSerialPort() 
{
  if (SerialPort.available() > 0) {
    SerialPort.read(buffer, BUFFER_SIZE);

    // find the start bit
    auto startBit = 0xFF;
    for (auto i = 0; i < BUFFER_SIZE; ++i) {
      if (buffer[i] == startBit) {
        // it can ONLY have 1 startBit or else it isn't the right packet
        for (auto x = i+1; x < 76; ++x) {
          if (buffer[x] == startBit) {
            return;
          }
        }
        auto offset = i;
        auto positionOfHeartRate = 65;
        
        if (positionOfHeartRate + offset < BUFFER_SIZE) {
          SerialPort_Debug.println("Heart Rate:");
          SerialPort_Debug.println(buffer[positionOfHeartRate + offset]);
          
          SerialPort_Debug.println("All info:");
          for (auto i = offset + 64; i < 76 + offset; ++i) {
            SerialPort_Debug.print(buffer[i]);
            SerialPort_Debug.print(" ");
          }
          SerialPort_Debug.println("");
          break;
        }
      }
    }
  }
}

void setup()  
{
  /** ---------------------- Debug functionality ---------------------- **/
  /**
   typedef enum {
    UART_BREAK_ERROR,
    UART_BUFFER_FULL_ERROR,
    UART_FIFO_OVF_ERROR,
    UART_FRAME_ERROR,
    UART_PARITY_ERROR
} hardwareSerial_error_t;
   */
 // SerialPort.onReceive([&](){ readSerialPort(); });
 // SerialPort.onReceiveError([&](hardwareSerial_error_t errorCode){ 
 //   SerialPort_Debug.println("Got an Error:");
 //   SerialPort_Debug.println(errorCode); 
 // }); 
 // SerialPort.setDebugOutput(true);
 /** ----------------------------------------------------------------- **/

  SerialPort.setRxBufferSize(BUFFER_SIZE);
  SerialPort.setTxBufferSize(BUFFER_SIZE);
  
  SerialPort.begin(38400, SERIAL_8N1, 16, 17);
  SerialPort_Debug.begin(38400);

  if (SerialPort.availableForWrite()) {
    /**
   * Capture ON 0x8A
   * Collection off 0x88
   * 
   * Physical examination open 0x8E
   * Physical examination off 0x8C
   * 
   * hibernate on 0x98
   * hibernate off 0x00
   */ 
    SerialPort.write(0x00);
    SerialPort.write(0x8A);
  }
} 
void loop()  
{
  readSerialPort();
}
