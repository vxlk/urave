#include <Protocentral_MLX90632.h>

#include "Protocentral_MLX90632.h"
#include <Wire.h>
Protocentral_MLX90632 ReadSensor;
Protocentral_MLX90632::status eStatus = Protocentral_MLX90632::status::SENSOR_GENERIC_ERROR;
void setup()
{
  Serial.begin(9600);
  Serial.println("MLX90632 Read Data");
  Wire.begin();
  //ReadSensor.setMode(MODE_CONTINUOUS);
  ReadSensor.begin(0x39, Wire, eStatus);
  ReadSensor._printDebug = false;
  //if (!ReadSensor.begin()) {
  //  Serial.print("Could not begin sensor reading");
  //}
}

/**
 * // Return values
    typedef enum
    {
      SENSOR_SUCCESS, <- 0
      SENSOR_ID_ERROR, <- 1
      SENSOR_I2C_ERROR, <- 2
      SENSOR_INTERNAL_ERROR, <- 3
      SENSOR_GENERIC_ERROR, <- 4
      SENSOR_TIMEOUT_ERROR <- 5
      //...
    } status;
 */

void getStatusUpdate() {
  Serial.print(ReadSensor.getStatus(eStatus));
  Serial.print("\n");
  Serial.print(eStatus);
  Serial.print("\n");
}

void loop()
{
  Serial.print("Polling in mode: ");
  Serial.print(ReadSensor.getMode(eStatus));
  Serial.print("\n");
  
  Serial.print("Initial Status: \n");
  getStatusUpdate();
  
  float tempInC = ReadSensor.getObjectTemp();
  Serial.print("Status After C Read: \n");
  getStatusUpdate();
  Serial.print("Body Temperature: ");
  Serial.print(tempInC, 2);
  Serial.print(" C");
  Serial.print("\n");
  
  float tempInF = ReadSensor.getObjectTempF(); //Get the temperature of the object
  Serial.print("Status: After F Read: \n");
  getStatusUpdate();
  Serial.print("Body Temperature: ");
  Serial.print(tempInF, 2);
  Serial.print(" F");
  Serial.print("\n");

  Serial.print("Debug Stream From Sensor: \n");
  if (ReadSensor._debugPort) {
    Serial.print(ReadSensor._debugPort->read()); 
  }

  /*
  float sensor_Temp = ReadSensor.getSensorTemp(); //Get the temperature of the sensor
  Serial.print(" Sensor temperature: ");
  Serial.print(sensor_Temp, 2);
  Serial.print(" C");
  Serial.println(); */
}
