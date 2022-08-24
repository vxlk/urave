#include <Protocentral_MLX90632.h>
#include <Wire.h>

namespace temperature {
    
Protocentral_MLX90632 ReadSensor;

void setup()
{
  Serial.begin(9600);
  Serial.println("MLX90632 Read Data");
  Wire.begin();
  ReadSensor.begin();
}
float getBodyTemperature()
{
  float object_Temp = ReadSensor.getObjectTemp(); //Get the temperature of the object
  Serial.print("Body Temperature: ");
  Serial.print(object_Temp, 2);
  Serial.print(" C");

  Serial.println();

  return object_Temp;
}
} //namespace temperature
