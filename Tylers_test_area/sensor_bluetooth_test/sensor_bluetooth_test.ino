#include "arduino-timer.h"
#include "bio.h"
#include "bluetooth.h"
#include "temperature.h"

void setup() {
  Timer <1> oneSecondTimerWithOneCallback;
  const auto ONE_SECOND = 1000;
  oneSecondTimerWithOneCallback.every(ONE_SECOND, [](void * ) {
    Serial.println("callback is working, 1 second has passed");
    return true; // needed for the fn definition
  });
  bio::setup();
  bluetooth::setup();
  temperature::setup();
}

void loop() {
        /* data to be exported
         *  BODY_TEMP in Celsius (MLX90632)- same data type as sensor sends
         *  BIO_1 (JFH111)- unsigned 8 bit
         *  BIO_2 (JFH111)- unsigned 8 bit
         *  BIO_3 (JFH111)- unsigned 8 bit
         *  BIO_4 (JFH111)- unsigned 8 bit
         *  BIO_5 (JFH111)- unsigned 8 bit
         *  BIO_6 (JFH111)- unsigned 8 bit
         *  TIME_1 - long int- time in milliseconds after run period was selected and blower timer started
         *  TIME_2 - this is time selected by user, unsigned 8 bit, 15 = 15 min, 30 = 30 min, 45 = 45 min, 60 = 60 min
         *  BATT_LVL - unsigned 8 bit, if above TBD then no action and 3 bars, if between TBD and TBD then no action and two bars, if between TBD and TBD low battery warning and 1 bar, less than TBD the FW has shutoff the device, data will stop
         *  BLWR_Duty- unsigned 8 bit, run speed selected by user for both blowers, 0 = OFF, 127 = 50%, 255 = 100% 
         *  
         */
        auto body_temp = temperature::getBodyTemperature();
        auto heart_rate = bio::getHeartRateIfAvailable();
        bluetooth::write<float>(body_temp);
        bluetooth::write<uint8_t>(heart_rate);
        Serial.println("Body temperature:");
        Serial.println(body_temp);

        Serial.println("Heart rate:");
        Serial.println(heart_rate);
        // Need other bio data from the buffer here ...  
}
