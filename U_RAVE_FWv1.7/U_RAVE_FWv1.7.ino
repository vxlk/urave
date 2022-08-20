#include "arduino-timer.h"

#include <BluetoothSerial.h>

#include <esp32-hal-ledc.h>

//https://randomnerdtutorials.com/esp32-bluetooth-classic-arduino-ide/
/*
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
*/

BluetoothSerial btSerial;

int PWR_CTL = 2; // start at 0-no function
int BLWR_PWM_L = 5; // start at 0
int BUZ_DRV = 13; // STart at 0
//int BLWR_PWM_R = 18;    // STart at 0
uint8_t BOOST_EN = 25; // active low
int LED_BL = 26; // start at 0
int BCTL = 27;
int BUT_3 = 32; // active low, 2ND DOWN after POWER
int BUT_4 = 33; // active low, 3RD DOWN after POWER
int BUT_1P = 34; // active low, POWER SW
int BUT_2 = 35; // active low, 1ST DOWN after POWER
int BAT_LVL = 36; // ANALOG-BATTERY
//int BAT_CHK = 39;        //ACTIVE HIGH
const int BLWR_PWM_R = 18;
/* Setting PWM Properties */
const uint32_t PWMFreq = 5000; /* 5 KHz */
const uint8_t PWMChannel = 0;
const uint8_t PWMResolution = 8;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
const int LONG_PRESS_TIME = 1000; // 1000 milliseconds
const int SHORT_PRESS_TIME = 50; // 50 milliseconds
const int BLWR_PWM_CHANNEL = 0;
const auto TONE_PWM_CHANNEL = 0;

// Variables will change:
int lastState = LOW; // the previous state from the input pin
uint32_t currentState = HIGH; // the current reading from the input pin
uint32_t BLWR_Duty = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long Blower_Run_Time_Start;
unsigned long Blower_Run_Time_Stop = 0;
bool batteryIsLow = false;
auto EXT_DAT = 0;

void setup() {
  // put your setup code here, to run once:
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  /* Attach the LED PWM Channel to the GPIO Pin */
  ledcAttachPin(BLWR_PWM_L, PWMChannel);
  ledcAttachPin(BLWR_PWM_R, PWMChannel);

  pinMode(PWR_CTL, OUTPUT);
  pinMode(BLWR_PWM_L, OUTPUT);
  pinMode(BUZ_DRV, OUTPUT);
  //pinMode(BLWR_PWM_R, OUTPUT);   
  pinMode(BOOST_EN, OUTPUT);
  pinMode(LED_BL, OUTPUT);
  pinMode(BCTL, OUTPUT);
  pinMode(BUT_1P, INPUT_PULLUP);
  pinMode(BUT_2, INPUT_PULLUP);
  pinMode(BUT_3, INPUT_PULLUP);
  pinMode(BUT_4, INPUT_PULLUP);
  pinMode(BAT_LVL, INPUT);
  // pinMode(BAT_CHK, OUTPUT);     

  const auto LOW_BATTERY_VALUE = 2; //todo: what is the real value here?
  Timer < 1 > oneSecondTimerWithOneCallback;
  const auto ONE_SECOND = 1000;
  oneSecondTimerWithOneCallback.every(ONE_SECOND, [](void * ) {
    const auto polledBatteryLevel = digitalRead(BAT_LVL);
    if (polledBatteryLevel > LOW_BATTERY_VALUE)
      batteryIsLow = true;
    else if (batteryIsLow) batteryIsLow = false;
    return true; // needed for the fn definition
  });

  Serial.begin(115200); // this is most likely the wrong baud rate
  btSerial.begin("U-Rave-Bluetooth");
}

void loop() {
  long TIME_2 = 0;
  //RESET/LOW POWER MODE ALL I/O
  //  digitalWrite(PWR_CTL, LOW); 
  digitalWrite(BOOST_EN, HIGH); //want the boost off at power up for low power mode
  digitalWrite(LED_BL, LOW); // off for low power
  digitalWrite(BCTL, LOW); // leave battery charger at default
  digitalWrite(BUZ_DRV, LOW);
  ledcWrite(0, 0);

  // check power switch
  //PWR ON while loop, wait for long press of button 1 to move ON, otherwise stay here
  // read the state of the power button
  currentState = digitalRead(BUT_1P);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH) { // button is released
    releasedTime = millis();
    // determine if long press occurred on button 1 , move on to DATA EXPORT if detected, otherwise loop forever
    long pressDuration = releasedTime - pressedTime;
    if (pressDuration > LONG_PRESS_TIME) {
      Serial.println("A long press is detected move on to DATA EXPORT"); //debug
    }
    // save the the last state
    lastState = currentState;
  }

  /* DATA EXPORT- if short press of button 1 then no data exported, if long press of button 1 then data exported while BLOWER TIMER is running,
   *  exit criteria is button 1 press that meets short or long criteria- this writes a 255 to EXT_DAT register when long press selected which means biometric data is to be exported*/
  digitalWrite(LED_BL, HIGH);
  digitalWrite(BUZ_DRV, HIGH);
  delay(500);
  digitalWrite(LED_BL, LOW);
  digitalWrite(BUZ_DRV, LOW);
  currentState = digitalRead(BUT_1P);
  //after next button 1 push decoded for short or long flash/buzz once for no data export and twice for data export

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH) { // button is released
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration < SHORT_PRESS_TIME) {
      //ledcWrite(0, 0);
      Serial.println("A short press is detected DO NOT SEND BLUETOOTH DATA");
      EXT_DAT = 0;
      digitalWrite(LED_BL, HIGH);
      delay(1000);
      digitalWrite(LED_BL, LOW);
    }

    if (pressDuration > LONG_PRESS_TIME) {
      Serial.println("A long press is detected SEND BLUETOOTH DATA");
      EXT_DAT = 255;
      digitalWrite(LED_BL, HIGH); // show that choice was accepted
      delay(1000);
      digitalWrite(LED_BL, LOW);
      delay(1000);
      digitalWrite(LED_BL, HIGH);
      delay(1000);
      digitalWrite(LED_BL, LOW);
      // save the the last state
      lastState = currentState;
    }

    /* BLOWER TIMER cycle time selection while loop- 4 different times to be selected by counting short presses of button 2, 1 =15 min, 2 = 30 min, 3 = 45 min, 4= 60 min then rollover back to one, flash the LED and pulse 
     *  buzzer 1-4 times to show current state, 
     *  when long press of button 2 detected lock in time period by counting short presses 
     this loop can be aborted by long press of BUT1 (back to beginning)  or low battery threshold reached (back to beginning), START the selected timer and advance to BLOWER FLOW 
     */
    digitalWrite(BOOST_EN, LOW); // turn on blower power supply
    currentState = digitalRead(BUT_2);
    if (lastState == HIGH && currentState == LOW) // button is pressed
      pressedTime = millis();
    else if (lastState == LOW && currentState == HIGH) { // button is released
      releasedTime = millis();
      long pressDuration = releasedTime - pressedTime;

      if (pressDuration < SHORT_PRESS_TIME)
        //ledcWrite(0, 0);
        //digitalWrite(MOTOR_PIN, LOW);
        Serial.println("A short press is detected, Buzzer and light will flash 1-4 times, for 15-60 minutes ");

      //todo: add code that will increment TIME_2 from 15 (flash and buzz once)  to 30 (flash and buzz twice) to 45 (flash and buzz thrice)to 60 (flash and buzz four times)then back to one until a long press is detected
      if (pressDuration > LONG_PRESS_TIME) // lock in TIME_2 variable and go to BLOWER FLOW
      {
        Serial.println("A long press is detected start BLOWER FLOW speed selection");
        ledcWriteNote(TONE_PWM_CHANNEL, NOTE_C, 4);
        //digitalWrite(MOTOR_PIN, HIGH);
        delay(500);
      }

      // save the the last state
      lastState = currentState;
    }

    /* BLOWER FLOW speed selection while loop- start at 127 for analogWrite of BLWR_PWM_R AND BLWR_PWM_L, add 10 for every short press of button 3, subtract 10 for every
     *  short press of button 4, exit criteria is long press of button 1 (abort and go to reset state) expiration of blower timer (abort and go back to reset state), or low battery threshhold reached (abort and go back to reset state 
    pulse the buzzer 3 times and before entering reset state, 
      */
    BLWR_Duty = 127; //half speed
    ledcWrite(PWMChannel, BLWR_Duty);
    if (TIME_2 == 15)
      Blower_Run_Time_Stop = 900000;
    if (TIME_2 == 30)
      Blower_Run_Time_Stop = 1800000;
    if (TIME_2 == 45)
      Blower_Run_Time_Stop = 2700000;
    if (TIME_2 == 60)
      Blower_Run_Time_Stop = 3600000;
    pressedTime = millis(); //start bluetooth transmssion if requested
    /*todo: inside a while loop, add 10 to duty cycle for every short press of button 3 up to max of 255, subtract 10 for every short press of button 4 for min of 0, 
     continuously check blower timer hasn't expired once every loop, check battery level once per loop, export
     * export Bluetooth once a second if EXT_DAT = 255, else skip it, if Bluetooth exported TIME_1 = pressiedTime, also, a long press of button 1 should send program back to reset state, check BAT_LVL once a second, exit if below TBD
     , stop bluetooth data when any exit criteria met   */
    long TIME_1 = releasedTime - pressedTime; //this should report the blower current blower run time in milliseconds while blower timer is running 
    while (TIME_1 < Blower_Run_Time_Stop && !batteryIsLow) {
      // export bluetooth
      if (EXT_DAT == 255) {
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
        if (btSerial.available()) {
          //todo: real data
          auto garb = 255;
          btSerial.write(garb);
        } else {
          Serial.write("Wanted to write to bluetooth, but could not :(");
        }
      }
    }
  }
}
