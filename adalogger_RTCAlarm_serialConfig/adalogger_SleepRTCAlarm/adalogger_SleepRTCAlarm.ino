/*
  Echo II: Configuring RTC Alarm for Adafruit Adalogger over Serial
    Used with simpleAlarmConfig running on the host PC (Processing 3)

    Receives current time and possible alarm times over Serial.
    Transmits back the received data.

    Issues:
      Time appears to be sync'd within seconds (can be improved)
      Error checking can be introduced to make receiving information reliable
      Problems with putting the processor to sleep - seem to hang up on waking up...

  Adapted from:
  http://arduino.cc/en/Tutorial/SleepRTCAlarm by Arturo Guadalupi
  https://www.arduino.cc/en/Tutorial/SerialEvent
*/

#include <RTCZero.h>

//#define USING_SERIAL // comment if testing w/o native USB

#define redLED 13
#define grnLED 8
boolean ledStatus = false;

#define alSize 6 // hhmmssDDMMYY data stored per each alarm (6 bytes)
#define numAlarmsMax 30 // can be increased to fill unused memory
#define rxBufferSize 180 // alSize * numAlarmsMax

byte rxBuffer[rxBufferSize]; // use flash memory: https://github.com/cmaglie/FlashStorage
int bytesStored = 0;

int alarmNdx = 1; // ndx 0 is used to set the time

RTCZero rtc; // create an rtc object

void setup() {
  
  Serial.begin(9600);
    //while (!Serial);
  pinMode(redLED, OUTPUT); // configure as PWM?
  pinMode(grnLED, OUTPUT);

  serialEventOnSetup(); // couldn't get serialEvent to work...
  
}

void loop() {

  digitalWrite(redLED, 1); delay(100); // to check if the processor hangs up post alarm!
  digitalWrite(redLED, 0); delay(100); 
 
  //Serial.begin(9600);
  //  while(!Serial);
    //Serial.begin(9600); while(!Serial); Serial.println();
  //Serial.print('@');
     rtc.standbyMode();
}

void timeConfig() { // set RTC time based on the received data
  
  rtc.begin();

  int i = 0;
  rtc.setTime(rxBuffer[i + 0], rxBuffer[i + 1], rxBuffer[i + 2]); // hh, mm, ss
  rtc.setDate(rxBuffer[i + 3], rxBuffer[i + 4], rxBuffer[i + 5]); // DD, MM, YY

  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS); // other modes are available, see RTCZero.h
  rtc.attachInterrupt(alarmMatch);
  
}

void alarmConfig() { // set alarms on startup or with each alarm trigger
  
  int i = alarmNdx * alSize; //Serial.println(i);
  rtc.setAlarmTime(rxBuffer[i + 0], rxBuffer[i + 1], rxBuffer[i + 2]); // hh, mm, ss
  rtc.setAlarmDate(rxBuffer[i + 3], rxBuffer[i + 4], rxBuffer[i + 5]); // DD, MM, YY
  
  alarmNdx++;

  /*
    Serial.write(rtc.getAlarmHours()); //Serial.write('_');
    Serial.write(rtc.getAlarmMinutes()); //Serial.write('_');
    Serial.write(rtc.getAlarmSeconds()); //Serial.write('`');
    Serial.write(rtc.getAlarmMonth()); //Serial.write('|');
    Serial.write(rtc.getAlarmDay()); //Serial.write('|');
    Serial.write(rtc.getAlarmYear());
  */

  //#ifdef USING_SERIAL
  //  Serial.end();
  //  USBDevice.detach(); // https://forums.adafruit.com/viewtopic.php?f=22&t=86144
  //  delay(500);
  //#endif

  //USBDevice.detach(); // https://forums.adafruit.com/viewtopic.php?f=22&t=86144
  //  delay(500);
  NVMCTRL->CTRLB.bit.SLEEPPRM = 3; //NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val; // http://community.atmel.com/forum/samd20-problem-waking-systemsleep
  //SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  //PM->SLEEP.reg = 2;
  //__WFI();
  
  rtc.standbyMode();
  
}

void alarmConfig2() { // set alarms on startup or with each alarm trigger
  
  int i = alarmNdx * alSize; //Serial.println(i);
  rtc.setAlarmTime(rxBuffer[i + 0], rxBuffer[i + 1], rxBuffer[i + 2]); // hh, mm, ss
  rtc.setAlarmDate(rxBuffer[i + 3], rxBuffer[i + 4], rxBuffer[i + 5]); // DD, MM, YY
  
  alarmNdx++;

  //NVMCTRL->CTRLB.bit.SLEEPPRM = 3; //NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val; // http://community.atmel.com/forum/samd20-problem-waking-systemsleep 
  //rtc.standbyMode();
  
}


void alarmMatch() { // delay() doesn't work in ISRs...?

  //USBDevice.attach();

  #ifdef USING_SERIAL
    //digitalWrite(grnLED, 1);
    //USBDevice.attach();
    //Serial.begin(9600);
    while(!Serial);
    //Serial.begin(9600); while(!Serial); Serial.println();
    Serial.print('@');
    //digitalWrite(grnLED, 0);
  #endif
  
  ledStatus = !ledStatus;
  digitalWrite(grnLED, ledStatus);
  digitalWrite(redLED, !ledStatus);
  //digitalWrite(grnLED, 1); //delay(100);
  //digitalWrite(grnLED, 0);

  // generate SD card file
  
  // enable recording
  
  // configure next alarm  // <--- wooo badddd. Need to test this w/o USB.
  // if ((alarmNdx +1)*alSize <= bytesStored) alarmConfig(); // THIS hangs up the processor post wake-up!
  alarmConfig2();
  
}

void serialEventOnSetup() {
  
  //___,_|_,-.____,_|_,-.____,_|_,-.____,_|_,-.__//
  // This is redundant: just to have a nice wait //
  // heartBeat() // somehow messes up call to Serial.write()
  
  int waitCounter = millis(), waitFor = 450, toFlip = 200;
  while (Serial.available() == 0 && Serial.read() != '$') { // waiting to receive data
    digitalWrite(redLED, 0); delayMicroseconds(millis() - waitCounter); digitalWrite(redLED, 1);
    if ((millis() - waitCounter) > waitFor) { //Serial.println("waiting...");
      waitCounter = millis();
      toFlip *= -1;
      waitFor += toFlip;
    }
  }
  digitalWrite(redLED, ledStatus); // off

  while (Serial.read() == '$'); // clear the header

  int bytesRxd = 0; // store received data to memory
  while (Serial.available()) { // to deal with the 64 byte s/w rx buffer
    bytesRxd = Serial.available();
      //Serial.println(bytesRxd);
      //Serial.println(bytesStored); // convert ASCII to int, read until '\n' (which is 1310)
    Serial.readBytes(rxBuffer +bytesStored, bytesRxd); // get the date/time data
    bytesStored += bytesRxd;
  }

  int bytesTxd = 0;//, i;
  while (bytesTxd < bytesStored) { // just to check the data
      // i = Serial.write(rxBuffer[bytesTxd]); // i returned 48 (decimal 0)...what...
    Serial.write(rxBuffer[bytesTxd]);
    bytesTxd++;
  }

  // failing attempts at resolving the standbyMode problem...
  //#ifndef USING_SERIAL
    //USBDevice.detach(); // https://forums.adafruit.com/viewtopic.php?f=22&t=86144
    //delay(500);
    //Serial.end();
    //USB->DEVICE.CTRLA.bit.RUNSTDBY = 0; 
      // https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/USB/samd21_host.c
  //#endif

  timeConfig(); // apparently need rtc.begin() first else alarm write syncs won't work...
  alarmConfig();
  
}

/*
void heartBeat(){
  
  int waitCounter = millis(), waitFor = 450, toFlip = 200;
  while (Serial.available() == 0 && Serial.read() != '$') { // waiting to receive data
    digitalWrite(redLED, 0); delayMicroseconds(millis() - waitCounter); digitalWrite(redLED, 1);
    if ((millis() - waitCounter) > waitFor) { //Serial.println("waiting...");
      waitCounter = millis();
      toFlip *= -1;
      waitFor += toFlip;
    }
  }
  
}
*/
