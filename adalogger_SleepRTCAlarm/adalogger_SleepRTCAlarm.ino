/*
  Echo II: Configuring RTC Alarm for Adafruit Adalogger over Serial
    Used with simpleAlarmConfig running on the host PC (Processing 3)

    Receives current time and possible alarm times over Serial.
    Transmits back the received data.

    Issues:
      Time appears to be sync'd within seconds (can be improved)
      Error checking can be introduced to make receiving information reliable
      Problems with putting the processor to sleep - only one alarm works when Serial used!

  Adapted from:
  http://arduino.cc/en/Tutorial/SleepRTCAlarm by Arturo Guadalupi
  https://www.arduino.cc/en/Tutorial/SerialEvent
*/

#include <RTCZero.h>

// #define CLOSE_SERIAL_TEST // uncomment to twiddle registers for tackling the standbyMode()/USB issue

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

void loop() {}

void alarmMatch() { // delay() doesn't work in ISRs...?
  #ifndef CLOSE_SERIAL_TEST
    //Serial.begin(9600); while(!Serial); Serial.println();
    Serial.print('@');
  #endif
  
  ledStatus = !ledStatus;
  digitalWrite(grnLED, ledStatus);
  digitalWrite(redLED, !ledStatus);

  // generate SD card file
  
  // enable recording
  
  // configure next alarm
  if ((alarmNdx +1)*alSize <= bytesStored) alarmConfig(); 
  
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

  #ifdef CLOSE_SERIAL_TEST
    rtc.standbyMode(); // <--- wooo badddd. Need to test this w/o USB.
  #endif
  
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
  #ifdef CLOSE_SERIAL_TEST
    Serial.end();
    USB->DEVICE.CTRLA.bit.RUNSTDBY = 0; 
      // https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/USB/samd21_host.c
  #endif

  timeConfig(); // apparently need rtc.begin() first else alarm write syncs won't work...
  alarmConfig();
  
}

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

