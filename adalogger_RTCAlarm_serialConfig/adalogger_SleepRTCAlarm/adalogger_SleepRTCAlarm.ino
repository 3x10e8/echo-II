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

#define USING_SERIAL
// uncomment if indicating alarms through Serial writes
// skips putting the processor to sleep

#define redLED 13
#define grnLED 8

#define alSize 6 // hhmmssDDMMYY data stored per each alarm (6 bytes)
#define numAlarmsMax 30 // can be increased to fill unused memory
#define rxBufferSize 180 // alSize * numAlarmsMax

byte rxBuffer[rxBufferSize]; // use flash memory: https://github.com/cmaglie/FlashStorage
int bytesStored = 0;

int alarmNdx = 0; // ndx 0 is used to set the time

RTCZero rtc; // create an rtc object

void setup() {

  Serial.begin(9600); //while (!Serial);
  pinMode(redLED, OUTPUT); // configure as PWM?
  pinMode(grnLED, OUTPUT);

  serialEventOnSetup(); // couldn't get serialEvent to work...
  timeConfig(); // apparently need rtc.begin() first else alarm writes won't work...
  alarmConfig();

  //Serial.println(NVMCTRL->CTRLB.bit.SLEEPPRM); // = 48 (ASCII) = 0 (decimal)
  // could set to 3: http://community.atmel.com/forum/samd20-problem-waking-systemsleep

  // Could Serial.end() and/or USBDevice.disable() but standBy forces that...
}

void loop() {   // Processor is awake!

  // USBDevice.attach(); Serial.write('@');
  // Serial.begin(9600); while(!Serial); Serial.write('@'); Serial.end();
  // nope doesn't come up...

  #ifndef USING_SERIAL
  
    digitalWrite(grnLED, 1); delay(100);
    digitalWrite(grnLED, 0); delay(100);
    digitalWrite(grnLED, 1); delay(100);
    digitalWrite(grnLED, 0); delay(100);
  
    rtc.standbyMode();
  #endif
}

void timeConfig() { // set RTC time based on the received data

  rtc.begin();
  rtc.setTime(rxBuffer[alarmNdx + 0], rxBuffer[alarmNdx + 1],
              rxBuffer[alarmNdx + 2]); // hh, mm, ss
  rtc.setDate(rxBuffer[alarmNdx + 3], rxBuffer[alarmNdx + 4],
              rxBuffer[alarmNdx + 5]); // DD, MM, YY

  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS); // see RTCZero.h for other modes
  rtc.attachInterrupt(alarmMatch);
}

void alarmConfig() { // set alarms on startup or with each alarm trigger

  alarmNdx++;
  int i = alarmNdx * alSize; //Serial.println(i);
  rtc.setAlarmTime(rxBuffer[i + 0], rxBuffer[i + 1],
                   rxBuffer[i + 2]); // hh, mm, ss
  rtc.setAlarmDate(rxBuffer[i + 3], rxBuffer[i + 4],
                   rxBuffer[i + 5]); // DD, MM, YY

  // rtc.standbyMode(); // <--- wooo badddd. Hangs up if called outside loop() or setup().
}

void alarmMatch() { // can't flash LED here, delay() doesn't work in ISRs...

  #ifdef USING_SERIAL
    Serial.print('@');
  #endif

  // generate SD card file
  // enable recording

  // configure next alarm
  if ((alarmNdx + 1)*alSize <= bytesStored) alarmConfig();
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
  digitalWrite(redLED, 0); // off

  while (Serial.read() == '$'); // clear the header

  int bytesRxd = 0; // store received data to memory. flash would be preferable
  while (Serial.available()) { // to deal with the 64 byte s/w rx buffer
    bytesRxd = Serial.available();
    //Serial.println(bytesRxd);
    //Serial.println(bytesStored); // convert ASCII to int, read until '\n' (which is 1310)
    Serial.readBytes(rxBuffer + bytesStored, bytesRxd); // get the date/time data
    bytesStored += bytesRxd;
  }

  int bytesTxd = 0;//, i;
  while (bytesTxd < bytesStored) { // just to check the data
    // i = Serial.write(rxBuffer[bytesTxd]); // i returned 48 (decimal 0)...not bytes written...
    Serial.write(rxBuffer[bytesTxd]);
    bytesTxd++;
  }
}


void alarmConfig_debug() { // set alarms on startup or with each alarm trigger

  int i = alarmNdx * alSize; //Serial.println(i);
  rtc.setAlarmTime(rxBuffer[i + 0], rxBuffer[i + 1], rxBuffer[i + 2]); // hh, mm, ss
  rtc.setAlarmDate(rxBuffer[i + 3], rxBuffer[i + 4], rxBuffer[i + 5]); // DD, MM, YY

  alarmNdx++;

  /*
    Serial.write(rtc.getAlarmHours()); Serial.write(rtc.getAlarmMinutes());
      Serial.write(rtc.getAlarmSeconds());
    Serial.write(rtc.getAlarmMonth()); Serial.write(rtc.getAlarmDay());
      Serial.write(rtc.getAlarmYear());
  */

  rtc.standbyMode();
}
