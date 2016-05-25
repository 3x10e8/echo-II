/*
  Echo II: Configuring RTC Alarm for Adafruit Adalogger over Serial

    Receives current time and possible alarm times over Serial.
    Transmits back the received data.

    Issues:
      Time appears to be sync'd within seconds (can be improved)
      Error checking can be introduced to make receiving information reliable
      Only the first configured alarm appears to work, not sure why the rest fail

  Adapted from:
  http://arduino.cc/en/Tutorial/SleepRTCAlarm by Arturo Guadalupi
  https://www.arduino.cc/en/Tutorial/SerialEvent
*/

#include <RTCZero.h>

#define redLED 13
#define grnLED 8
boolean ledStatus = false;

#define alSize 6 //+hhmmssDDMMYY
#define numAlarmsMax 30
#define rxBufferSize 180 // alSize * numAlarmsMax
byte rxBuffer[rxBufferSize];
int bytesRead = 0;

int alarmNdx = 1; // ndx 0 is used to set the time

RTCZero rtc; // create an rtc object 

void setup() {
  Serial.begin(9600); // might help with debugging SD writes
  //while (!Serial);
  pinMode(redLED, OUTPUT); // configure as PWM?
  pinMode(grnLED, OUTPUT);
  digitalWrite(redLED, 0);
  digitalWrite(grnLED, 1);
  serialEventOnSetup();
}

void loop(){
  //  rtc.standbyMode();    // Sleep until next alarm match
}

void alarmMatch() { // delay() doesn't work in ISRs...?
  ledStatus = !ledStatus;
  digitalWrite(grnLED, ledStatus);
  digitalWrite(redLED, !ledStatus);
  
  // enable recording

  // setting the next alarm fails?
    //if (alarmNdx <= (int)bytesRead/(int)alSize) 
    //RTC->MODE2.INTFLAG.bit.ALARM0 = 1; // clear flag
    //RTC->MODE2.INTENSET.reg |= RTC_MODE2_INTENSET_ALARM0;
    //RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; 
    //rtc.begin();
    //rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS); // not needed every time?
    //rtc.attachInterrupt(alarmMatch);
  
  if ((alarmNdx +1)*alSize < bytesRead) alarmConfig(); // configure next alarm
}

void timeConfig() {
  int i = 0;
  
  rtc.begin();
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS); // not needed every time?
  rtc.attachInterrupt(alarmMatch);
  
  rtc.setTime(rxBuffer[i+0], rxBuffer[i+1], rxBuffer[i+2]); // hh, mm, ss
  rtc.setDate(rxBuffer[i+3], rxBuffer[i+4], rxBuffer[i+5]); // DD, MM, YY
}

void alarmConfig() {
  int i = alarmNdx*alSize;
  
  rtc.setAlarmTime(rxBuffer[i+0], rxBuffer[i+1], rxBuffer[i+2]); // hh, mm, ss
  rtc.setAlarmDate(rxBuffer[i+3], rxBuffer[i+4], rxBuffer[i+5]); // DD, MM, YY
 
  alarmNdx++;
  
  // "Set alarm for ";
  Serial.write(rtc.getAlarmHours()); Serial.write(':');
  Serial.write(rtc.getAlarmMinutes()); Serial.write(':');
  Serial.write(rtc.getAlarmSeconds()); Serial.write(' ');
  Serial.write(rtc.getAlarmMonth()); Serial.write('/');
  Serial.write(rtc.getAlarmDay()); Serial.write('/'); 
  Serial.write(rtc.getAlarmYear());
  
  rtc.standbyMode();
}

void serialEventOnSetup() { // couldn't get serialEvent to work...
  digitalWrite(grnLED, 0); //Serial.println("Waiting to be configured...");
  
  int waitInterval = millis();
  while (Serial.available() == 0 && Serial.read() != '$') { // waiting to receive data
    digitalWrite(redLED, 0); delayMicroseconds(millis() -waitInterval); digitalWrite(redLED, 1);
    if ((millis() -waitInterval) > 500) { //Serial.println("waiting...");
      waitInterval = millis(); 
    }
  }
  digitalWrite(redLED, ledStatus);

  bytesRead = Serial.available(); // includes '$' and '\n', hence -2 later
  // Serial.println(bytesRead); // convert ASCII to int, read until '\n' (which is 1310)
  Serial.read(); // '$'
  Serial.readBytes(rxBuffer, bytesRead -2);
  Serial.write(rxBuffer, bytesRead -2);
  
  timeConfig(); // apparently need rtc.begin() first else alarm write syncs won't work...
  alarmConfig(); // each alarm setting is 6 bytes
  
  /*
  bytesRead = 0;
  while (numPackets>0) { 
    bytesRead += Serial.readBytesUntil('>', rxBuffer+bytesRead, sizeof(byte)*alSize);
    numPackets--;
    //delay(1);
  }
  Serial.write(rxBuffer, bytesRead);
  */
  
  //byte numPackets[1];
          //int numPackets_int = (int)Serial.read();
  //Serial.readBytesUntil('\n', numPackets, 1);
  //int numPackets_int = (int) numPackets;
          //Serial.println(numPackets_int);
}
