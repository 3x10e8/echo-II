// Echo II: Configuring RTC Alarm for Adafruit Adalogger over Serial
//   Used with adalogger_SLeepRTCAlarm running on SAMD21

import processing.serial.*;
Serial port;

char[] alarmTimes = { // to be generated by a GUI'
  // check to guarantee valid numbers (after additions)

  '$', '$', '*', //char(4), '>', // for syncing 
  //added '*' so first data symbol won't get lost in while(Serial.read() == '$')

  char(hour()), char(minute()), char(second()), 
    char(day()), char(month()), char(year()-2000), //'>',
  
  char(hour()), char(minute()+1), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour()), char(minute()+2), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour()), char(minute()+3), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  /*
  char(hour() +1), char(0), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour() +2), char(0), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour() +3), char(0), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour() +4), char(0), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour() +5), char(0), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour() +6), char(0), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
  char(hour() +7), char(0), char(0), 
    char(day()), char(month()), char(year()-2000), //'>',
    */
};

String alarmTimesStr = new String(alarmTimes);

void setup() {
  println(Serial.list()); // This will be a problem...
  String portName = Serial.list()[0];
  port = new Serial(this, portName, 9600);

  //int numAlarms = 3;
  port.write(alarmTimesStr.getBytes()); //println(alarmTimesStr);
}

void draw() { // needed to keep triggering serialEvent?
}

int bytesRxd = 0;
void serialEvent(Serial port) {
  bytesRxd++;
  int rxByte = port.read();
  if (rxByte == '@') { // printing host times for testing RTC alarms
    println("" +hour() +'.' +minute() +'.' +second() +'\t' +
      day() +'.' +month() +'.' +(year()-2000));
  } else { // rx'd memory dump
    if (bytesRxd == 1) print("---@\t Memory Dump: Time Set & Alarms \t@---"); 
    if (bytesRxd % 3 == 1) print("\t");
    if (bytesRxd % 6 == 1) println();
    print(rxByte);
    print(".");
    if (bytesRxd == 6) print(" (to set RTC time)");
    if (bytesRxd == (alarmTimesStr.length() -3)) 
      println("\n---@\t Host Times For Triggered Alarms \t@---"); // rx'd memory dump
  }
}