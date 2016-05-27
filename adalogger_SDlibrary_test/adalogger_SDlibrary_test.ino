// Project Echo | Timing writes to SD card using Sparkfun's SD library
// The goal here is to print the output of micros() to an SD card as quickly as possible
//  while using standard calls to the SD library.

// This sketch is adapted from:
//  https://learn.adafruit.com/adafruit-feather-m0-adalogger/using-the-sd-card
//   and Examples/SD/DumpFile

// Hardware: Adfruit Adalogger running Atmel SAMD21 ARM M0+ @48MHz

#include <SPI.h>
#include <SD.h>

// Set the pins used
#define cardSelect 4
#define redLED 13 // used for indicating button presses
#define grnLED 8 // lights up when logging
char filename[9]; // could reduce size...
File logfile;

// Some dummy data for more timing comparisons
// Replace with 32-bit left/right samples obtained from codec
float placeHolder1 = 1.0202020;
float placeHolder2 = 1.0202020;

void setup() {
  pinMode(redLED, INPUT);
  attachInterrupt(redLED, switchISR, FALLING); // for stopping logging. 
  // Could use other interrupts; I happened to have a pushbutton already there.
  pinMode(grnLED, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {} // wait until serial is available. Not neccessary?
  Serial.println("Welcome to the SD echo test!");

  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
  }

  strcpy(filename, "LOG00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = '0' + i / 10;
    filename[4] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) break;
  }

  logfile = SD.open(filename, FILE_WRITE);
  if ( ! logfile ) {
    Serial.print("Couldn't create ");
    Serial.println(filename);
  }
  Serial.print("Writing to "); Serial.println(filename);
  Serial.println(F("\nSend any character to begin")); // from MPU-6050 teapot demo!
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  digitalWrite(grnLED, HIGH);
  Serial.println("Push button to stop logging!");
}

void loop() {
  logfile.println(micros());
  //logfile.print(micros()); //logfile.print("\t");
  //logfile.println(placeHolder1);// logfile.print("\t");
  //logfile.println(placeHolder2);
  //placeHolder1 += 1.0;
  //placeHolder2 *= 2.0;
  //placeHolder = analogRead(0); // apparently costs ~400 micros -> can be reduced
  //placeHolder = analogRead(1);
  //delayMicroseconds(75); // adds up
}

void fileDump() { // print contents of file for copy/paste/plotting
  Serial.println("Reading back what we wrote...");
  File dataFile = SD.open(String(filename));

  if (dataFile) { // read from the file if its available
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    Serial.println("FIN"); // we did it!
    dataFile.close();
  } else { // else: error!
    Serial.println("error opening datalog");
  }
}

void switchISR() {
  logfile.flush(); logfile.close();
  digitalWrite(grnLED, LOW); // not logging anymore
  fileDump();
  exit(0); // z Z z
}


