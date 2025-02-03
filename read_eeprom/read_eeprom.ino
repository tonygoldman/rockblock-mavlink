/*
  Reading and writing test of the EEPROM functions on the Artemis
  By: Nathan Seidle
  SparkFun Electronics
  Date: June 24th, 2019
  This example code is in the public domain.

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15376

  Page erase takes 15ms
  Write byte takes 30ms - This is much longer than Arduino that takes 3.3ms
  Float write across two words takes 30ms
  Update (no write) takes 1ms
*/

#include <EEPROM.h>

const int EEPROM_SIZE = 1024;
const int START_ADDRESS = 0;

size_t write_to_eeprom_current_time_and_qos(int address, char* msg)
{
  if (address > 1023) return 0;
  size_t len = strlen(msg);
  Serial.println("Write to EEPROM - ");
  Serial.println(msg);
  EEPROM.write(address, (uint8_t*)msg, len);

  return len + 1;
}
void setup()
{
  Serial.begin(115200);
  Serial.println("EEPROM Examples");


  EEPROM.init();
  delay(100);
  readAndPrintEEPROM();

  }
void cleanEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
  Serial.println("EEPROM Clean!");
}

void readAndPrintEEPROM() {
  char buffer[1024];
  int bytesRead = 0;

  // Read all bytes from EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
    int value = EEPROM.read(START_ADDRESS + i);
    if (value == -1) {
      // End of EEPROM reached
      break;
    }
    buffer[bytesRead++] = char(value);
  }



  // Print the parsed string
  Serial.println("Parsed string:");
  int len = sizeof(buffer);
  for (int i = 0; i < len; i++) {
    Serial.print(buffer[i]);
  }
  Serial.flush(); // YOHAI: Added this line, i'll explain it to you
}
  void loop() {

  }
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
