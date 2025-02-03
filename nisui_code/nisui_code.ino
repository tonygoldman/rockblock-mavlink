#include <RTC.h>

/*
 Artemis Global Tracker
 Example: Ring

 Written by Paul Clark (PaulZC)
 September 7th 2021

 ** Updated for v2.1.0 of the Apollo3 core / Artemis board package **
 ** (At the time of writing, v2.1.1 of the core conatins a feature which makes communication with the u-blox GNSS problematic. Be sure to use v2.1.0) **

 ** Set the Board to "RedBoard Artemis ATP" **
 ** (The Artemis Module does not have a Wire port defined, which prevents the IridiumSBD library from compiling) **

 This example demonstrates how to use the Iridium Ring Indicator line to detect
 when inbound messages are available and retrieve them.
 
 You will need to install version 3.0.5 of the Iridium SBD I2C library
 before this example will run successfully:
 https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
 (Available through the Arduino Library Manager: search for IridiumSBDi2c)

 Power for the 9603N is provided by the LTC3225 super capacitor charger.
 D27 needs to be pulled high to enable the charger.
 The LTC3225 PGOOD signal is connected to D28.

 Power for the 9603N is switched by the ADM4210 inrush current limiter.
 D22 needs to be pulled high to enable power for the 9603N.

 The 9603N itself is enabled via its ON/OFF (SLEEP) pin which is connected
 to D17. Pull high - via the IridiumSBD library - to enable the 9603N.

 The 9603N's Network Available signal is conected to D18,
 so let's configure D18 as an input.

 The 9603N's Ring Indicator is conected to D41,
 so let's configure D41 as an input.

 To prevent bad tings happening to the AS179 RF antenna switch,
 D26 needs to be pulled high to disable the GNSS power.

*/

// Artemis Tracker pin definitions
#define spiCS1              4  // D4 can be used as an SPI chip select or as a general purpose IO pin
#define geofencePin         10 // Input for the ZOE-M8Q's PIO14 (geofence) pin
#define busVoltagePin       13 // Bus voltage divided by 3 (Analog in)
#define iridiumSleep        17 // Iridium 9603N ON/OFF (sleep) pin: pull high to enable the 9603N
#define iridiumNA           18 // Input for the Iridium 9603N Network Available
#define LED                 19 // White LED
#define iridiumPwrEN        22 // ADM4210 ON: pull high to enable power for the Iridium 9603N
#define gnssEN              26 // GNSS Enable: pull low to enable power for the GNSS (via Q2)
#define gnssBckpBatChgEN    44 // GNSS backup battery charge enable; when set as INPUT = disabled, when OUTPUT+LOW = charging.
#define superCapChgEN       27 // LTC3225 super capacitor charger: pull high to enable the super capacitor charger
#define superCapPGOOD       28 // Input for the LTC3225 super capacitor charger PGOOD signal
#define busVoltageMonEN     34 // Bus voltage monitor enable: pull high to enable bus voltage monitoring (via Q4 and Q3)
#define spiCS2              35 // D35 can be used as an SPI chip select or as a general purpose IO pin
#define iridiumRI           41 // Input for the Iridium 9603N Ring Indicator
// Make sure you do not have gnssEN and iridiumPwrEN enabled at the same time!
// If you do, bad things might happen to the AS179 RF switch!

#include <EEPROM.h> // Needed for EEPROM storage on the Artemis
#include "RTC.h" //Include RTC library included with the Arduino_Apollo3 core

void getTimeString(char timeStringBuffer[])
{
  rtc.getTime();

  timeStringBuffer[0] = 0; // Clear the previous time

  char rtcTime[9]; // 09:14:37
  char rtcHour[3];
  char rtcMin[3];
  char rtcSec[3];
  if (rtc.hour < 10)
    sprintf(rtcHour, "0%d", rtc.hour);
  else
    sprintf(rtcHour, "%d", rtc.hour);
  if (rtc.minute < 10)
    sprintf(rtcMin, "0%d", rtc.minute);
  else
    sprintf(rtcMin, "%d", rtc.minute);
  if (rtc.seconds < 10)
    sprintf(rtcSec, "0%d", rtc.seconds);
  else
    sprintf(rtcSec, "%d", rtc.seconds);
  sprintf(rtcTime, "%s:%s:%s", rtcHour, rtcMin, rtcSec);
  strcat(timeStringBuffer, rtcTime);
}



size_t write_to_eeprom_current_time_and_qos(int address, char* msg)
{
  if (address > 1023) return 0;
  size_t len = strlen(msg);
  Serial.println("Write to EEPROM - ");
  Serial.println(msg);
  EEPROM.write(address, (uint8_t*)msg, len);

  return len + 1;
}
// We use Serial1 to communicate with the Iridium modem. Serial1 on the ATP uses pin 24 for TX and 25 for RX. AGT uses the same pins.

#include <IridiumSBD.h> //http://librarymanager/All#IridiumSBDI2C
#define DIAGNOSTICS false // Change this to true to see IridiumSBD diagnostics
// Declare the IridiumSBD object (including the sleep (ON/OFF) and Ring Indicator pins)
IridiumSBD modem(Serial1, iridiumSleep, iridiumRI);


// Overwrite the IridiumSBD beginSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::beginSerialPort() // Start the serial port connected to the satellite modem
{
  diagprint(F("custom IridiumSBD::beginSerialPort\r\n"));
  
  // Configure the standard ATP pins for UART1 TX and RX - endSerialPort may have disabled the RX pin
  
  am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
  pinConfigTx.uFuncSel = AM_HAL_PIN_24_UART1TX;
  pin_config(D24, pinConfigTx);
  
  am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
  pinConfigRx.uFuncSel = AM_HAL_PIN_25_UART1RX;
  pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK; // Put a weak pull-up on the Rx pin
  pin_config(D25, pinConfigRx);
  
  Serial1.begin(19200);
}

// Overwrite the IridiumSBD endSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::endSerialPort()
{
  diagprint(F("custom IridiumSBD::endSerialPort\r\n"));
  
  // Disable the Serial1 RX pin to avoid the code hang
  am_hal_gpio_pinconfig(PinName(D25), g_AM_HAL_GPIO_DISABLE);
}

int address = 0;

void setup()
{

  int signalQuality = -1;
  int err;
  pinMode(LED, OUTPUT); // Make the LED pin an output
  pinMode(gnssBckpBatChgEN, INPUT); // GNSS backup batttery charge control; input = disable charging; output+low=charging. 
  pinMode(geofencePin, INPUT); // Configure the geofence pin as an input

  pinMode(iridiumPwrEN, OUTPUT); // Configure the Iridium Power Pin (connected to the ADM4210 ON pin)
  digitalWrite(iridiumPwrEN, LOW); // Disable Iridium Power (HIGH = enable; LOW = disable)
  pinMode(superCapChgEN, OUTPUT); // Configure the super capacitor charger enable pin (connected to LTC3225 !SHDN)
  digitalWrite(superCapChgEN, LOW); // Disable the super capacitor charger (HIGH = enable; LOW = disable)
  pinMode(iridiumSleep, OUTPUT); // Iridium 9603N On/Off (Sleep) pin
  digitalWrite(iridiumSleep, LOW); // Put the Iridium 9603N to sleep (HIGH = on; LOW = off/sleep)
  pinMode(iridiumRI, INPUT); // Configure the Iridium Ring Indicator as an input
  pinMode(iridiumNA, INPUT); // Configure the Iridium Network Available as an input
  pinMode(superCapPGOOD, INPUT); // Configure the super capacitor charger PGOOD input
  pinMode(D11, OUTPUT); // Configure the Pixhawk pin

  // Make sure the Serial1 RX pin is disabled to prevent the power-on glitch on the modem's RX(OUT) pin
  // causing problems with v2.1.0 of the Apollo3 core. Requires v3.0.5 of IridiumSBDi2c.
  modem.endSerialPort();

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial) // Wait for the user to open the serial monitor
    ;
  delay(100);
  Serial.println();
  Serial.println();
  Serial.println(F("Artemis Global Tracker"));
  Serial.println();

  EEPROM.init(); // Initialize the EEPROM
  rtc.setToCompilerTime();

  // Manually set RTC date and time
  rtc.setTime(0, 00, 37, 14, 19, 11, 24); // 14:26:00.000, June 3rd, 2020 (hund, ss, mm, hh, dd, mm, yy)
  //empty the serial buffer
  while(Serial.available() > 0)
    Serial.read();

  //wait for the user to press any key before beginning
  Serial.println(F("Please check that the Serial Monitor is set to 115200 Baud"));
  Serial.println(F("and that the line ending is set to Newline."));
  Serial.println(F("Then click Send to start the example."));
  Serial.println();
  while(Serial.available() == 0)
    ;

  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  digitalWrite(superCapChgEN, HIGH); // Enable the super capacitor charger
  delay(1000);

  // Wait for the supercapacitor charger PGOOD signal to go high
  while (digitalRead(superCapPGOOD) == false)
  {
    Serial.println(F("Waiting for supercapacitors to charge..."));
    delay(1000);
  }
  Serial.println(F("Supercapacitors charged!"));

  // Enable power for the 9603N
  Serial.println(F("Enabling 9603N power..."));
  digitalWrite(iridiumPwrEN, HIGH); // Enable Iridium Power
  delay(1000);

  // If we're powering the device by USB, tell the library to 
  // relax timing constraints waiting for the supercap to recharge.
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  // Also begin the serial port connected to the satellite modem via IridiumSBD::beginSerialPort
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }

  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("SignalQuality failed: error "));
    Serial.println(err);
    return;
  }

  Serial.print(F("On a scale of 0 to 5, signal quality is currently "));
  Serial.print(signalQuality);
  Serial.println(F("."));

  // Check network available.
  Serial.println(F("Checking Network Available (for up to 60 seconds)..."));
  int NA = LOW;
  int loop_count = 0;
  while ((NA == LOW) && (loop_count < 60))
  {
    NA = digitalRead(iridiumNA);
    Serial.print(F("Network is "));
    if (NA == LOW) Serial.print(F("NOT "));
    Serial.println(F("available!"));
    if (NA == LOW) Serial.println(F("(This might be because the 9603N has not yet aquired the ring channel.)"));
    delay(1000);
    loop_count++;
  }

}
void loop()
{
  // Test the signal quality. 2 or better is preferred.
  int qos = -1;
  int errQuality = modem.getSignalQuality(qos);
  if (errQuality != ISBD_SUCCESS)
  {
    Serial.print(F("SignalQuality failed: error "));
    Serial.println(errQuality);
    return;
  }
  Serial.print(F("signal quality is currently "));
  Serial.println(qos);

  // Send message Quality of Signal + time
  Serial.println("Trying to send message - This might take several minutes...");
  char timeString[9]; // 09:14:37
  getTimeString(timeString);
  char msg[16];
  sprintf(msg, "%d %s", qos, timeString);
  Serial.println(msg);
  int errSend = modem.sendSBDText(msg);
  if (errSend != ISBD_SUCCESS)
  {
    address += write_to_eeprom_current_time_and_qos(address, msg);
    Serial.print(F("sendSBDText failed: error "));
    Serial.println(errSend);
    if (errSend == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println(F("Try again with a better view of the sky."));
  }
  else
  {
    Serial.println("Message sent!");
  }

  // Waiting for RING.
  Serial.println(F("Check if there is RING..."));
  static int err = ISBD_SUCCESS;
  bool ring = modem.hasRingAsserted();
  if ((ring == true) || (modem.getWaitingMessageCount() > 0) || (err != ISBD_SUCCESS))
  {
    if (ring == true) {
      Serial.println(F("RING asserted!  Let's try to read the incoming message."));
      Serial.println("Send to Pixhawk RTL Request!");
      digitalWrite(D11, HIGH); // pixhawk gpio 
      delay(10000);
      digitalWrite(D11, LOW);} // pixhawk gpio
    else if (modem.getWaitingMessageCount() > 0)
      Serial.println(F("Waiting messages available.  Let's try to read them."));
    else
      Serial.println(F("Let's try again."));

    // Clear the Mobile Originated message buffer - just in case it has an old message in it!
    Serial.println(F("Clearing the MO buffer."));
    err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("clearBuffers failed: error "));
      Serial.println(err);
      return;
    }

    uint8_t buffer[200];
    size_t bufferSize = sizeof(buffer);
    err = modem.sendReceiveSBDText(NULL, buffer, bufferSize);
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("sendReceiveSBDBinary failed: error "));
      Serial.println(err);
      return;
    }

    Serial.println(F("Message received!"));
    Serial.print(F("Inbound message size is "));
    Serial.println(bufferSize);
    // Check message content
    buffer[bufferSize] = 0; // Make sure message is NULL terminated
    String msg_str = String((char *)buffer); // Convert message into a String
    Serial.println(msg_str);
    Serial.println();
    Serial.print(F("Messages remaining to be retrieved: "));
    Serial.println(modem.getWaitingMessageCount());

    ring = modem.hasRingAsserted();
    while (ring == true)
    {
      Serial.println(F("RING is still asserted. Waiting for it to clear..."));
      delay(1000);
      ring = modem.hasRingAsserted();
    }
    
    Serial.println(F("RING has cleared. Begin waiting for a new RING..."));
  }
  else {
    Serial.println("No Ring yet...");
  }
  Serial.println("Delay 5 minutes ");
  delay(300000);
}

