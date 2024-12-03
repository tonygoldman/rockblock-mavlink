#define busVoltagePin 13     // Bus voltage divided by 3 (Analog in)
#define iridiumSleep 17      // Iridium 9603N ON/OFF (sleep) pin: pull high to enable the 9603N
#define iridiumNA 18         // Input for the Iridium 9603N Network Available
#define LED 19               // White LED
#define iridiumPwrEN 22      // ADM4210 ON: pull high to enable power for the Iridium 9603N
#define gnssEN 26            // GNSS Enable: pull low to enable power for the GNSS (via Q2)
#define gnssBckpBatChgEN 44  // GNSS backup battery charge enable; when set as INPUT = disabled, when OUTPUT+LOW = charging.
#define superCapChgEN 27     // LTC3225 super capacitor charger: pull high to enable the super capacitor charger
#define superCapPGOOD 28     // Input for the LTC3225 super capacitor charger PGOOD signal
#define busVoltageMonEN 34   // Bus voltage monitor enable: pull high to enable bus voltage monitoring (via Q4 and Q3)
#define iridiumRI 41         // Input for the Iridium 9603N Ring Indicator

#define sbdMessageSize 150

#include <SoftwareSerial.h>
#include <MAVLink_ardupilotmega.h>
#include <rtos.h>
#include <IridiumSBD.h>

SoftwareSerial mavlinkSerial(D11, D15);  // D11 id the blue which is TX from the pixhawk, D15 is the purple which is RX from the pixhawk
IridiumSBD modem(Serial1, iridiumSleep, iridiumRI);
rtos::Thread mavlinkRxThread;
rtos::Thread modemThread;
rtos::Queue<uint8_t[sbdMessageSize], 3> modemRecvQueue;
rtos::Queue<char[sbdMessageSize], 3> modemSendQueue;
// TODO: use EventFlags to reduce sleep and optimize speed
rtos::Mutex mavlinkSerialMutex;
// Usage: connect to Wi-Fi network and use QGroundControl to open MAVLink connection
bool a = false;


void gnssOFF(void)  // Disable power for the GNSS
{
  am_hal_gpio_pincfg_t pinCfg = g_AM_HAL_GPIO_OUTPUT;  // Begin by making the gnssEN pin an open-drain output
  pinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
  pin_config(PinName(gnssEN), pinCfg);
  delay(1);

  digitalWrite(gnssEN, HIGH);  // Disable GNSS power (HIGH = disable; LOW = enable)
}

void setupModemPins(void) {
  pinMode(LED, OUTPUT);  // Make the LED pin an output

  gnssOFF();                         // Disable power for the GNSS
  pinMode(gnssBckpBatChgEN, INPUT);  // GNSS backup batttery charge control; input = disable charging; output+low=charging.

  pinMode(iridiumPwrEN, OUTPUT);     // Configure the Iridium Power Pin (connected to the ADM4210 ON pin)
  digitalWrite(iridiumPwrEN, LOW);   // Disable Iridium Power (HIGH = enable; LOW = disable)
  pinMode(superCapChgEN, OUTPUT);    // Configure the super capacitor charger enable pin (connected to LTC3225 !SHDN)
  digitalWrite(superCapChgEN, LOW);  // Disable the super capacitor charger (HIGH = enable; LOW = disable)
  pinMode(iridiumSleep, OUTPUT);     // Iridium 9603N On/Off (Sleep) pin
  digitalWrite(iridiumSleep, LOW);   // Put the Iridium 9603N to sleep (HIGH = on; LOW = off/sleep)
  pinMode(iridiumRI, INPUT);         // Configure the Iridium Ring Indicator as an input
  pinMode(iridiumNA, INPUT);         // Configure the Iridium Network Available as an input
  pinMode(superCapPGOOD, INPUT);     // Configure the super capacitor charger PGOOD input
}

void setupModemSuperCap(void) {
  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  digitalWrite(superCapChgEN, HIGH);  // Enable the super capacitor charger
  delay(1000);

  // Wait for the supercapacitor charger PGOOD signal to go high
  while (digitalRead(superCapPGOOD) == false) {
    Serial.println(F("Waiting for supercapacitors to charge..."));
    delay(1000);
  }
  Serial.println(F("Supercapacitors charged!"));
}

// TODO: is it really necessary
// Overwrite the IridiumSBD beginSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::beginSerialPort()  // Start the serial port connected to the satellite modem
{
  diagprint(F("custom IridiumSBD::beginSerialPort\r\n"));

  // Configure the standard ATP pins for UART1 TX and RX - endSerialPort may have disabled the RX pin

  am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
  pinConfigTx.uFuncSel = AM_HAL_PIN_24_UART1TX;
  pin_config(D24, pinConfigTx);

  am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
  pinConfigRx.uFuncSel = AM_HAL_PIN_25_UART1RX;
  pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK;  // Put a weak pull-up on the Rx pin
  pin_config(D25, pinConfigRx);

  Serial1.begin(19200);
}

// TODO: is it really necessary
// Overwrite the IridiumSBD endSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::endSerialPort() {
  diagprint(F("custom IridiumSBD::endSerialPort\r\n"));

  // Disable the Serial1 RX pin to avoid the code hang
  am_hal_gpio_pinconfig(PinName(D25), g_AM_HAL_GPIO_DISABLE);
}

void setupModem(void) {
  int signalQuality = -1;
  int err;

  setupModemPins();
  setupModemSuperCap();

  // TODO: is it really necessary
  // Make sure the Serial1 RX pin is disabled to prevent the power-on glitch on the modem's RX(OUT) pin
  // causing problems with v2.1.0 of the Apollo3 core. Requires v3.0.5 of IridiumSBDi2c.
  modem.endSerialPort();

  // Enable power for the 9603N
  Serial.println(F("Enabling 9603N power..."));
  digitalWrite(iridiumPwrEN, HIGH);  // Enable Iridium Power
  delay(1000);

  // If we're powering the device by USB, tell the library to
  // relax timing constraints waiting for the supercap to recharge.
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  // Also begin the serial port connected to the satellite modem via IridiumSBD::beginSerialPort
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err != ISBD_SUCCESS) {
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
  if (err != ISBD_SUCCESS) {
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
  while ((NA == LOW) && (loop_count < 60)) {
    NA = digitalRead(iridiumNA);
    Serial.print(F("Network is "));
    if (NA == LOW) Serial.print(F("NOT "));
    Serial.println(F("available!"));
    if (NA == LOW) Serial.println(F("(This might be because the 9603N has not yet aquired the ring channel.)"));
    delay(1000);
    loop_count++;
  }
}

bool ISBDCallback(void) {
  // Allow other threads run time during blocking calls
  rtos::ThisThread::sleep_for(100);  // TODO: check if effects boot and send times
  Serial.println("sleeping");
  return true;
}

void modemLoop() {
  while (1) {
    static int err = ISBD_SUCCESS;
    bool ring = modem.hasRingAsserted();
    uint8_t waitingMessageCount = modem.getWaitingMessageCount();
    uint8_t sendQueueSize = modemSendQueue.count();

    Serial.printf("ring:%d,waiting_message_count:%d,send_queue_size:%d\n", ring, waitingMessageCount, sendQueueSize);

    if (
      (ring) || (waitingMessageCount > 0) || (sendQueueSize > 0)) {

      if (ring)
        Serial.println(F("RING asserted!  Let's try to read the incoming message."));
      if (modem.getWaitingMessageCount() > 0)
        Serial.println(F("Waiting messages available.  Let's try to read them."));

      // Clear the Mobile Originated message buffer - just in case it has an old message in it!
      Serial.println(F("Clearing the MO buffer (just in case)."));
      err = modem.clearBuffers(ISBD_CLEAR_MO);  // Clear MO buffer
      if (err != ISBD_SUCCESS) {
        Serial.print(F("clearBuffers failed: error "));
        Serial.println(err);
        return;
      }

      uint8_t recvBuffer[sbdMessageSize];
      size_t bufferSize = sbdMessageSize;

      if (modemSendQueue.count() > 0) {
        char sendBuffer[sbdMessageSize];  // TODO: should be optimized using pointers
        char(*sendBufferPtr)[sbdMessageSize] = &sendBuffer;
        bool success = modemSendQueue.try_get(&sendBufferPtr);

        if (!success) {
          Serial.println("Error: failed to get message from send queue");
          continue;
        }

        err = modem.sendReceiveSBDText(sendBuffer, recvBuffer, bufferSize);
      } else {
        err = modem.sendReceiveSBDText(NULL, recvBuffer, bufferSize);
      }

      if (err != ISBD_SUCCESS) {
        Serial.print(F("sendReceiveSBDBinary failed: error "));
        Serial.println(err);
        return;
      }

      if (sizeof(recvBuffer) > 0) {

        modemRecvQueue.try_put(&recvBuffer);
      }

      Serial.println(F("Message received!"));
      Serial.print(F("Inbound message size is "));
      Serial.println(bufferSize);
      for (int i = 0; i < (int)bufferSize; ++i) {
        Serial.print(recvBuffer[i], HEX);
        if (isprint(recvBuffer[i])) {
          Serial.print(F("("));
          Serial.write(recvBuffer[i]);
          Serial.print(F(")"));
        }
        Serial.print(F(" "));
      }
      Serial.println();

      while (ring == true) {
        Serial.println(F("RING is still asserted. Waiting for it to clear..."));
        rtos::ThisThread::sleep_for(500);
        ring = modem.hasRingAsserted();
      }

      Serial.println(F("RING has cleared. Begin waiting for a new RING..."));
    }

    rtos::ThisThread::sleep_for(1000);
  }
}

void sendMAVLink() {
  // Generate HEARTBEAT message buffer
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(44, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  mavlinkSerialMutex.lock();
  // Send buffer over UDP
  Serial.println("Sending heartbeat");
  mavlinkSerial.write(buf, len);
  mavlinkSerialMutex.unlock();
  rtos::ThisThread::sleep_for(1000);
}

void receiveMAVLink() {
  while (1) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlinkSerialMutex.lock();
    while (mavlinkSerial.available() > 0) {
      uint8_t c = mavlinkSerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:
            Serial.println("Received heartbeat");
            if (!a) {
              char send_rockblock[sbdMessageSize];
              sprintf(send_rockblock, "%s", msg.sysid);
              modemSendQueue.try_put(&send_rockblock);
              a = true;
            }
            break;

          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            Serial.println("Received global position int");
            break;

            // default:
            //   Serial.print("Received message with ID ");
            //   Serial.println(msg.msgid);
        }
      }
    }
    mavlinkSerialMutex.unlock();
    rtos::ThisThread::sleep_for(10);
  }
}


void setup() {
  Serial.begin(115200);
  mavlinkSerial.begin(9600);  // We must set a low speed for softwareserial
  setupModem();
  // mavlinkRxThread.start(receiveMAVLink);
  modemThread.start(modemLoop);
  Serial.println("Setup completed, starting loop");
}

void loop() {
  // sendMAVLink();
}
