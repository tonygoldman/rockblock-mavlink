#include <SoftwareSerial.h>
#include <MAVLink_ardupilotmega.h>
#include <rtos.h>
#include <IridiumSBD.h>

// consts from the artemis sbd example
#define BUS_VOLTAGE_PIN (13)                        // Bus voltage divided by 3 (Analog in)
#define IRIDIUM_SLEEP_PIN (17)                      // Iridium 9603N ON/OFF (sleep) pin: pull high to enable the 9603N
#define IRIDIUM_NA_PIN (18)                         // Input for the Iridium 9603N Network Available
#define IRIDIUM_ENABLE_PIN (22)                     // ADM4210 ON: pull high to enable power for the Iridium 9603N
#define IRIDIUM_RING_PIN (41)                       // Input for the Iridium 9603N Ring Indicator
#define GNSS_ENABLE_PIN (26)                        // GNSS Enable: pull low to enable power for the GNSS (via Q2)
#define GNSS_BACKUP_BATTERY_CHARGE_ENABLE_PIN (44)  // GNSS backup battery charge enable; when set as INPUT = disabled, when OUTPUT+LOW = charging.
#define SUPER_CAPACITOR_CHARGE_ENABLE_PIN (27)      // LTC3225 super capacitor charger: pull high to enable the super capacitor charger
#define SUPER_CAPACITOR_PGOOD_PIN (28)              // Input for the LTC3225 super capacitor charger PGOOD signal
#define BUS_VOLTAGE_MONITOR_ENABLE_PIN (34)         // Bus voltage monitor enable: pull high to enable bus voltage monitoring (via Q4 and Q3)
// project consts
#define SBD_MAX_MESSAGE_SIZE (150)
#define SEND_QUEUE_MESSAGE_COUNT (3)
#define SECOND_TO_MILLIS (1e3)
#define DRONE_STATUS_TRANSMISSION_INTERVAL_SEC (90)
#define SBD_SEND_EVENT (1UL << 0)
#define RTL_EVENT (1UL << 1)
#define STATUS_MSG_FORMAT ("{'signal':%d,'lat':%d,'lon':%d,'relative_alt':%d,'hdg':%u,'pitch':%s,'mode':%u,'voltage':%u,'time':%llu}")

typedef struct __drone_status_t {
  // from global position int
  int32_t lat = 0;          /*< [degE7] Latitude, expressed*/
  int32_t lon = 0;          /*< [degE7] Longitude, expressed*/
  int32_t relative_alt = 0; /*< [mm] Altitude above home*/
  uint16_t hdg = 0;         /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
  // from attitude
  float roll = 0;  /*< [rad] Roll angle (-pi..+pi)*/
  float pitch = 0; /*< [rad] Pitch angle (-pi..+pi)*/
  float yaw = 0;   /*< [rad] Yaw angle (-pi..+pi)*/
  // from heartbeat
  uint32_t custom_mode = 0; /*<  A bitfield for use for autopilot-specific flags*/
  // from sys status
  uint16_t voltage_battery = 0; /*< [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot*/
  // from system time
  uint64_t time_unix_usec = 0; /*< [us] Timestamp (UNIX epoch time).*/
} drone_status_t;

enum class sbdState : uint8_t {
  BOOTING,
  NO_CONNECTION,
  IDLE,
  SENDRECV,
  ERROR
};

SoftwareSerial mavlinkSerial(D11, D15);  // D11 id the blue which is TX from the pixhawk, D15 is the purple which is RX from the pixhawk
IridiumSBD sbdModem(Serial1, IRIDIUM_SLEEP_PIN, IRIDIUM_RING_PIN);
rtos::Thread mavlinkRxThread;
rtos::Thread mavlinkHeartbeatThread;
rtos::Thread mavlinkRtlThread;
rtos::Thread sendDroneStatusThread;
rtos::Mutex mavlinkSerialMutex;
rtos::EventFlags event_flags;
char sbdSendQueue[SEND_QUEUE_MESSAGE_COUNT * SBD_MAX_MESSAGE_SIZE];
drone_status_t droneStatus;
enum sbdState sbd_state = sbdState::BOOTING;
uint32_t counter = 0;

bool ISBDCallback(void) {
  if ((millis() % 1000) == 0) {
    Serial.println("SENDRECV in progress");
  }
  return true;
}

void droneStatusMessage(char* message) {
  int signalQuality = -1;
  sbdModem.getSignalQuality(signalQuality);
  
  // sprintf does not format floats due to a bug in mbedos github.com/sparkfun/Arduino_Apollo3/issues/278
  char pitchStr[8];
  ftoa(droneStatus.pitch,pitchStr,3,8);
  
  snprintf(
    message,
    SBD_MAX_MESSAGE_SIZE,
    STATUS_MSG_FORMAT,
    signalQuality,
    droneStatus.lat,
    droneStatus.lon,
    droneStatus.relative_alt,
    droneStatus.hdg,
    pitchStr,
    droneStatus.custom_mode,
    droneStatus.voltage_battery,
    droneStatus.time_unix_usec);
}


void gnssOFF(void)  // Disable power for the GNSS
{
  am_hal_gpio_pincfg_t pinCfg = g_AM_HAL_GPIO_OUTPUT;  // Begin by making the GNSS_ENABLE_PIN pin an open-drain output
  pinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
  pin_config(PinName(GNSS_ENABLE_PIN), pinCfg);
  delay(1);

  digitalWrite(GNSS_ENABLE_PIN, HIGH);  // Disable GNSS power (HIGH = disable; LOW = enable)
}

// TODO: is it really necessary
// Overwrite the IridiumSBD beginSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::beginSerialPort(void)  // Start the serial port connected to the satellite modem
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
void IridiumSBD::endSerialPort(void) {
  diagprint(F("custom IridiumSBD::endSerialPort\r\n"));

  // Disable the Serial1 RX pin to avoid the code hang
  am_hal_gpio_pinconfig(PinName(D25), g_AM_HAL_GPIO_DISABLE);
}

void setupModemPins(void) {
  gnssOFF();                                              // Disable power for the GNSS
  pinMode(GNSS_BACKUP_BATTERY_CHARGE_ENABLE_PIN, INPUT);  // GNSS backup batttery charge control; input = disable charging; output+low=charging.

  pinMode(IRIDIUM_ENABLE_PIN, OUTPUT);                   // Configure the Iridium Power Pin (connected to the ADM4210 ON pin)
  digitalWrite(IRIDIUM_ENABLE_PIN, LOW);                 // Disable Iridium Power (HIGH = enable; LOW = disable)
  pinMode(SUPER_CAPACITOR_CHARGE_ENABLE_PIN, OUTPUT);    // Configure the super capacitor charger enable pin (connected to LTC3225 !SHDN)
  digitalWrite(SUPER_CAPACITOR_CHARGE_ENABLE_PIN, LOW);  // Disable the super capacitor charger (HIGH = enable; LOW = disable)
  pinMode(IRIDIUM_SLEEP_PIN, OUTPUT);                    // Iridium 9603N On/Off (Sleep) pin
  digitalWrite(IRIDIUM_SLEEP_PIN, LOW);                  // Put the Iridium 9603N to sleep (HIGH = on; LOW = off/sleep)
  pinMode(IRIDIUM_RING_PIN, INPUT);                      // Configure the Iridium Ring Indicator as an input
  pinMode(IRIDIUM_NA_PIN, INPUT);                        // Configure the Iridium Network Available as an input
  pinMode(SUPER_CAPACITOR_PGOOD_PIN, INPUT);             // Configure the super capacitor charger PGOOD input
}

void setupModemSuperCap(void) {
  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  digitalWrite(SUPER_CAPACITOR_CHARGE_ENABLE_PIN, HIGH);  // Enable the super capacitor charger
  delay(1000);

  // Wait for the supercapacitor charger PGOOD signal to go high
  while (digitalRead(SUPER_CAPACITOR_PGOOD_PIN) == false) {
    Serial.println(F("Waiting for supercapacitors to charge..."));
    delay(1000);
  }
  Serial.println(F("Supercapacitors charged!"));
}

void startModem(void) {
  // Begin satellite modem operation
  // Also begin the serial port connected to the satellite modem via IridiumSBD::beginSerialPort
  Serial.println(F("Starting modem..."));
  int err = sbdModem.begin();
  if (err != ISBD_SUCCESS) {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }
}

void testSignal(void) {
  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  int signalQuality = -1;
  int err = sbdModem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS) {
    Serial.print(F("SignalQuality failed: error "));
    Serial.println(err);
    return;
  }

  Serial.print(F("On a scale of 0 to 5, signal quality is currently "));
  Serial.print(signalQuality);
  Serial.println(F("."));
}

void waitForNetwork(void) {
  // Check network available.
  Serial.println(F("Checking Network Available (for up to 60 seconds)..."));
  int NA = LOW;
  int loop_count = 0;
  while ((NA == LOW) && (loop_count < 60)) {
    NA = digitalRead(IRIDIUM_NA_PIN);
    Serial.print(F("Network is "));
    if (NA == LOW) Serial.print(F("NOT "));
    Serial.println(F("available!"));
    if (NA == LOW) Serial.println(F("(This might be because the 9603N has not yet aquired the ring channel.)"));
    delay(1000);
    loop_count++;
  }
}

void enableModemPower(void) {
  // Enable power for the 9603N
  Serial.println(F("Enabling 9603N power..."));
  digitalWrite(IRIDIUM_ENABLE_PIN, HIGH);  // Enable Iridium Power
  delay(1000);
  // If we're powering the device by USB, tell the library
  // to relax timing constraints waiting for the supercap to recharge.
  sbdModem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
}

void setupModem(void) {
  Serial.println("Initializing modem");
  setupModemPins();

  // TODO: is it really necessary
  // Make sure the Serial1 RX pin is disabled to prevent the power-on glitch on the modem's RX(OUT) pin
  // causing problems with v2.1.0 of the Apollo3 core. Requires v3.0.5 of IridiumSBDi2c.
  sbdModem.endSerialPort();

  setupModemSuperCap();
  enableModemPower();
  startModem();
  testSignal();
  sbd_state = sbdState::NO_CONNECTION;
  waitForNetwork();
  sbd_state = sbdState::IDLE;
}

// TODO: verify ack
void sendRTL() {
  uint8_t target_system = 1;               // Target drone id
  uint8_t target_component = 0;            // Target component, 0 = all
  uint16_t command = MAV_CMD_DO_SET_MODE;  // Specific command for PX4
  uint8_t confirmation = 0;
  float param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  float param2 = COPTER_MODE_RTL;
  float param3 = 0;
  float param4 = 0;
  float param5 = 0;
  float param6 = 0;
  float param7 = 0;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_command_long_pack(1, 44, &msg, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  mavlinkSerialMutex.lock();
  mavlinkSerial.write(buf, len);
  mavlinkSerialMutex.unlock();
}

void modemLoop(void) {
  static int err = ISBD_SUCCESS;
  bool ring = sbdModem.hasRingAsserted();
  int waitingMessageCount = sbdModem.getWaitingMessageCount();
  bool isSendEventSet = event_flags.get() & SBD_SEND_EVENT;

  if (ring) {
    Serial.println("Ring!");
  }

  if ((ring) || (waitingMessageCount > 0) || (isSendEventSet)) {
    // Clear the Mobile Originated message buffer - just in case it has an old message in it!
    Serial.println(F("Clearing the MO buffer (just in case)."));
    err = sbdModem.clearBuffers(ISBD_CLEAR_MO);  // Clear MO buffer
    if (err != ISBD_SUCCESS) {
      Serial.print(F("clearBuffers failed: error "));
      Serial.println(err);
      return;
    }

    uint8_t recvBuffer[SBD_MAX_MESSAGE_SIZE];
    size_t bufferSize = SBD_MAX_MESSAGE_SIZE;

    sbd_state = sbdState::SENDRECV;
    if (isSendEventSet) {
      char message[SBD_MAX_MESSAGE_SIZE];
      droneStatusMessage(message);
      Serial.print("Sending: ");
      Serial.println(message);
      err = sbdModem.sendReceiveSBDText(message, recvBuffer, bufferSize);
      event_flags.clear(SBD_SEND_EVENT);
    } else {
      err = sbdModem.sendReceiveSBDText(NULL, recvBuffer, bufferSize);
    }


    if (err != ISBD_SUCCESS) {
      Serial.print(F("sendReceiveSBDBinary failed: error "));
      Serial.println(err);
      sbd_state = sbdState::ERROR;
      return;
    } else {
      sbd_state = sbdState::IDLE;
    }

    if (bufferSize > 0) {
      Serial.println("Setting RTL flag");
      event_flags.set(RTL_EVENT);
    }

    if (bufferSize > 0) {
      Serial.print(F("Message received: "));
      for (int i = 0; i < (int)bufferSize; ++i) {
        if (isprint(recvBuffer[i])) {
          Serial.write(recvBuffer[i]);
        }
      }
      Serial.println();
      Serial.print(F("Inbound message size is "));
      Serial.println(bufferSize);
    }

    while (ring == true) {
      Serial.println(F("RING is still asserted. Waiting for it to clear..."));
      rtos::ThisThread::sleep_for(500);
      ring = sbdModem.hasRingAsserted();
    }

    Serial.println(F("RING has cleared. Begin waiting for a new RING..."));
  }

  rtos::ThisThread::sleep_for(1000);
}

void sendHeartbeat(void) {
  while (1) {
    // Generate HEARTBEAT message buffer
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(44, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, static_cast<uint8_t>(sbd_state));
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    mavlinkSerialMutex.lock();
    // Serial.println("Sending heartbeat");
    mavlinkSerial.write(buf, len);
    mavlinkSerialMutex.unlock();
    rtos::ThisThread::sleep_for(1000);
  }
}

void parseHeartbeat(mavlink_message_t* msg) {
  Serial.println("Parsing heartbeat");
  mavlink_heartbeat_t heartbeat;
  mavlink_msg_heartbeat_decode(msg, &heartbeat);
  droneStatus.custom_mode = heartbeat.custom_mode;
}

void parseGlobalPositionInt(mavlink_message_t* msg) {
  Serial.println("Parsing global position int");
  mavlink_global_position_int_t globalPositionInt;
  mavlink_msg_global_position_int_decode(msg, &globalPositionInt);
  droneStatus.lat = globalPositionInt.lat;
  droneStatus.lon = globalPositionInt.lon;
  droneStatus.hdg = globalPositionInt.hdg;
  droneStatus.relative_alt = globalPositionInt.relative_alt;
}

void parseAttitude(mavlink_message_t* msg) {
  Serial.println("Parsing attitude");
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(msg, &attitude);
  droneStatus.roll = attitude.roll;
  droneStatus.pitch = attitude.pitch;
  droneStatus.yaw = attitude.yaw;
}

void parseSysStatus(mavlink_message_t* msg) {
  Serial.println("Parsing sys status");
  mavlink_sys_status_t sysStatus;
  mavlink_msg_sys_status_decode(msg, &sysStatus);
  droneStatus.voltage_battery = sysStatus.voltage_battery;
}

void parseSystemTime(mavlink_message_t* msg) {
  Serial.println("Parsing sys status");
  mavlink_system_time_t systemTime;
  mavlink_msg_system_time_decode(msg, &systemTime);
  droneStatus.time_unix_usec = systemTime.time_unix_usec;
}

void receiveMAVLink(void) {
  int16_t lastReadChar;
  while (1) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlinkSerialMutex.lock();
    while ((lastReadChar = mavlinkSerial.read()) != -1) {
      if (mavlink_parse_char(MAVLINK_COMM_0, lastReadChar, &msg, &status)) {
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:
            parseHeartbeat(&msg);
            break;

          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            parseGlobalPositionInt(&msg);
            break;

          case MAVLINK_MSG_ID_ATTITUDE:
            parseAttitude(&msg);
            break;

          case MAVLINK_MSG_ID_SYS_STATUS:
            parseSysStatus(&msg);
            break;

          case MAVLINK_MSG_ID_SYSTEM_TIME:
            parseSystemTime(&msg);
            break;
        }
      }
    }
    mavlinkSerialMutex.unlock();
    rtos::ThisThread::sleep_for(10);
  }
}

void sendRtlRequest(void) {
  while (1) {
    event_flags.wait_all(RTL_EVENT);
    sendRTL();
  }
}

void setSbdSendFlag(void) {
  while (1) {
    event_flags.set(SBD_SEND_EVENT);
    rtos::ThisThread::sleep_for(DRONE_STATUS_TRANSMISSION_INTERVAL_SEC * SECOND_TO_MILLIS);
  }
}

void setup(void) {
  Serial.begin(115200);
  mavlinkSerial.begin(9600);  // We must set a low speed for softwareserial
  mavlinkRxThread.start(receiveMAVLink);
  mavlinkHeartbeatThread.start(sendHeartbeat);
  mavlinkRtlThread.start(sendRtlRequest);
  sendDroneStatusThread.start(setSbdSendFlag);
  Serial.println("Threads started successfully");
  setupModem();
  Serial.println("Setup completed, starting loop");
}

void loop(void) {
  modemLoop();
}

// sprintf does not format floats due to a bug in mbedos github.com/sparkfun/Arduino_Apollo3/issues/278
// This code is taken from github.com/sparkfun/SparkFun_Artemis_Global_Tracker/blob/main/Software/examples/Example15_BetterTracker/Example15_BetterTracker.ino

//*****************************************************************************
//
//  Divide an unsigned 32-bit value by 10.
//
//  Note: Adapted from Ch10 of Hackers Delight (hackersdelight.org).
//
//*****************************************************************************
static uint64_t divu64_10(uint64_t ui64Val)
{
    uint64_t q64, r64;
    uint32_t q32, r32, ui32Val;

    //
    // If a 32-bit value, use the more optimal 32-bit routine.
    //
    if ( ui64Val >> 32 )
    {
        q64 = (ui64Val>>1) + (ui64Val>>2);
        q64 += (q64 >> 4);
        q64 += (q64 >> 8);
        q64 += (q64 >> 16);
        q64 += (q64 >> 32);
        q64 >>= 3;
        r64 = ui64Val - q64*10;
        return q64 + ((r64 + 6) >> 4);
    }
    else
    {
        ui32Val = (uint32_t)(ui64Val & 0xffffffff);
        q32 = (ui32Val>>1) + (ui32Val>>2);
        q32 += (q32 >> 4);
        q32 += (q32 >> 8);
        q32 += (q32 >> 16);
        q32 >>= 3;
        r32 = ui32Val - q32*10;
        return (uint64_t)(q32 + ((r32 + 6) >> 4));
    }
}

//*****************************************************************************
//
// Converts ui64Val to a string.
// Note: pcBuf[] must be sized for a minimum of 21 characters.
//
// Returns the number of decimal digits in the string.
//
// NOTE: If pcBuf is NULL, will compute a return ui64Val only (no chars
// written).
//
//*****************************************************************************
static int uint64_to_str(uint64_t ui64Val, char *pcBuf)
{
    char tbuf[25];
    int ix = 0, iNumDig = 0;
    unsigned uMod;
    uint64_t u64Tmp;

    do
    {
        //
        // Divide by 10
        //
        u64Tmp = divu64_10(ui64Val);

        //
        // Get modulus
        //
        uMod = ui64Val - (u64Tmp * 10);

        tbuf[ix++] = uMod + '0';
        ui64Val = u64Tmp;
    } while ( ui64Val );

    //
    // Save the total number of digits
    //
    iNumDig = ix;

    //
    // Now, reverse the buffer when saving to the caller's buffer.
    //
    if ( pcBuf )
    {
        while ( ix-- )
        {
            *pcBuf++ = tbuf[ix];
        }

        //
        // Terminate the caller's buffer
        //
        *pcBuf = 0x00;
    }

    return iNumDig;
}

//*****************************************************************************
//
//  Float to ASCII text. A basic implementation for providing support for
//  single-precision %f.
//
//  param
//      fValue     = Float value to be converted.
//      pcBuf      = Buffer to place string AND input of buffer size.
//      iPrecision = Desired number of decimal places.
//      bufSize    = The size (in bytes) of the buffer.
//                   The recommended size is at least 16 bytes.
//
//  This function performs a basic translation of a floating point single
//  precision value to a string.
//
//  return Number of chars printed to the buffer.
//
//*****************************************************************************
#define OLA_FTOA_ERR_VAL_TOO_SMALL   -1
#define OLA_FTOA_ERR_VAL_TOO_LARGE   -2
#define OLA_FTOA_ERR_BUFSIZE         -3

typedef union
{
    int32_t I32;
    float F;
} ola_i32fl_t;

static int ftoa(float fValue, char *pcBuf, int iPrecision, int bufSize)
{
    ola_i32fl_t unFloatValue;
    int iExp2, iBufSize;
    int32_t i32Significand, i32IntPart, i32FracPart;
    char *pcBufInitial, *pcBuftmp;

    iBufSize = bufSize; // *(uint32_t*)pcBuf;
    if (iBufSize < 4)
    {
        return OLA_FTOA_ERR_BUFSIZE;
    }

    if (fValue == 0.0f)
    {
        // "0.0"
        *(uint32_t*)pcBuf = 0x00 << 24 | ('0' << 16) | ('.' << 8) | ('0' << 0);
        return 3;
    }

    pcBufInitial = pcBuf;

    unFloatValue.F = fValue;

    iExp2 = ((unFloatValue.I32 >> 23) & 0x000000FF) - 127;
    i32Significand = (unFloatValue.I32 & 0x00FFFFFF) | 0x00800000;
    i32FracPart = 0;
    i32IntPart = 0;

    if (iExp2 >= 31)
    {
        return OLA_FTOA_ERR_VAL_TOO_LARGE;
    }
    else if (iExp2 < -23)
    {
        return OLA_FTOA_ERR_VAL_TOO_SMALL;
    }
    else if (iExp2 >= 23)
    {
        i32IntPart = i32Significand << (iExp2 - 23);
    }
    else if (iExp2 >= 0)
    {
        i32IntPart = i32Significand >> (23 - iExp2);
        i32FracPart = (i32Significand << (iExp2 + 1)) & 0x00FFFFFF;
    }
    else // if (iExp2 < 0)
    {
        i32FracPart = (i32Significand & 0x00FFFFFF) >> -(iExp2 + 1);
    }

    if (unFloatValue.I32 < 0)
    {
        *pcBuf++ = '-';
    }

    if (i32IntPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        if (i32IntPart > 0)
        {
            uint64_to_str(i32IntPart, pcBuf);
        }
        else
        {
            *pcBuf++ = '-';
            uint64_to_str(-i32IntPart, pcBuf);
        }
        while (*pcBuf)    // Get to end of new string
        {
            pcBuf++;
        }
    }

    //
    // Now, begin the fractional part
    //
    *pcBuf++ = '.';

    if (i32FracPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        int jx, iMax;

        iMax = iBufSize - (pcBuf - pcBufInitial) - 1;
        iMax = (iMax > iPrecision) ? iPrecision : iMax;

        for (jx = 0; jx < iMax; jx++)
        {
            i32FracPart *= 10;
            *pcBuf++ = (i32FracPart >> 24) + '0';
            i32FracPart &= 0x00FFFFFF;
        }

        //
        // Per the printf spec, the number of digits printed to the right of the
        // decimal point (i.e. iPrecision) should be rounded.
        // Some examples:
        // Value        iPrecision          Formatted value
        // 1.36399      Unspecified (6)     1.363990
        // 1.36399      3                   1.364
        // 1.36399      4                   1.3640
        // 1.36399      5                   1.36399
        // 1.363994     Unspecified (6)     1.363994
        // 1.363994     3                   1.364
        // 1.363994     4                   1.3640
        // 1.363994     5                   1.36399
        // 1.363995     Unspecified (6)     1.363995
        // 1.363995     3                   1.364
        // 1.363995     4                   1.3640
        // 1.363995     5                   1.36400
        // 1.996        Unspecified (6)     1.996000
        // 1.996        2                   2.00
        // 1.996        3                   1.996
        // 1.996        4                   1.9960
        //
        // To determine whether to round up, we'll look at what the next
        // decimal value would have been.
        //
        if ( ((i32FracPart * 10) >> 24) >= 5 )
        {
            //
            // Yes, we need to round up.
            // Go back through the string and make adjustments as necessary.
            //
            pcBuftmp = pcBuf - 1;
            while ( pcBuftmp >= pcBufInitial )
            {
                if ( *pcBuftmp == '.' )
                {
                }
                else if ( *pcBuftmp == '9' )
                {
                    *pcBuftmp = '0';
                }
                else
                {
                    *pcBuftmp += 1;
                    break;
                }
                pcBuftmp--;
            }
        }
    }

    //
    // Terminate the string and we're done
    //
    *pcBuf = 0x00;

    return (pcBuf - pcBufInitial);
} // ftoa()