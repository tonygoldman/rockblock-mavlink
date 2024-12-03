#include <SoftwareSerial.h>
#include <MAVLink_ardupilotmega.h>
#include <rtos.h>
// Set up a new SoftwareSerial object

// D11 id the blue which is TX from the pixhawk, D15 is the purple which is RX from the pixhawk
SoftwareSerial mySerial(D11, D15);
// Usage: connect to Wi-Fi network and use QGroundControl to open MAVLink connection

void setup() {
  Serial.begin(115200);
  // We need slow speed for softwareserial
  mySerial.begin(9600);
}

void loop() {

  //TARGET DRONE
  uint8_t target_system = 1; // Target drone id
  uint8_t target_component = 0; // Target component, 0 = all
  uint16_t command = MAV_CMD_DO_SET_MODE; // Specific command for PX4
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
  mavlink_msg_command_long_pack( 1, 44, &msg, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  mySerial.write(buf, len);
  Serial.println("sent rtl");
  delay(5000);}

