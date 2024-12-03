#include <SoftwareSerial.h>
#include <MAVLink_ardupilotmega.h>
#include <rtos.h>
// Set up a new SoftwareSerial object

// D11 id the blue which is TX from the pixhawk, D15 is the purple which is RX from the pixhawk
SoftwareSerial mySerial(D11, D15);
rtos::Thread receive_thread;
rtos::Mutex serial_mutex;
// Usage: connect to Wi-Fi network and use QGroundControl to open MAVLink connection

void setup() {
  Serial.begin(115200);
  // We need slow speed for softwareserial
  mySerial.begin(9600);
  receive_thread.start(receiveMAVLink);
}

void loop() { 
  sendMAVLink();
}

void sendMAVLink() {
  // Generate HEARTBEAT message buffer
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(44, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  serial_mutex.lock();
  // Send buffer over UDP
  Serial.println("Sending heartbeat");
  mySerial.write(buf, len);
  serial_mutex.unlock();
  rtos::ThisThread::sleep_for(1000);
}

void receiveMAVLink() {
  while (1) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    serial_mutex.lock();
    while (mySerial.available() > 0) {
      uint8_t c = mySerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:
          Serial.println("Received heartbeat");
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
    serial_mutex.unlock();
    rtos::ThisThread::sleep_for(10);
  }
}
