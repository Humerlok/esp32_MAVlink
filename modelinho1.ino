#include <Arduino.h>
extern "C" {
  #include "mavlink/ardupilotmega/mavlink.h"
}

// UART (Serial2 no ESP32)
#define RXD2 16
#define TXD2 17

uint8_t system_id = 1;
uint8_t component_id = 200;

void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    system_id,
    component_id,
    &msg,
    MAV_TYPE_ONBOARD_CONTROLLER,
    MAV_AUTOPILOT_INVALID,
    0, 0, 0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

void send_disarm() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    system_id,
    component_id,
    &msg,
    1, // target system (Cube)
    1, // target component
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, // param1 = 0 → DISARM
    0,0,0,0,0,0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

void setup() {
  Serial.begin(115200);

  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);

  delay(5000); // deixa o Cube respirar

  Serial.println("Iniciando envio MAVLink...");
}

void loop() {
  send_heartbeat();

  static unsigned long lastCmd = 0;

  if (millis() - lastCmd > 3000) {
    Serial.println("Enviando DISARM...");
    send_disarm();
    lastCmd = millis();
  }

  delay(1000);
}