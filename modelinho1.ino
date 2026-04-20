#include <Arduino.h>

extern "C" {
  #include "mavlink/ardupilotmega/mavlink.h"
}

// UART2 (ajuste se quiser)
#define RXD2 16
#define TXD2 17

// IDs MAVLink
uint8_t system_id = 1;
uint8_t component_id = 200;
uint8_t target_system = 1;
uint8_t target_component = 1;

// ===================== HEARTBEAT =====================
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

// ===================== STATUSTEXT =====================
void send_status_text(const char* text, uint8_t severity) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_statustext_pack(
    system_id,
    component_id,
    &msg,
    severity,
    text,
    0, 0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

// ===================== DISARM =====================
void send_disarm() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    system_id,
    component_id,
    &msg,
    target_system,
    target_component,
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, // param1 = 0 → DISARM
    0,0,0,0,0,0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);

  delay(2000); // deixa o Cube inicializar

  Serial.println("ESP32 MAVLink iniciado");
}

// ===================== LOOP =====================
void loop() {

  // envia heartbeat sempre
  send_heartbeat();

  static unsigned long last_msg = 0;
  static unsigned long last_cmd = 0;

  // envia mensagem pro Mission Planner
  if (millis() - last_msg > 3000) {
    Serial.println("Enviando STATUSTEXT...");
    send_status_text("ESP32 ONLINE", MAV_SEVERITY_INFO);
    last_msg = millis();
  }

  // envia comando de disarm (teste)
  if (millis() - last_cmd > 10000) {
    Serial.println("Enviando DISARM...");
    send_status_text("DISARM CMD SENT", MAV_SEVERITY_WARNING);
    send_disarm();
    last_cmd = millis();
  }

  delay(1000);
}