#include <Arduino.h>
extern "C" {
  #include "mavlink/cubepilot/mavlink.h"
}

// UART (Serial2 no ESP32)
#define RXD2 12
#define TXD2 13

uint8_t system_id = 1;
uint8_t component_id = 32;
uint8_t target_system = 1;      // ID do Cube
uint8_t target_component = 1;   // ID do Componente (Autopiloto)

// Definições de Modos de Voo (ArduCopter)
#define MODE_STABILIZE 0
#define MODE_ALTHOLD   2
#define MODE_LOITER    5

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

// Função genérica para enviar COMMAND_LONG
void send_command_long(uint16_t command, float p1, float p2 = 0, float p3 = 0, float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    system_id,
    component_id,
    &msg,
    target_system,
    target_component,
    command,
    0, // confirmation
    p1, p2, p3, p4, p5, p6, p7
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

// Funções específicas solicitadas
void arm_drone() {
  Serial.println(">>> Comando: ARM");
  send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f); // 1 = Arm
}

void disarm_drone() {
  Serial.println(">>> Comando: DISARM");
  send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f); // 0 = Disarm
}

void set_mode(uint32_t custom_mode) {
  // MAV_CMD_DO_SET_MODE: p1 = base_mode (1), p2 = custom_mode
  send_command_long(MAV_CMD_DO_SET_MODE, 1.0f, (float)custom_mode);
}

void setup() {
  Serial.begin(00);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);

  delay(2000);
  Serial.println("Monitor Serial pronto. Comandos: arm, disarm, stabilize, althold, loiter");
}

void loop() {
  // Envia Heartbeat a cada 1 segundo
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 1000) {
    send_heartbeat();
    lastHeartbeat = millis();
  }

  // Leitura do Terminal Serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove espaços e \r\n
    input.toLowerCase();

    if (input == "arm") {
      arm_drone();
    } 
    else if (input == "disarm") {
      disarm_drone();
    } 
    else if (input == "stabilize") {
      Serial.println(">>> Mudando para STABILIZE");
      set_mode(MODE_STABILIZE);
    } 
    else if (input == "althold") {
      Serial.println(">>> Mudando para ALTHOLD");
      set_mode(MODE_ALTHOLD);
    } 
    else if (input == "loiter") {
      Serial.println(">>> Mudando para LOITER");
      set_mode(MODE_LOITER);
    }
    else if (input != "") {
      Serial.print("Comando desconhecido: ");
      Serial.println(input);
    }
  }
}
