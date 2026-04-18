//Made by Renan Mandelo and Gabriel Lima
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI false

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ctype.h>
#include <string.h>
extern "C" {
  #include "mavlink/ardupilotmega/mavlink.h"
}

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

#define TELEMETRY_TX 12
#define TELEMETRY_RX 13
#define TELEMETRY_BAUD 57600

#define GSM_BAUD 9600
#define HEARTBEAT_INTERVAL_MS 1000UL
#define SMS_POLL_INTERVAL_MS 5000UL

// Authorized numbers allowed to control the system via SMS.
static const char *COMMAND_NUMBERS[] = {
  "+351929161529",
  "+5516991623916"
};
static const size_t COMMAND_NUMBER_COUNT = sizeof(COMMAND_NUMBERS) / sizeof(COMMAND_NUMBERS[0]);
static const char *STATUS_RECIPIENT = COMMAND_NUMBERS[0];

// Supported remote commands for SMS and serial monitor control.
enum RemoteCommand {
  CMD_NONE,
  CMD_ARM,
  CMD_DISARM,
  CMD_LOITER,
  CMD_BRAKE,
  CMD_STATUS,
};

HardwareSerial gsmSerial(1);
HardwareSerial telemetrySerial(2);
TinyGsm modem(gsmSerial);

// MAVLink identity for this ESP32 node and target autopilot.
uint8_t systemId = 200;
uint8_t componentId = MAV_COMP_ID_ONBOARD_COMPUTER;
uint8_t targetSystem = 1;
uint8_t targetComponent = 1;
bool gsmReady = false;

String lastModeLabel = "UNKNOWN";
unsigned long lastHeartbeatAt = 0;
unsigned long lastSmsPollAt = 0;
unsigned long lastTelemetryPrintAt = 0;

bool cubeHeartbeatSeen = false;
bool gpsSeen = false;
bool batterySeen = false;
bool telemetryLinkActive = false;
bool startupSmsSent = false;
uint8_t cubeBaseMode = 0;
uint32_t cubeCustomMode = 0;
uint8_t cubeSystemStatus = 0;
uint16_t batteryMv = 0;
int8_t batteryRemaining = -1;
uint8_t gpsFixType = 0;
uint8_t gpsSatellitesVisible = 0;
int32_t latitudeE7 = 0;
int32_t longitudeE7 = 0;
unsigned long lastCubeMessageAt = 0;

constexpr uint32_t MODE_LOITER = 5;
constexpr uint32_t MODE_BRAKE = 17;
constexpr float FORCE_DISARM_MAGIC = 21196.0f;
constexpr unsigned long TELEMETRY_LOSS_TIMEOUT_MS = 6000UL;
constexpr unsigned long TELEMETRY_PRINT_INTERVAL_MS = 2000UL;
constexpr uint8_t SMS_SEND_RETRY_COUNT = 3;

// Print timestamped messages to the USB serial monitor.

void logMessage(const String &message) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] ");
  Serial.println(message);
}

// Drain any bytes left in the UART buffer before a new transaction.
void clearSerialBuffer(HardwareSerial &serialPort) {
  while (serialPort.available()) {
    serialPort.read();
  }
}

// Send a raw AT command and collect the modem response.
String runATCommand(const char *command, uint32_t timeoutMs = 1000) {
  String response;

  clearSerialBuffer(gsmSerial);
  gsmSerial.print(command);
  gsmSerial.print("\r");

  const uint32_t deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    while (gsmSerial.available()) {
      response += static_cast<char>(gsmSerial.read());
    }

    if (response.indexOf("OK") >= 0 || response.indexOf("ERROR") >= 0 || response.indexOf('>') >= 0) {
      break;
    }
    delay(10);
  }

  response.trim();
  return response;
}

// Keep only numeric digits so different phone number formats match.
void normalizePhoneNumber(const char *source, char *destination, size_t capacity) {
  size_t outputIndex = 0;
  for (size_t index = 0; source[index] != '\0' && outputIndex + 1 < capacity; ++index) {
    if (isdigit(static_cast<unsigned char>(source[index]))) {
      destination[outputIndex++] = source[index];
    }
  }
  destination[outputIndex] = '\0';
}

bool isAuthorizedCommandSender(const char *sender) {
  char normalizedSender[32] = {0};
  normalizePhoneNumber(sender, normalizedSender, sizeof(normalizedSender));

  for (size_t index = 0; index < COMMAND_NUMBER_COUNT; ++index) {
    char normalizedAllowed[32] = {0};
    normalizePhoneNumber(COMMAND_NUMBERS[index], normalizedAllowed, sizeof(normalizedAllowed));
    if (strcmp(normalizedSender, normalizedAllowed) == 0) {
      return true;
    }
  }
  return false;
}

// Trim leading and trailing ASCII whitespace in place.
void trimAscii(char *value) {
  size_t length = strlen(value);
  size_t start = 0;
  while (start < length && isspace(static_cast<unsigned char>(value[start]))) {
    ++start;
  }

  size_t end = length;
  while (end > start && isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }

  if (start > 0) {
    memmove(value, value + start, end - start);
  }
  value[end - start] = '\0';
}

// Normalize incoming commands to upper case for reliable matching.
void toUpperAscii(char *value) {
  for (size_t index = 0; value[index] != '\0'; ++index) {
    value[index] = static_cast<char>(toupper(static_cast<unsigned char>(value[index])));
  }
}

// Decode operator commands coming from SMS or the serial monitor.
RemoteCommand parseRemoteCommand(const char *rawCommand) {
  char command[64] = {0};
  strncpy(command, rawCommand, sizeof(command) - 1);
  trimAscii(command);
  toUpperAscii(command);

  if (strcmp(command, "ARM") == 0) {
    return CMD_ARM;
  }
  if (strcmp(command, "DISARM") == 0) {
    return CMD_DISARM;
  }
  if (strcmp(command, "LOITER") == 0) {
    return CMD_LOITER;
  }
  if (strcmp(command, "BRAKE") == 0) {
    return CMD_BRAKE;
  }
  if (strcmp(command, "STATUS") == 0) {
    return CMD_STATUS;
  }
  return CMD_NONE;
}

// Serialize and send one MAVLink packet to the Cube.
void sendMavlinkMessage(const mavlink_message_t &message) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
  telemetrySerial.write(buffer, length);
}

// Keep the ESP32 visible as an onboard controller on the MAVLink network.
void sendHeartbeat() {
  mavlink_message_t message;
  mavlink_msg_heartbeat_pack(
    systemId,
    componentId,
    &message,
    MAV_TYPE_ONBOARD_CONTROLLER,
    MAV_AUTOPILOT_INVALID,
    0,
    0,
    0
  );
  sendMavlinkMessage(message);
}

// Generic COMMAND_LONG helper used by arm/disarm actions.
void sendCommandLong(uint16_t command, float param1, float param2 = 0.0f, float param3 = 0.0f,
                     float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f) {
  mavlink_message_t message;
  mavlink_msg_command_long_pack(
    systemId,
    componentId,
    &message,
    targetSystem,
    targetComponent,
    command,
    0,
    param1,
    param2,
    param3,
    param4,
    param5,
    param6,
    param7
  );
  sendMavlinkMessage(message);
}

// Request an ArduPilot flight mode change using custom_mode.
void setFlightMode(uint32_t customMode, const char *label) {
  mavlink_message_t message;
  mavlink_msg_set_mode_pack(
    systemId,
    componentId,
    &message,
    targetSystem,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    customMode
  );
  sendMavlinkMessage(message);

  lastModeLabel = label;
  logMessage(String("Requested flight mode: ") + label);
}

// Update the latest telemetry snapshot with decoded MAVLink data.
void decodeTelemetryMessage(const mavlink_message_t &message) {
  lastCubeMessageAt = millis();

  if (!telemetryLinkActive) {
    logMessage("Cube telemetry detected");
  }
  telemetryLinkActive = true;

  switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(&message, &heartbeat);
      cubeHeartbeatSeen = true;
      cubeBaseMode = heartbeat.base_mode;
      cubeCustomMode = heartbeat.custom_mode;
      cubeSystemStatus = heartbeat.system_status;
      break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t sysStatus;
      mavlink_msg_sys_status_decode(&message, &sysStatus);
      batterySeen = true;
      batteryMv = sysStatus.voltage_battery;
      batteryRemaining = sysStatus.battery_remaining;
      break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_gps_raw_int_t gpsRaw;
      mavlink_msg_gps_raw_int_decode(&message, &gpsRaw);
      gpsSeen = true;
      gpsFixType = gpsRaw.fix_type;
      gpsSatellitesVisible = gpsRaw.satellites_visible;
      latitudeE7 = gpsRaw.lat;
      longitudeE7 = gpsRaw.lon;
      break;
    }
    default:
      break;
  }
}

// Continuously parse inbound bytes so telemetry recovery happens automatically.
void pollCubeTelemetry() {
  static mavlink_message_t message;
  static mavlink_status_t status;

  while (telemetrySerial.available()) {
    const uint8_t byteRead = static_cast<uint8_t>(telemetrySerial.read());
    if (mavlink_parse_char(MAVLINK_COMM_0, byteRead, &message, &status)) {
      decodeTelemetryMessage(message);
    }
  }
}

// Detect telemetry loss, keep searching, and report recovery once packets return.
void updateTelemetryLinkState() {
  const unsigned long now = millis();

  if (telemetryLinkActive && (now - lastCubeMessageAt > TELEMETRY_LOSS_TIMEOUT_MS)) {
    telemetryLinkActive = false;
    cubeHeartbeatSeen = false;
    logMessage("Cube telemetry lost. Still searching for MAVLink traffic...");
  }
}

// Print a compact telemetry summary without flooding the serial monitor.
void printTelemetrySnapshot() {
  const unsigned long now = millis();
  if (now - lastTelemetryPrintAt < TELEMETRY_PRINT_INTERVAL_MS) {
    return;
  }
  lastTelemetryPrintAt = now;

  if (!cubeHeartbeatSeen) {
    Serial.println("[CUBE] OFFLINE | searching for heartbeat...");
    return;
  }

  Serial.print("[CUBE] ONLINE");
  Serial.print(" | linkAgeMs=");
  Serial.print(now - lastCubeMessageAt);

  if (batterySeen) {
    Serial.print(" | battery=");
    Serial.print(batteryMv);
    Serial.print("mV");
    if (batteryRemaining >= 0) {
      Serial.print(" ");
      Serial.print(batteryRemaining);
      Serial.print("%");
    }
  } else {
    Serial.print(" | battery=NO_DATA");
  }

  if (gpsSeen) {
    Serial.print(" | gpsFix=");
    Serial.print(gpsFixType);
    Serial.print(" sats=");
    Serial.print(gpsSatellitesVisible);
    Serial.print(" lat=");
    Serial.print(latitudeE7 / 10000000.0, 6);
    Serial.print(" lon=");
    Serial.print(longitudeE7 / 10000000.0, 6);
  } else {
    Serial.print(" | gps=NO_DATA");
  }

  Serial.println();
}

void armDrone() {
  sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
  lastModeLabel = "ARM_REQUESTED";
  logMessage("ARM command sent");
}

void disarmDrone() {
  sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f, FORCE_DISARM_MAGIC);
  lastModeLabel = "FORCE_DISARM_REQUESTED";
  logMessage("FORCE DISARM command sent");
}

// Send SMS with basic retries so transient network failures do not break the flow.
bool sendSMS(const char *number, const String &message) {
  if (!gsmReady) {
    logMessage("SMS skipped because the modem is not ready");
    return false;
  }

  for (uint8_t attempt = 1; attempt <= SMS_SEND_RETRY_COUNT; ++attempt) {
    int16_t signalQuality = modem.getSignalQuality();
    if (signalQuality == 99 || signalQuality <= 0) {
      logMessage(String("SMS retry ") + attempt + ": no GSM signal yet");
      delay(1000);
      continue;
    }

    if (modem.sendSMS(number, message)) {
      logMessage(String("SMS sent to ") + number + ": " + message);
      return true;
    }

    logMessage(String("SMS send failed on attempt ") + attempt);
    delay(1200);
  }

  logMessage(String("SMS failed after retries for ") + number);
  return false;
}

// Dispatch commands from SMS or serial monitor to the appropriate action.
void handleCommand(RemoteCommand command, const char *origin, const char *replyNumber = nullptr) {
  switch (command) {
    case CMD_ARM:
      armDrone();
      break;
    case CMD_DISARM:
      disarmDrone();
      break;
    case CMD_LOITER:
      setFlightMode(MODE_LOITER, "LOITER");
      break;
    case CMD_BRAKE:
      setFlightMode(MODE_BRAKE, "BRAKE");
      break;
    case CMD_STATUS:
      logMessage(String("Last requested action: ") + lastModeLabel);
      break;
    case CMD_NONE:
    default:
      logMessage(String("Invalid command via ") + origin);
      if (replyNumber) {
        sendSMS(replyNumber, "Invalid command. Use: loiter, brake, arm, disarm or status.");
      }
      return;
  }

  if (replyNumber) {
    if (command == CMD_STATUS) {
      sendSMS(replyNumber, String("Last requested action: ") + lastModeLabel);
    } else {
      sendSMS(replyNumber, String("Command accepted via ") + origin + ": " + lastModeLabel);
    }
  }
}

// Remove processed messages from the SIM storage to avoid duplicates.
void deleteSmsByIndex(int smsIndex) {
  if (smsIndex <= 0) {
    return;
  }

  gsmSerial.print("AT+CMGD=");
  gsmSerial.print(smsIndex);
  gsmSerial.print("\r");
  delay(250);
  clearSerialBuffer(gsmSerial);
}

// Parse the modem inbox listing and process each unread SMS command.
void processSmsResponse(char *response) {
  char *savePtr = nullptr;
  char *line = strtok_r(response, "\r\n", &savePtr);

  while (line) {
    if (strncmp(line, "+CMGL:", 6) == 0) {
      int smsIndex = -1;
      char sender[32] = {0};
      sscanf(line, "+CMGL: %d", &smsIndex);

      const char *cursor = line;
      int quoteCount = 0;
      const char *senderStart = nullptr;
      const char *senderEnd = nullptr;
      while (*cursor) {
        if (*cursor == '"') {
          ++quoteCount;
          if (quoteCount == 3) {
            senderStart = cursor + 1;
          } else if (quoteCount == 4) {
            senderEnd = cursor;
            break;
          }
        }
        ++cursor;
      }

      if (senderStart && senderEnd && senderEnd > senderStart) {
        size_t senderLength = static_cast<size_t>(senderEnd - senderStart);
        if (senderLength >= sizeof(sender)) {
          senderLength = sizeof(sender) - 1;
        }
        memcpy(sender, senderStart, senderLength);
        sender[senderLength] = '\0';
      }

      char *message = strtok_r(nullptr, "\r\n", &savePtr);
      while (message && message[0] == '\0') {
        message = strtok_r(nullptr, "\r\n", &savePtr);
      }

      if (message && message[0] != '\0') {
        trimAscii(message);
        logMessage(String("SMS received from ") + sender + ": " + message);

        if (isAuthorizedCommandSender(sender)) {
          handleCommand(parseRemoteCommand(message), "SMS", sender);
        } else {
          logMessage(String("Unauthorized number: ") + sender);
        }
      }

      deleteSmsByIndex(smsIndex);
    }

    line = strtok_r(nullptr, "\r\n", &savePtr);
  }
}

// Poll unread SMS messages on a fixed interval.
void pollSmsInbox() {
  if (!gsmReady) {
    return;
  }

  static char response[1800];
  size_t responseLength = 0;
  response[0] = '\0';

  clearSerialBuffer(gsmSerial);
  gsmSerial.print("AT+CMGL=\"REC UNREAD\"\r");
  delay(250);

  const uint32_t deadline = millis() + 2000;
  while (millis() < deadline) {
    while (gsmSerial.available()) {
      const char c = static_cast<char>(gsmSerial.read());
      if (responseLength + 1 < sizeof(response)) {
        response[responseLength++] = c;
        response[responseLength] = '\0';
      }
    }
    delay(10);
  }

  if (strstr(response, "+CMGL:") != nullptr) {
    processSmsResponse(response);
  }
}

// Allow local testing from the USB serial monitor.
void handleSerialMonitor() {
  if (!Serial.available()) {
    return;
  }

  String input = Serial.readStringUntil('\n');
  input.trim();
  input.toLowerCase();

  if (input.length() == 0) {
    return;
  }

  RemoteCommand command = parseRemoteCommand(input.c_str());
  handleCommand(command, "Serial");
}

// Basic modem bring-up and SMS text mode configuration.
bool probeModemBoot() {
  logMessage("Probing SIM800...");
  String response = runATCommand("AT", 3000);
  if (response.indexOf("OK") < 0) {
    logMessage("SIM800 did not respond to AT");
    return false;
  }
  runATCommand("ATE0", 1000);
  runATCommand("AT+CMEE=2", 1000);
  runATCommand("AT+CMGF=1", 1000);
  runATCommand("AT+CNMI=0,0,0,0,0", 1000);
  runATCommand("AT+CPMS=\"SM\",\"SM\",\"SM\"", 1000);
  runATCommand("AT+CSCS=\"GSM\"", 1000);
  runATCommand("AT+CMGD=1,4", 3000);
  return true;
}

// Power sequence for the SIM800 module.
void powerOnModem() {
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  delay(1000);

  digitalWrite(MODEM_PWKEY, HIGH);
  delay(1200);
  digitalWrite(MODEM_PWKEY, LOW);
  delay(5000);
}

// Notify the primary operator that the FTS application started correctly.
void sendStartupStatusSMS() {
  if (startupSmsSent || !gsmReady) {
    return;
  }

  if (sendSMS(STATUS_RECIPIENT, "FTS boot OK. MAVLink and SMS control are running.")) {
    startupSmsSent = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(1500);

  logMessage("ESP32 MAVLink + SMS controller starting");

  telemetrySerial.begin(TELEMETRY_BAUD, SERIAL_8N1, TELEMETRY_RX, TELEMETRY_TX);
  telemetrySerial.setRxBufferSize(1024);
  logMessage("Cube connected on Serial2 RX=13 TX=12");

  gsmSerial.begin(GSM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  gsmSerial.setRxBufferSize(1024);
  powerOnModem();
  gsmReady = probeModemBoot();

  if (gsmReady) {
    logMessage("SIM800 is ready for SMS");
    sendStartupStatusSMS();
  } else {
    logMessage("SMS unavailable; serial monitor control remains active");
  }

  Serial.println("Accepted commands: loiter, brake, arm, disarm, status");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastHeartbeatAt >= HEARTBEAT_INTERVAL_MS) {
    sendHeartbeat();
    lastHeartbeatAt = now;
  }

  pollCubeTelemetry();
  updateTelemetryLinkState();
  printTelemetrySnapshot();

  handleSerialMonitor();

  if (gsmReady && !startupSmsSent) {
    sendStartupStatusSMS();
  }

  if (now - lastSmsPollAt >= SMS_POLL_INTERVAL_MS) {
    pollSmsInbox();
    lastSmsPollAt = now;
  }
}