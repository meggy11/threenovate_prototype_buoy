#include <Wire.h>
#include <AltSoftSerial.h>
#include <EEPROM.h>

AltSoftSerial trm; // Arduino Nano: RX = D8, TX = D9

// ============================================================
// CONFIGURATION
// ============================================================

#define USB_BAUD 115200
#define TRM_BAUD 19200

#define RTC_ADDR 0x32
#define MPU_ADDR 0x69

#define LIGHT_PIN 3
#define SIREN_PIN 2
#define TRM_POWER_PIN 4

/*
 * MOSFET logic levels.
 *
 * Current assumption:
 * HIGH = powered/on
 * LOW  = powered/off
 *
 * If your MOSFET modules are active-low, swap these values.
 */
#define MOSFET_ON_LEVEL HIGH
#define MOSFET_OFF_LEVEL LOW

/*
 * Time given to the TRM142/4G modem to boot after power-on.
 * Increase this if the modem does not respond to AT commands
 * after 15 seconds.
 */
const unsigned long TRM_BOOT_DELAY_MS = 15000UL;

#define EEPROM_LAST_CMD_ID_ADDR 0

const char BROKER[] = "31.147.205.19";
const int BROKER_PORT = 1883;

const char CLIENT_ID[] = "nanoTRM142node01";

const char MQTT_USER[] = "threenovate";
const char MQTT_PASS[] = "threenovate";

const char TOPIC_PUB[] = "threenovate/test";
const char TOPIC_SUB[] = "threenovate/cmd";

// Default complete procedure interval: 10 minutes
unsigned long procedureIntervalSeconds = 600;

// IMU batch settings
byte sampleCount = 5;
unsigned long samplePeriodSeconds = 10;

bool firstRun = true;
unsigned long lastRunRtcSeconds = 0;
unsigned long lastCommandId = 0;

bool trmPowerEnabled = false;

// ============================================================
// STRUCTURES
// ============================================================

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t gx;
  int16_t gy;
  int16_t gz;

  int16_t tempRaw;
};

#define MAX_SAMPLES 10

ImuSample samples[MAX_SAMPLES];

// ============================================================
// EEPROM COMMAND ID
// ============================================================

void loadLastCommandId() {
  EEPROM.get(
    EEPROM_LAST_CMD_ID_ADDR,
    lastCommandId
  );

  if (lastCommandId == 0xFFFFFFFF) {
    lastCommandId = 0;
  }

  Serial.print(F("Last handled command ID: "));
  Serial.println(lastCommandId);
}

void saveLastCommandId(unsigned long id) {
  lastCommandId = id;

  EEPROM.put(
    EEPROM_LAST_CMD_ID_ADDR,
    lastCommandId
  );
}

unsigned long extractCommandId(const char* text) {
  const char* p = strstr(text, "id=");

  if (!p) {
    return 0;
  }

  p += 3;

  return atol(p);
}

// ============================================================
// I2C / RTC
// ============================================================

bool i2cExists(byte address) {
  Wire.beginTransmission(address);

  return Wire.endTransmission() == 0;
}

byte decToBcd(byte value) {
  return ((value / 10 * 16) + (value % 10));
}

byte bcdToDec(byte value) {
  return ((value / 16 * 10) + (value % 16));
}

void rtcSetTime(
  byte hour,
  byte minute,
  byte second
) {
  Wire.beginTransmission(RTC_ADDR);

  Wire.write(0x00);

  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));

  // Remaining values retained from the original prototype.
  Wire.write(decToBcd(1));
  Wire.write(decToBcd(1));
  Wire.write(decToBcd(1));
  Wire.write(decToBcd(24));

  Wire.endTransmission();
}

bool rtcReadTime(
  byte& hour,
  byte& minute,
  byte& second
) {
  Wire.beginTransmission(RTC_ADDR);

  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  Wire.requestFrom(RTC_ADDR, 3);

  if (Wire.available() < 3) {
    return false;
  }

  second = bcdToDec(
    Wire.read() & 0x7F
  );

  minute = bcdToDec(
    Wire.read()
  );

  hour = bcdToDec(
    Wire.read() & 0x3F
  );

  return true;
}

unsigned long rtcSecondsOfDay() {
  byte hour;
  byte minute;
  byte second;

  if (!rtcReadTime(
        hour,
        minute,
        second
      )) {
    return 0;
  }

  return (
    (unsigned long)hour * 3600UL
    + (unsigned long)minute * 60UL
    + second
  );
}

unsigned long elapsedRtcSeconds(
  unsigned long nowSeconds,
  unsigned long previousSeconds
) {
  if (nowSeconds >= previousSeconds) {
    return nowSeconds - previousSeconds;
  }

  // Handle midnight rollover.
  return (
    86400UL
    - previousSeconds
    + nowSeconds
  );
}

bool initRTC() {
  Serial.println(F("\n=== INIT RTC ==="));

  if (!i2cExists(RTC_ADDR)) {
    Serial.println(
      F("RTC not found at address 0x32.")
    );

    return false;
  }

  /*
   * Current prototype behaviour:
   * reset RTC time to 00:00:00 whenever Arduino starts.
   *
   * Remove this line later if the RTC has a backup battery
   * and should preserve real time.
   */
  rtcSetTime(0, 0, 0);

  byte hour;
  byte minute;
  byte second;

  if (!rtcReadTime(
        hour,
        minute,
        second
      )) {
    Serial.println(F("RTC read failed."));

    return false;
  }

  Serial.print(F("RTC started at: "));
  Serial.print(hour);
  Serial.print(':');
  Serial.print(minute);
  Serial.print(':');
  Serial.println(second);

  return true;
}

// ============================================================
// 4G / TRM142 POWER CONTROL
// ============================================================

void power4GOn() {
  if (trmPowerEnabled) {
    return;
  }

  Serial.println(
    F("\n=== POWER ON 4G / TRM142 ===")
  );

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_ON_LEVEL
  );

  trmPowerEnabled = true;

  Serial.print(F("Waiting "));
  Serial.print(
    TRM_BOOT_DELAY_MS / 1000UL
  );
  Serial.println(
    F(" seconds for modem boot...")
  );

  delay(TRM_BOOT_DELAY_MS);
}

void power4GOff() {
  Serial.println(
    F("\n=== POWER OFF 4G / TRM142 ===")
  );

  /*
   * Read and display any remaining serial data
   * before removing modem power.
   */
  while (trm.available()) {
    Serial.write(trm.read());
  }

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  trmPowerEnabled = false;

  delay(500);

  Serial.println(
    F("4G / TRM142 power disabled.")
  );
}

// ============================================================
// TRM142 HELPERS
// ============================================================

void flushTRM() {
  while (trm.available()) {
    trm.read();
  }
}

bool waitFor(
  const char* expectedText,
  unsigned long timeout = 8000
) {
  char buffer[180];

  int position = 0;

  buffer[0] = '\0';

  unsigned long start = millis();

  while (millis() - start < timeout) {
    while (trm.available()) {
      char character = trm.read();

      Serial.write(character);

      if (
        position
        < (int)sizeof(buffer) - 1
      ) {
        buffer[position++] = character;
        buffer[position] = '\0';
      } else {
        memmove(
          buffer,
          buffer + 40,
          position - 40
        );

        position -= 40;

        buffer[position++] = character;
        buffer[position] = '\0';
      }

      if (
        strstr(
          buffer,
          expectedText
        ) != NULL
      ) {
        return true;
      }
    }
  }

  return false;
}

bool sendAT(
  const char* command,
  const char* expected = "OK",
  unsigned long timeout = 5000
) {
  flushTRM();

  Serial.print(F("\n>> "));
  Serial.println(command);

  trm.print(command);
  trm.print('\r');

  bool success = waitFor(
    expected,
    timeout
  );

  if (success) {
    Serial.println(F("\n[OK]"));
  } else {
    Serial.println(F("\n[TIMEOUT/FAIL]"));
  }

  return success;
}

bool initTRM() {
  Serial.println(F("\n=== INIT TRM142 ==="));

  bool success = true;

  success &= sendAT(
    "AT",
    "OK",
    3000
  );

  success &= sendAT(
    "ATI",
    "OK",
    3000
  );

  success &= sendAT(
    "AT+IPR?",
    "+IPR: 19200",
    3000
  );

  success &= sendAT(
    "AT+CPIN?",
    "READY",
    5000
  );

  success &= sendAT(
    "AT+CEREG?",
    "+CEREG: 0,1",
    5000
  );

  success &= sendAT(
    "AT+CGPADDR=1",
    "+CGPADDR:",
    5000
  );

  return success;
}

// ============================================================
// IMU
// ============================================================

bool initIMU() {
  Serial.println(F("\n=== INIT IMU ==="));

  if (!i2cExists(MPU_ADDR)) {
    Serial.println(
      F("MPU6050 not found at 0x69.")
    );

    return false;
  }

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x6B);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    Serial.println(
      F("Failed to wake MPU6050.")
    );

    return false;
  }

  Serial.println(F("MPU6050 OK."));

  return true;
}

void sleepIMU() {
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x6B);
  Wire.write(0x40);

  Wire.endTransmission();

  Serial.println(
    F("IMU sleep requested.")
  );
}

bool readIMU(ImuSample& sample) {
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B);

  if (
    Wire.endTransmission(false) != 0
  ) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() < 14) {
    return false;
  }

  sample.ax =
    Wire.read() << 8
    | Wire.read();

  sample.ay =
    Wire.read() << 8
    | Wire.read();

  sample.az =
    Wire.read() << 8
    | Wire.read();

  sample.tempRaw =
    Wire.read() << 8
    | Wire.read();

  sample.gx =
    Wire.read() << 8
    | Wire.read();

  sample.gy =
    Wire.read() << 8
    | Wire.read();

  sample.gz =
    Wire.read() << 8
    | Wire.read();

  return true;
}

byte collectIMUSamples() {
  Serial.println(
    F("\n=== COLLECT IMU SAMPLES ===")
  );

  if (sampleCount > MAX_SAMPLES) {
    sampleCount = MAX_SAMPLES;
  }

  for (
    byte index = 0;
    index < sampleCount;
    index++
  ) {
    Serial.print(F("Sample "));
    Serial.println(index + 1);

    if (!readIMU(samples[index])) {
      Serial.println(
        F("IMU sample failed.")
      );

      return index;
    }

    if (index < sampleCount - 1) {
      delay(
        samplePeriodSeconds
        * 1000UL
      );
    }
  }

  return sampleCount;
}

// ============================================================
// COMMAND HANDLING
// ============================================================

void pulsePin(
  byte pin,
  const char* name
) {
  Serial.print(F("Pulsing "));
  Serial.print(name);
  Serial.println(F(" for 10 seconds."));

  digitalWrite(
    pin,
    MOSFET_ON_LEVEL
  );

  delay(10000);

  digitalWrite(
    pin,
    MOSFET_OFF_LEVEL
  );
}

unsigned long parseNumberAfter(
  const char* text,
  const char* key,
  unsigned long fallback
) {
  const char* pointer = strstr(
    text,
    key
  );

  if (!pointer) {
    return fallback;
  }

  pointer += strlen(key);

  while (
    *pointer == ' '
    || *pointer == ':'
    || *pointer == '='
  ) {
    pointer++;
  }

  unsigned long value = atol(pointer);

  if (value == 0) {
    return fallback;
  }

  return value;
}

void processCommands(const char* text) {
  Serial.println(
    F("\n=== PROCESS COMMANDS ===")
  );

  unsigned long commandId =
    extractCommandId(text);

  if (commandId == 0) {
    Serial.println(
      F("No command ID found. Ignoring command.")
    );

    return;
  }

  Serial.print(
    F("Received command ID: ")
  );

  Serial.println(commandId);

  if (commandId <= lastCommandId) {
    Serial.println(
      F("Command already handled. Skipping.")
    );

    return;
  }

  if (
    strstr(
      text,
      "light on"
    ) != NULL
  ) {
    pulsePin(
      LIGHT_PIN,
      "LIGHT"
    );
  }

  if (
    strstr(
      text,
      "siren on"
    ) != NULL
  ) {
    pulsePin(
      SIREN_PIN,
      "SIREN"
    );
  }

  if (
    strstr(
      text,
      "interval"
    ) != NULL
  ) {
    unsigned long newInterval =
      parseNumberAfter(
        text,
        "interval",
        procedureIntervalSeconds
      );

    if (
      newInterval >= 10
      && newInterval <= 86400UL
    ) {
      procedureIntervalSeconds =
        newInterval;

      Serial.print(
        F("New procedure interval seconds: ")
      );

      Serial.println(
        procedureIntervalSeconds
      );
    }
  }

  if (
    strstr(
      text,
      "frequency"
    ) != NULL
  ) {
    unsigned long newInterval =
      parseNumberAfter(
        text,
        "frequency",
        procedureIntervalSeconds
      );

    if (
      newInterval >= 10
      && newInterval <= 86400UL
    ) {
      procedureIntervalSeconds =
        newInterval;

      Serial.print(
        F("New procedure interval seconds: ")
      );

      Serial.println(
        procedureIntervalSeconds
      );
    }
  }

  if (
    strstr(
      text,
      "sample_count"
    ) != NULL
  ) {
    unsigned long newCount =
      parseNumberAfter(
        text,
        "sample_count",
        sampleCount
      );

    if (
      newCount >= 1
      && newCount <= MAX_SAMPLES
    ) {
      sampleCount =
        (byte)newCount;

      Serial.print(
        F("New sample count: ")
      );

      Serial.println(sampleCount);
    }
  }

  if (
    strstr(
      text,
      "sample_period"
    ) != NULL
  ) {
    unsigned long newPeriod =
      parseNumberAfter(
        text,
        "sample_period",
        samplePeriodSeconds
      );

    if (
      newPeriod >= 1
      && newPeriod <= 3600UL
    ) {
      samplePeriodSeconds =
        newPeriod;

      Serial.print(
        F("New sample period seconds: ")
      );

      Serial.println(
        samplePeriodSeconds
      );
    }
  }

  saveLastCommandId(commandId);

  Serial.print(
    F("Saved handled command ID: ")
  );

  Serial.println(lastCommandId);
}

void listenForMqttCommands(
  unsigned long listenMilliseconds
) {
  Serial.println(
    F("\n=== LISTEN FOR MQTT COMMANDS ===")
  );

  char buffer[320];

  int position = 0;

  buffer[0] = '\0';

  unsigned long start = millis();

  while (
    millis() - start
    < listenMilliseconds
  ) {
    while (trm.available()) {
      char character = trm.read();

      Serial.write(character);

      if (
        position
        < (int)sizeof(buffer) - 1
      ) {
        buffer[position++] = character;
        buffer[position] = '\0';
      }
    }
  }

  if (position == 0) {
    Serial.println(
      F("No MQTT command received.")
    );

    return;
  }

  if (
    strstr(
      buffer,
      "+QMTRECV:"
    ) != NULL
  ) {
    processCommands(buffer);
  } else {
    Serial.println(
      F("Received data, but no MQTT command URC found.")
    );
  }
}

// ============================================================
// MQTT
// ============================================================

bool mqttConnect() {
  Serial.println(F("\n=== MQTT CONNECT ==="));

  /*
   * These commands may fail when there is no previous session.
   * That is expected and does not stop the procedure.
   */
  sendAT(
    "AT+QMTDISC=0",
    "OK",
    3000
  );

  sendAT(
    "AT+QMTCLOSE=0",
    "OK",
    3000
  );

  char command[180];

  snprintf(
    command,
    sizeof(command),
    "AT+QMTOPEN=0,\"%s\",%d",
    BROKER,
    BROKER_PORT
  );

  if (
    !sendAT(
      command,
      "+QMTOPEN: 0,0",
      30000
    )
  ) {
    Serial.println(
      F("MQTT open failed.")
    );

    return false;
  }

  snprintf(
    command,
    sizeof(command),
    "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
    CLIENT_ID,
    MQTT_USER,
    MQTT_PASS
  );

  if (
    !sendAT(
      command,
      "+QMTCONN: 0,0,0",
      15000
    )
  ) {
    Serial.println(
      F("MQTT connect failed.")
    );

    return false;
  }

  snprintf(
    command,
    sizeof(command),
    "AT+QMTSUB=0,1,\"%s\",0",
    TOPIC_SUB
  );

  if (
    !sendAT(
      command,
      "+QMTSUB:",
      10000
    )
  ) {
    Serial.println(
      F("MQTT subscribe failed.")
    );

    return false;
  }

  Serial.println(
    F("MQTT connected and subscribed.")
  );

  return true;
}

bool mqttPublishBatch(byte count) {
  Serial.println(
    F("\n=== MQTT PUBLISH BATCH ===")
  );

  char command[140];

  snprintf(
    command,
    sizeof(command),
    "AT+QMTPUB=0,0,0,0,\"%s\"",
    TOPIC_PUB
  );

  flushTRM();

  Serial.print(F("\n>> "));
  Serial.println(command);

  trm.print(command);
  trm.print('\r');

  if (!waitFor(">", 5000)) {
    Serial.println(
      F("No publish prompt.")
    );

    return false;
  }

  trm.print(F("{\"count\":"));
  trm.print(count);

  trm.print(F(",\"period_s\":"));
  trm.print(samplePeriodSeconds);

  trm.print(F(",\"samples\":["));

  for (
    byte index = 0;
    index < count;
    index++
  ) {
    if (index > 0) {
      trm.print(',');
    }

    float temperatureC =
      samples[index].tempRaw / 340.0
      + 36.53;

    trm.print(F("{\"n\":"));
    trm.print(index);

    trm.print(F(",\"ax\":"));
    trm.print(samples[index].ax);

    trm.print(F(",\"ay\":"));
    trm.print(samples[index].ay);

    trm.print(F(",\"az\":"));
    trm.print(samples[index].az);

    trm.print(F(",\"gx\":"));
    trm.print(samples[index].gx);

    trm.print(F(",\"gy\":"));
    trm.print(samples[index].gy);

    trm.print(F(",\"gz\":"));
    trm.print(samples[index].gz);

    trm.print(F(",\"tempC\":"));
    trm.print(temperatureC, 2);

    trm.print('}');
  }

  trm.print(F("]}"));

  /*
   * Ctrl+Z tells the modem that the MQTT payload is complete.
   */
  trm.write(0x1A);

  if (
    !waitFor(
      "+QMTPUB:",
      20000
    )
  ) {
    Serial.println(
      F("Batch publish failed.")
    );

    return false;
  }

  Serial.println(
    F("Batch publish OK.")
  );

  return true;
}

void mqttDisconnect() {
  sendAT(
    "AT+QMTDISC=0",
    "OK",
    5000
  );

  delay(2000);
}

// ============================================================
// MAIN PROCEDURE
// ============================================================

void runProcedure() {
  Serial.println(
    F("\n==============================")
  );

  Serial.println(
    F("=== RUNNING MAIN PROCEDURE ===")
  );

  Serial.println(
    F("==============================")
  );

  if (!initIMU()) {
    Serial.println(
      F("Procedure stopped: IMU failed.")
    );

    return;
  }

  /*
   * The 4G modem is normally powered off.
   * Power is enabled only for the communication cycle.
   */
  power4GOn();

  if (!initTRM()) {
    Serial.println(
      F("Procedure stopped: TRM failed.")
    );

    sleepIMU();
    power4GOff();

    return;
  }

  if (!mqttConnect()) {
    Serial.println(
      F("Procedure stopped: MQTT failed.")
    );

    sleepIMU();
    power4GOff();

    return;
  }

  /*
   * Wait for retained or live MQTT commands.
   */
  listenForMqttCommands(8000);

  byte collected =
    collectIMUSamples();

  if (collected > 0) {
    mqttPublishBatch(collected);
  }

  mqttDisconnect();

  /*
   * Disable the TRM142 power after MQTT communication.
   */
  power4GOff();

  sleepIMU();

  Serial.println(
    F("Procedure finished.")
  );
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);

  trm.begin(TRM_BAUD);

  Wire.begin();

  pinMode(
    LIGHT_PIN,
    OUTPUT
  );

  pinMode(
    SIREN_PIN,
    OUTPUT
  );

  pinMode(
    TRM_POWER_PIN,
    OUTPUT
  );

  /*
   * Ensure all MOSFET-controlled loads are off on startup.
   */
  digitalWrite(
    LIGHT_PIN,
    MOSFET_OFF_LEVEL
  );

  digitalWrite(
    SIREN_PIN,
    MOSFET_OFF_LEVEL
  );

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  trmPowerEnabled = false;

  delay(3000);

  Serial.println(
    F("=== ARDUINO + RTC + IMU + TRM142 LOOPING PROTOTYPE ===")
  );

  loadLastCommandId();

  if (!initRTC()) {
    Serial.println(
      F("WARNING: RTC failed. Timing may not work correctly.")
    );
  }
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  unsigned long currentRtcSeconds =
    rtcSecondsOfDay();

  bool shouldRun = false;

  if (firstRun) {
    shouldRun = true;
  } else {
    unsigned long elapsed =
      elapsedRtcSeconds(
        currentRtcSeconds,
        lastRunRtcSeconds
      );

    if (
      elapsed
      >= procedureIntervalSeconds
    ) {
      shouldRun = true;
    }
  }

  if (shouldRun) {
    runProcedure();

    lastRunRtcSeconds =
      rtcSecondsOfDay();

    firstRun = false;

    Serial.print(
      F("Next run in seconds: ")
    );

    Serial.println(
      procedureIntervalSeconds
    );
  }

  /*
   * Manual serial bridge for testing AT commands.
   *
   * Note: while the modem is powered off, forwarded commands
   * will not receive a response.
   */
  while (Serial.available()) {
    trm.write(
      Serial.read()
    );
  }

  while (trm.available()) {
    Serial.write(
      trm.read()
    );
  }

  delay(200);
}
