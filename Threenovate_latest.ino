#include <AltSoftSerial.h>
#include <Wire.h>
#include <EEPROM.h>

AltSoftSerial trm; // Arduino Nano: RX = D8, TX = D9

// ============================================================
// SERIAL CONFIGURATION
// ============================================================

#define USB_BAUD 115200
#define TRM_BAUD 19200

// ============================================================
// HARDWARE PINS
// ============================================================

#define LIGHT_PIN 3
#define LIGHT_DATA_PIN 5
#define SIREN_PIN 2
#define TRM_POWER_PIN 4

/*
 * Change these two values if your MOSFET modules are active-low.
 */
#define MOSFET_ON_LEVEL HIGH
#define MOSFET_OFF_LEVEL LOW

/*
 * WS2812 strip configuration.
 *
 * This driver is BUFFERLESS: it does not allocate 3 bytes per LED.
 * Every LED receives the same GRB value as the frame is transmitted.
 *
 * IMPORTANT:
 * - Designed for an Arduino Nano / ATmega328P running at 16 MHz.
 * - LIGHT_DATA_PIN must remain D5 with this implementation.
 * - Set LED_COUNT to the real number of LEDs in the strip.
 */
#define LED_COUNT 100U

/*
 * Default white intensity for each RGB channel.
 * 64 is about 25% of the 0-255 channel range.
 */
#define LED_WHITE_LEVEL 64U

#if F_CPU != 16000000UL
#error "Bufferless WS2812 driver requires a 16 MHz ATmega328P."
#endif

#if LIGHT_DATA_PIN != 5
#error "Bufferless WS2812 driver is hard-coded for Arduino Nano D5."
#endif

// ============================================================
// IMU CONFIGURATION — MPU6050
// ============================================================

#define MPU6050_ADDR 0x69

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_DATA_START 0x3B

/*
 * Arduino Nano has limited SRAM.
 * Keep this value relatively small.
 */
#define MAX_IMU_SAMPLES 10

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t temperatureRaw;

  int16_t gx;
  int16_t gy;
  int16_t gz;

  unsigned long relativeTimeMs;
};

ImuSample samples[MAX_IMU_SAMPLES];

/*
 * Default acquisition:
 *
 * 5 samples
 * one sample every 1 second
 */
byte sampleCount = 5;
unsigned long samplePeriodMs = 1000UL;

// ============================================================
// TIMING
// ============================================================

/*
 * The first procedure runs immediately after startup.
 *
 * After the procedure finishes, the device waits for this
 * duration before starting the next procedure.
 */
unsigned long offTimeMs = 5UL * 60UL * 1000UL;

/*
 * TRM startup delay after setting D4 to ON.
 */
const unsigned long TRM_BOOT_DELAY_MS = 20000UL;

/*
 * Pause between modem commands.
 */
const unsigned long COMMAND_PAUSE_MS = 500UL;

/*
 * Time during which the modem listens for MQTT commands.
 */
const unsigned long COMMAND_LISTEN_MS = 10000UL;

/*
 * Duration for light and siren commands.
 */
const unsigned long ACTUATOR_ON_MS = 10000UL;

// ============================================================
// MQTT CONFIGURATION
// ============================================================

const char BROKER[] = "31.147.205.19";
const int BROKER_PORT = 1883;

const char MQTT_USER[] = "threenovate";
const char MQTT_PASS[] = "threenovate";

const char CLIENT_ID[] = "nanoTRM142test02";

const char TOPIC_PUB[] = "threenovate/test";
const char TOPIC_SUB[] = "threenovate/cmd";

// ============================================================
// GLOBAL STATE
// ============================================================

String rxBuf;

unsigned long cycleNumber = 0;

/*
 * Last processed MQTT command ID is stored in EEPROM.
 *
 * This prevents a retained MQTT command from running again
 * after every reconnection.
 */
const int EEPROM_LAST_COMMAND_ID_ADDR = 0;

long lastCommandId = -1;

/*
 * Forward declarations for MQTT URC processing. A retained
 * +QMTRECV message can arrive while mqttConnect() is waiting for
 * the subscription response.
 */
void processQmtRecvLine(const String &line);
void processQmtRecvFromBuffer(const String &buffer);

// ============================================================
// BUFFERLESS WS2812 OUTPUT CONTROL
// ============================================================

/*
 * Send one WS2812 byte, most-significant bit first.
 *
 * The implementation uses direct PORTD access and fixed AVR cycle
 * delays. It is intentionally specific to:
 *
 *   Arduino Nano / ATmega328P
 *   CPU clock: 16 MHz
 *   data pin: D5 / PD5
 *
 * Interrupts are disabled only while the complete LED frame is
 * transmitted. For 100 LEDs this is approximately 3 ms.
 */
static inline void ws2812SendByte(uint8_t value) {
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
    if (value & mask) {
      /*
       * Logical 1:
       * approximately 0.7 us HIGH and 0.55 us LOW.
       */
      PORTD |= _BV(PD5);

      __builtin_avr_delay_cycles(8);

      PORTD &= (uint8_t)~_BV(PD5);

      __builtin_avr_delay_cycles(5);
    } else {
      /*
       * Logical 0:
       * approximately 0.35 us HIGH and 0.9 us LOW.
       */
      PORTD |= _BV(PD5);

      __builtin_avr_delay_cycles(3);

      PORTD &= (uint8_t)~_BV(PD5);

      __builtin_avr_delay_cycles(10);
    }
  }
}

/*
 * Send the same color to every LED without storing an LED array.
 *
 * WS2812 normally expects GRB byte order.
 */
void setAllLedsSameColor(
  uint8_t red,
  uint8_t green,
  uint8_t blue
) {
  uint8_t oldSreg = SREG;

  noInterrupts();

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    ws2812SendByte(green);
    ws2812SendByte(red);
    ws2812SendByte(blue);
  }

  /*
   * Restore the previous interrupt state.
   */
  SREG = oldSreg;

  /*
   * WS2812 reset/latch interval.
   */
  delayMicroseconds(300);
}

void forceLightOffFrame() {
  /*
   * Always transmit an all-zero frame.
   *
   * Do not skip this based on a software state flag: after reset,
   * the Arduino may not know that the strip still contains a
   * previously latched color.
   */
  setAllLedsSameColor(0, 0, 0);
}

void turnLightOff() {
  /*
   * Keep strip power enabled while sending the OFF frame.
   * If power is removed first, the LEDs cannot receive the zeros.
   */
  digitalWrite(LIGHT_PIN, MOSFET_ON_LEVEL);
  delay(5UL);

  forceLightOffFrame();

  /*
   * Then remove strip power through the MOSFET.
   */
  digitalWrite(LIGHT_PIN, MOSFET_OFF_LEVEL);

  /*
   * Hold the data line LOW to avoid parasitic powering through DIN.
   */
  digitalWrite(LIGHT_DATA_PIN, LOW);
}

void turnSirenOff() {
  digitalWrite(SIREN_PIN, MOSFET_OFF_LEVEL);
}

void turnLightOnFor(unsigned long durationMs) {
  Serial.println();
  Serial.println(F("Executing command: LIGHT ON"));

  /*
   * Power the strip before transmitting the frame.
   */
  digitalWrite(LIGHT_PIN, MOSFET_ON_LEVEL);
  delay(50UL);

  setAllLedsSameColor(
    LED_WHITE_LEVEL,
    LED_WHITE_LEVEL,
    LED_WHITE_LEVEL
  );

  Serial.print(F("Sent one bufferless white frame to "));
  Serial.print(LED_COUNT);
  Serial.println(F(" WS2812 LEDs."));

  Serial.print(F("Per-channel intensity: "));
  Serial.println(LED_WHITE_LEVEL);

  Serial.print(F("Light will remain ON for "));
  Serial.print(durationMs / 1000UL);
  Serial.println(F(" seconds."));

  delay(durationMs);

  turnLightOff();

  Serial.println(
    F("Zero frame sent and strip power switched OFF.")
  );
}

void turnSirenOnFor(unsigned long durationMs) {
  Serial.println();
  Serial.println(F("Executing command: SIREN ON"));

  digitalWrite(SIREN_PIN, MOSFET_ON_LEVEL);

  delay(durationMs);

  turnSirenOff();

  Serial.println(F("Siren OFF."));
}

// ============================================================
// TRM POWER CONTROL
// ============================================================

void powerOnTRM() {
  Serial.println();
  Serial.println(F("=== POWER ON TRM142 ==="));

  digitalWrite(TRM_POWER_PIN, MOSFET_ON_LEVEL);

  Serial.print(F("Waiting "));
  Serial.print(TRM_BOOT_DELAY_MS / 1000UL);
  Serial.println(F(" seconds for modem startup..."));

  delay(TRM_BOOT_DELAY_MS);

  Serial.println(F("TRM startup delay finished."));
}

void powerOffTRM() {
  Serial.println();
  Serial.println(F("Attempting TRM142 power OFF..."));

  digitalWrite(TRM_POWER_PIN, MOSFET_OFF_LEVEL);

  delay(500UL);

  /*
   * The pin functionality remains present even if the current
   * hardware does not physically remove modem power.
   */
  Serial.println(F("TRM power-control pin set to OFF."));
}

void setSafeOffState() {
  turnLightOff();
  turnSirenOff();
  powerOffTRM();
}

// ============================================================
// EEPROM COMMAND-ID STORAGE
// ============================================================

void loadLastCommandId() {
  EEPROM.get(
    EEPROM_LAST_COMMAND_ID_ADDR,
    lastCommandId
  );

  /*
   * Basic sanity check in case EEPROM contains invalid data.
   */
  if (
    lastCommandId < -1L ||
    lastCommandId > 2000000000L
  ) {
    lastCommandId = -1L;
  }

  Serial.print(F("Last command ID: "));
  Serial.println(lastCommandId);
}

void saveLastCommandId(long id) {
  if (id == lastCommandId) {
    return;
  }

  lastCommandId = id;

  EEPROM.put(
    EEPROM_LAST_COMMAND_ID_ADDR,
    lastCommandId
  );
}

// ============================================================
// IMU LOW-LEVEL FUNCTIONS
// ============================================================

bool writeImuRegister(
  byte registerAddress,
  byte value
) {
  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(registerAddress);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool imuExists() {
  Wire.beginTransmission(MPU6050_ADDR);

  return Wire.endTransmission() == 0;
}

bool wakeImu() {
  Serial.println();
  Serial.println(F("=== WAKE MPU6050 ==="));

  if (!imuExists()) {
    Serial.println(
      F("MPU6050 not detected at address 0x69.")
    );

    return false;
  }

  /*
   * Clear sleep bit in PWR_MGMT_1.
   */
  if (!writeImuRegister(
        MPU6050_PWR_MGMT_1,
        0x00
      )) {
    Serial.println(F("Failed to wake MPU6050."));

    return false;
  }

  delay(100UL);

  Serial.println(F("MPU6050 awake."));

  return true;
}

void sleepImu() {
  Serial.println();
  Serial.println(F("Putting MPU6050 into sleep mode..."));

  /*
   * Bit 6 in PWR_MGMT_1 enables sleep mode.
   */
  if (writeImuRegister(
        MPU6050_PWR_MGMT_1,
        0x40
      )) {
    Serial.println(F("MPU6050 sleeping."));
  } else {
    Serial.println(
      F("Could not put MPU6050 into sleep mode.")
    );
  }
}

bool readImuSample(
  ImuSample &sample,
  unsigned long relativeTimeMs
) {
  /*
   * Start reading at accelerometer X high byte.
   *
   * MPU6050 returns:
   *
   * AX high, AX low
   * AY high, AY low
   * AZ high, AZ low
   * temperature high, temperature low
   * GX high, GX low
   * GY high, GY low
   * GZ high, GZ low
   */
  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(MPU6050_DATA_START);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const byte bytesNeeded = 14;

  byte received = Wire.requestFrom(
    (int)MPU6050_ADDR,
    (int)bytesNeeded,
    (int)true
  );

  if (received != bytesNeeded) {
    return false;
  }

  sample.ax =
    (int16_t)((Wire.read() << 8) | Wire.read());

  sample.ay =
    (int16_t)((Wire.read() << 8) | Wire.read());

  sample.az =
    (int16_t)((Wire.read() << 8) | Wire.read());

  sample.temperatureRaw =
    (int16_t)((Wire.read() << 8) | Wire.read());

  sample.gx =
    (int16_t)((Wire.read() << 8) | Wire.read());

  sample.gy =
    (int16_t)((Wire.read() << 8) | Wire.read());

  sample.gz =
    (int16_t)((Wire.read() << 8) | Wire.read());

  sample.relativeTimeMs = relativeTimeMs;

  return true;
}

// ============================================================
// IMU SAMPLE COLLECTION
// ============================================================

byte collectImuSamples() {
  Serial.println();
  Serial.println(F("=== COLLECT IMU SAMPLES ==="));

  Serial.print(F("Requested samples: "));
  Serial.println(sampleCount);

  Serial.print(F("Sample period: "));
  Serial.print(samplePeriodMs);
  Serial.println(F(" ms"));

  byte successfulSamples = 0;

  unsigned long batchStart = millis();

  for (byte i = 0; i < sampleCount; i++) {
    unsigned long targetTime =
      batchStart +
      ((unsigned long)i * samplePeriodMs);

    /*
     * Wait until the requested sample time.
     *
     * The signed subtraction remains safe when millis()
     * eventually overflows.
     */
    while ((long)(millis() - targetTime) < 0) {
      delay(1UL);
    }

    ImuSample candidate;

    unsigned long relativeTime =
      millis() - batchStart;

    if (readImuSample(
          candidate,
          relativeTime
        )) {
      samples[successfulSamples] = candidate;

      Serial.print(F("Sample "));
      Serial.print(successfulSamples);

      Serial.print(F(" | A: "));
      Serial.print(candidate.ax);
      Serial.print(F(", "));
      Serial.print(candidate.ay);
      Serial.print(F(", "));
      Serial.print(candidate.az);

      Serial.print(F(" | G: "));
      Serial.print(candidate.gx);
      Serial.print(F(", "));
      Serial.print(candidate.gy);
      Serial.print(F(", "));
      Serial.println(candidate.gz);

      successfulSamples++;
    } else {
      Serial.print(
        F("IMU read failed at requested sample ")
      );

      Serial.println(i);
    }
  }

  Serial.print(F("Successful samples: "));
  Serial.println(successfulSamples);

  return successfulSamples;
}

// ============================================================
// MODEM SERIAL HELPERS
// ============================================================

void flushTRM() {
  while (trm.available()) {
    trm.read();
  }

  rxBuf = "";
}

bool waitFor(
  const char *expected,
  unsigned long timeoutMs
) {
  unsigned long start = millis();

  size_t expectedLength = strlen(expected);
  size_t matched = 0;

  while (millis() - start < timeoutMs) {
    while (trm.available()) {
      char c = trm.read();

      Serial.write(c);

      /*
       * Keep a bounded copy for later +QMTRECV parsing.
       * This prevents unbounded String growth on the Nano.
       */
      if (rxBuf.length() < 420U) {
        rxBuf += c;
      }

      /*
       * Match the expected response directly from the incoming
       * stream, so success does not depend on rxBuf allocation.
       */
      if (c == expected[matched]) {
        matched++;

        if (matched == expectedLength) {
          unsigned long quietStart = millis();

          /*
           * Continue reading until the modem has been quiet for
           * 500 ms. This also captures an immediately delivered
           * retained +QMTRECV message.
           */
          while (millis() - quietStart < 500UL) {
            while (trm.available()) {
              char d = trm.read();

              Serial.write(d);

              if (rxBuf.length() < 420U) {
                rxBuf += d;
              }

              quietStart = millis();
            }
          }

          return true;
        }
      } else {
        /*
         * Restart matching. If this character is also the first
         * expected character, preserve it as a one-character match.
         */
        matched = (c == expected[0]) ? 1U : 0U;
      }
    }
  }

  return false;
}

bool sendAT(
  const char *command,
  const char *expected,
  unsigned long timeoutMs
) {
  flushTRM();

  Serial.println();
  Serial.print(F(">> "));
  Serial.println(command);

  trm.print(command);
  trm.print('\r');

  bool success = waitFor(
    expected,
    timeoutMs
  );

  if (success) {
    Serial.println(F("\n[OK]"));
  } else {
    Serial.println(F("\n[TIMEOUT / FAIL]"));
  }

  delay(COMMAND_PAUSE_MS);

  return success;
}

bool modemResponds() {
  Serial.println();
  Serial.println(F("=== TEST MODEM RESPONSE ==="));

  return sendAT(
    "AT",
    "OK",
    5000UL
  );
}

// ============================================================
// MOBILE NETWORK REGISTRATION
// ============================================================

bool waitForNetwork() {
  Serial.println();
  Serial.println(F("=== WAIT FOR MOBILE NETWORK ==="));

  unsigned long start = millis();

  const unsigned long timeoutMs = 90000UL;

  while (millis() - start < timeoutMs) {
    flushTRM();

    trm.print(F("AT+CEREG?\r"));

    if (waitFor(
          "+CEREG:",
          5000UL
        )) {
      /*
       * Registration states:
       *
       * 1 = registered on home network
       * 5 = registered while roaming
       */
      if (
        rxBuf.indexOf(",1") >= 0 ||
        rxBuf.indexOf(",5") >= 0
      ) {
        Serial.println();
        Serial.println(
          F("Registered on mobile network.")
        );

        return true;
      }
    }

    Serial.println();
    Serial.println(
      F("Not registered yet; retrying...")
    );

    delay(3000UL);
  }

  Serial.println(
    F("Mobile network registration timed out.")
  );

  return false;
}

// ============================================================
// MQTT CONNECTION
// ============================================================

bool mqttConnect() {
  Serial.println();
  Serial.println(F("=== MQTT CONNECT ==="));

  /*
   * These cleanup commands may return ERROR when there is
   * no previous MQTT connection. That is acceptable.
   */
  sendAT(
    "AT+QMTDISC=0",
    "OK",
    5000UL
  );

  sendAT(
    "AT+QMTCLOSE=0",
    "OK",
    5000UL
  );

  char command[180];

  // ----------------------------------------------------------
  // Open broker connection
  // ----------------------------------------------------------

  snprintf(
    command,
    sizeof(command),
    "AT+QMTOPEN=0,\"%s\",%d",
    BROKER,
    BROKER_PORT
  );

  if (!sendAT(
        command,
        "+QMTOPEN: 0,0",
        45000UL
      )) {
    Serial.println(F("MQTT broker open failed."));

    return false;
  }

  // ----------------------------------------------------------
  // Authenticate MQTT client
  // ----------------------------------------------------------

  snprintf(
    command,
    sizeof(command),
    "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
    CLIENT_ID,
    MQTT_USER,
    MQTT_PASS
  );

  if (!sendAT(
        command,
        "+QMTCONN: 0,0,0",
        50000UL
      )) {
    Serial.println(F("MQTT client connection failed."));

    return false;
  }

  // ----------------------------------------------------------
  // Subscribe to command topic
  // ----------------------------------------------------------

  snprintf(
    command,
    sizeof(command),
    "AT+QMTSUB=0,1,\"%s\",0",
    TOPIC_SUB
  );

  if (!sendAT(
        command,
        "+QMTSUB:",
        15000UL
      )) {
    Serial.println(F("MQTT subscription failed."));

    return false;
  }

  /*
   * A retained command may arrive immediately with +QMTRECV while
   * sendAT()/waitFor() is still reading the subscription response.
   * At that point the command is already in rxBuf and no longer in
   * the serial input buffer, so process it here before opening the
   * normal 10-second command window.
   */
  processQmtRecvFromBuffer(rxBuf);

  Serial.println(
    F("MQTT connected and subscribed.")
  );

  return true;
}

// ============================================================
// MQTT DISCONNECT
// ============================================================

void mqttDisconnect() {
  Serial.println();
  Serial.println(F("=== MQTT DISCONNECT ==="));

  sendAT(
    "AT+QMTDISC=0",
    "OK",
    10000UL
  );

  sendAT(
    "AT+QMTCLOSE=0",
    "OK",
    10000UL
  );

  Serial.println(
    F("MQTT disconnect procedure finished.")
  );
}

// ============================================================
// MQTT COMMAND VALUE PARSING
// ============================================================

long extractCommandId(const String &command) {
  int idPosition = command.indexOf("id=");

  if (idPosition < 0) {
    return -1L;
  }

  idPosition += 3;

  int endPosition = idPosition;

  while (
    endPosition < command.length() &&
    isDigit(command.charAt(endPosition))
  ) {
    endPosition++;
  }

  if (endPosition == idPosition) {
    return -1L;
  }

  return command
    .substring(idPosition, endPosition)
    .toInt();
}

unsigned long extractUnsignedValue(
  const String &command,
  const char *key
) {
  int position = command.indexOf(key);

  if (position < 0) {
    return 0;
  }

  position += strlen(key);

  while (
    position < command.length() &&
    command.charAt(position) == ' '
  ) {
    position++;
  }

  int endPosition = position;

  while (
    endPosition < command.length() &&
    isDigit(command.charAt(endPosition))
  ) {
    endPosition++;
  }

  if (endPosition == position) {
    return 0;
  }

  return (unsigned long)command
    .substring(position, endPosition)
    .toInt();
}

// ============================================================
// MQTT COMMAND EXECUTION
// ============================================================

void executeCommand(String command) {
  command.trim();

  String originalCommand = command;
  command.toLowerCase();

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("MQTT COMMAND PROCESSING STARTED"));
  Serial.println(F("========================================"));

  Serial.print(F("Received command: "));
  Serial.println(originalCommand);

  Serial.println();
  Serial.println(F("Current configuration before command:"));

  Serial.print(F("  Procedure wait interval: "));
  Serial.print(offTimeMs / 1000UL);
  Serial.println(F(" seconds"));

  Serial.print(F("  IMU sample count: "));
  Serial.println(sampleCount);

  Serial.print(F("  IMU sample period: "));
  Serial.print(samplePeriodMs);
  Serial.println(F(" ms"));

  Serial.print(F("  Last processed command ID: "));
  Serial.println(lastCommandId);

  // ----------------------------------------------------------
  // Extract command ID
  // ----------------------------------------------------------

  long commandId = extractCommandId(command);

  Serial.println();

  if (commandId >= 0) {
    Serial.print(F("Command ID detected: "));
    Serial.println(commandId);
  } else {
    Serial.println(
      F("No valid command ID was included.")
    );

    Serial.println(
      F("The command may still execute, but it will not be protected from retained-message repetition.")
    );
  }

  // ----------------------------------------------------------
  // Duplicate-command protection
  // ----------------------------------------------------------

  if (
    commandId >= 0 &&
    commandId <= lastCommandId
  ) {
    Serial.println();
    Serial.println(F("COMMAND SKIPPED"));

    Serial.print(F("Reason: command ID "));
    Serial.print(commandId);
    Serial.print(F(" was already processed. Last stored ID is "));
    Serial.print(lastCommandId);
    Serial.println('.');

    Serial.println(
      F("No actuator or configuration change was performed.")
    );

    Serial.println(F("========================================"));

    return;
  }

  bool recognized = false;
  bool successful = false;

  // ==========================================================
  // LIGHT COMMAND
  // ==========================================================

  if (command.indexOf("light on") >= 0) {
    recognized = true;

    Serial.println();
    Serial.println(F("Command type: LIGHT CONTROL"));

    Serial.println(
      F("Requested action: turn the light ON temporarily.")
    );

    Serial.print(F("Light output pin: D"));
    Serial.println(LIGHT_PIN);

    Serial.print(F("Requested ON duration: "));
    Serial.print(ACTUATOR_ON_MS / 1000UL);
    Serial.println(F(" seconds"));

    Serial.println(
      F("The light will be turned on now. Command processing pauses during the activation period.")
    );

    turnLightOnFor(ACTUATOR_ON_MS);

    Serial.println(
      F("Light activation completed and the light was turned OFF.")
    );

    successful = true;
  }

  // ==========================================================
  // SIREN COMMAND
  // ==========================================================

  else if (command.indexOf("siren on") >= 0) {
    recognized = true;

    Serial.println();
    Serial.println(F("Command type: SIREN CONTROL"));

    Serial.println(
      F("Requested action: turn the siren ON temporarily.")
    );

    Serial.print(F("Siren output pin: D"));
    Serial.println(SIREN_PIN);

    Serial.print(F("Requested ON duration: "));
    Serial.print(ACTUATOR_ON_MS / 1000UL);
    Serial.println(F(" seconds"));

    Serial.println(
      F("The siren will be turned on now. Command processing pauses during the activation period.")
    );

    turnSirenOnFor(ACTUATOR_ON_MS);

    Serial.println(
      F("Siren activation completed and the siren was turned OFF.")
    );

    successful = true;
  }

  // ==========================================================
  // PROCEDURE INTERVAL COMMAND
  // ==========================================================

  else if (command.indexOf("interval ") >= 0) {
    recognized = true;

    Serial.println();
    Serial.println(F("Command type: PROCEDURE INTERVAL"));

    unsigned long seconds =
      extractUnsignedValue(
        command,
        "interval"
      );

    Serial.print(F("Requested new wait interval: "));
    Serial.print(seconds);
    Serial.println(F(" seconds"));

    if (seconds >= 10UL) {
      unsigned long oldIntervalSeconds =
        offTimeMs / 1000UL;

      offTimeMs = seconds * 1000UL;

      Serial.print(F("Previous wait interval: "));
      Serial.print(oldIntervalSeconds);
      Serial.println(F(" seconds"));

      Serial.print(F("New wait interval: "));
      Serial.print(offTimeMs / 1000UL);
      Serial.println(F(" seconds"));

      Serial.println(
        F("This value controls the wait after the current procedure finishes.")
      );

      Serial.println(
        F("It does not include modem startup, MQTT communication, IMU sampling, or publishing time.")
      );

      successful = true;
    } else {
      Serial.println(F("COMMAND VALUE REJECTED"));

      Serial.println(
        F("The minimum allowed interval is 10 seconds.")
      );

      Serial.println(
        F("The existing interval remains unchanged.")
      );
    }
  }

  // ==========================================================
  // SAMPLE COUNT COMMAND
  // ==========================================================

  else if (
    command.indexOf("sample_count ") >= 0
  ) {
    recognized = true;

    Serial.println();
    Serial.println(F("Command type: IMU SAMPLE COUNT"));

    unsigned long requestedCount =
      extractUnsignedValue(
        command,
        "sample_count"
      );

    Serial.print(F("Requested number of samples: "));
    Serial.println(requestedCount);

    Serial.print(F("Allowed range: 1 to "));
    Serial.println(MAX_IMU_SAMPLES);

    if (
      requestedCount >= 1UL &&
      requestedCount <= MAX_IMU_SAMPLES
    ) {
      byte oldCount = sampleCount;

      sampleCount = (byte)requestedCount;

      Serial.print(F("Previous sample count: "));
      Serial.println(oldCount);

      Serial.print(F("New sample count: "));
      Serial.println(sampleCount);

      Serial.println(
        F("This changes how many raw IMU readings are collected in one acquisition batch.")
      );

      Serial.println(
        F("A larger sample count increases RAM usage, acquisition duration, and MQTT payload size.")
      );

      successful = true;
    } else {
      Serial.println(F("COMMAND VALUE REJECTED"));

      Serial.print(
        F("The sample count must be between 1 and ")
      );

      Serial.print(MAX_IMU_SAMPLES);
      Serial.println('.');

      Serial.println(
        F("The existing sample count remains unchanged.")
      );
    }
  }

  // ==========================================================
  // SAMPLE PERIOD COMMAND
  // ==========================================================

  else if (
    command.indexOf("sample_period ") >= 0
  ) {
    recognized = true;

    Serial.println();
    Serial.println(F("Command type: IMU SAMPLE PERIOD"));

    unsigned long seconds =
      extractUnsignedValue(
        command,
        "sample_period"
      );

    Serial.print(F("Requested sample period: "));
    Serial.print(seconds);
    Serial.println(F(" seconds"));

    if (
      seconds >= 1UL &&
      seconds <= 3600UL
    ) {
      unsigned long oldPeriodMs =
        samplePeriodMs;

      samplePeriodMs =
        seconds * 1000UL;

      Serial.print(F("Previous sample period: "));
      Serial.print(oldPeriodMs);
      Serial.println(F(" ms"));

      Serial.print(F("New sample period: "));
      Serial.print(samplePeriodMs);
      Serial.println(F(" ms"));

      Serial.println(
        F("This is the delay between consecutive IMU samples.")
      );

      unsigned long approximateDurationMs = 0;

      if (sampleCount > 1) {
        approximateDurationMs =
          ((unsigned long)sampleCount - 1UL) *
          samplePeriodMs;
      }

      Serial.print(
        F("Approximate acquisition duration with the current sample count: ")
      );

      Serial.print(
        approximateDurationMs / 1000UL
      );

      Serial.println(F(" seconds"));

      Serial.println(
        F("Changing the sample period changes acquisition duration but does not significantly change MQTT message size.")
      );

      successful = true;
    } else {
      Serial.println(F("COMMAND VALUE REJECTED"));

      Serial.println(
        F("The sample period must be between 1 and 3600 seconds.")
      );

      Serial.println(
        F("The existing sample period remains unchanged.")
      );
    }
  }

  // ==========================================================
  // UNKNOWN COMMAND
  // ==========================================================

  else {
    Serial.println();
    Serial.println(F("UNKNOWN MQTT COMMAND"));

    Serial.print(F("Unsupported payload: "));
    Serial.println(originalCommand);

    Serial.println();
    Serial.println(F("Supported commands are:"));

    Serial.println(
      F("  light on id=<number>")
    );

    Serial.println(
      F("  siren on id=<number>")
    );

    Serial.println(
      F("  interval <seconds> id=<number>")
    );

    Serial.println(
      F("  sample_count <number> id=<number>")
    );

    Serial.println(
      F("  sample_period <seconds> id=<number>")
    );

    Serial.println(
      F("No actuator or configuration change was performed.")
    );
  }

  // ----------------------------------------------------------
  // Store command ID only after a successful command
  // ----------------------------------------------------------

  if (
    recognized &&
    successful &&
    commandId >= 0
  ) {
    saveLastCommandId(commandId);

    Serial.println();

    Serial.print(F("Command ID "));
    Serial.print(commandId);
    Serial.println(
      F(" was successfully saved to EEPROM.")
    );

    Serial.println(
      F("A retained MQTT message with this ID will not execute again.")
    );
  }

  else if (
    recognized &&
    !successful
  ) {
    Serial.println();

    Serial.println(
      F("The command ID was not saved because the command was not executed successfully.")
    );
  }

  else if (
    recognized &&
    successful &&
    commandId < 0
  ) {
    Serial.println();

    Serial.println(
      F("The command executed successfully, but no ID was stored because the payload did not include a valid id=<number> field.")
    );
  }

  // ----------------------------------------------------------
  // Final state
  // ----------------------------------------------------------

  Serial.println();
  Serial.println(F("Configuration after command:"));

  Serial.print(F("  Procedure wait interval: "));
  Serial.print(offTimeMs / 1000UL);
  Serial.println(F(" seconds"));

  Serial.print(F("  IMU sample count: "));
  Serial.println(sampleCount);

  Serial.print(F("  IMU sample period: "));
  Serial.print(samplePeriodMs);
  Serial.println(F(" ms"));

  unsigned long approximateDurationMs = 0;

  if (sampleCount > 1) {
    approximateDurationMs =
      ((unsigned long)sampleCount - 1UL) *
      samplePeriodMs;
  }

  Serial.print(
    F("  Approximate IMU acquisition duration: ")
  );

  Serial.print(
    approximateDurationMs / 1000UL
  );

  Serial.println(F(" seconds"));

  Serial.print(F("  Last stored command ID: "));
  Serial.println(lastCommandId);

  Serial.println();

  if (successful) {
    Serial.println(
      F("MQTT COMMAND COMPLETED SUCCESSFULLY.")
    );
  } else if (recognized) {
    Serial.println(
      F("MQTT COMMAND WAS RECOGNIZED BUT NOT EXECUTED.")
    );
  } else {
    Serial.println(
      F("MQTT COMMAND WAS NOT RECOGNIZED.")
    );
  }

  Serial.println(F("========================================"));
}

// ============================================================
// MQTT RECEIVED-MESSAGE PARSING
// ============================================================

void processQmtRecvLine(const String &line) {
  /*
   * Expected Quectel-style URC:
   *
   * +QMTRECV: 0,0,"threenovate/cmd","command payload"
   *
   * This extracts the final quoted value.
   */
  int lastQuote = line.lastIndexOf('"');

  if (lastQuote < 1) {
    Serial.println(
      F("Could not find MQTT payload closing quote.")
    );

    return;
  }

  int previousQuote =
    line.lastIndexOf('"', lastQuote - 1);

  if (
    previousQuote < 0 ||
    previousQuote >= lastQuote
  ) {
    Serial.println(
      F("Could not find MQTT payload opening quote.")
    );

    return;
  }

  String payload = line.substring(
    previousQuote + 1,
    lastQuote
  );

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("MQTT COMMAND RECEIVED"));
  Serial.println(F("========================================"));

  Serial.print(F("Payload: "));
  Serial.println(payload);

  executeCommand(payload);
}

// ============================================================
// MQTT MESSAGES CAPTURED DURING AT COMMANDS
// ============================================================

void processQmtRecvFromBuffer(const String &buffer) {
  int searchPosition = 0;

  while (true) {
    int lineStart = buffer.indexOf(
      "+QMTRECV:",
      searchPosition
    );

    if (lineStart < 0) {
      break;
    }

    int lineEnd = buffer.indexOf('\n', lineStart);

    if (lineEnd < 0) {
      lineEnd = buffer.length();
    }

    String line = buffer.substring(
      lineStart,
      lineEnd
    );

    line.trim();

    Serial.println();
    Serial.println(
      F("MQTT command detected during AT response.")
    );

    processQmtRecvLine(line);

    searchPosition = lineEnd + 1;
  }
}

// ============================================================
// MQTT COMMAND LISTEN WINDOW
// ============================================================

void listenForMqttCommands(
  unsigned long listenMs
) {
  Serial.println();
  Serial.println(F("=== MQTT COMMAND WINDOW ==="));

  Serial.print(F("Listening for "));
  Serial.print(listenMs / 1000UL);
  Serial.println(F(" seconds..."));

  String line;

  unsigned long start = millis();

  while (millis() - start < listenMs) {
    while (trm.available()) {
      char c = trm.read();

      Serial.write(c);

      if (c == '\n') {
        line.trim();

        if (line.indexOf("+QMTRECV:") >= 0) {
          processQmtRecvLine(line);
        }

        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  /*
   * Process a final line even if it did not end with newline.
   */
  line.trim();

  if (line.indexOf("+QMTRECV:") >= 0) {
    processQmtRecvLine(line);
  }

  Serial.println();
  Serial.println(F("Command window closed."));
}

// ============================================================
// IMU TEMPERATURE OUTPUT
// ============================================================

void printTemperatureC(
  Print &output,
  int16_t rawTemperature
) {
  /*
   * MPU6050 conversion:
   *
   * temperature = raw / 340 + 36.53
   *
   * Integer hundredths are used to avoid float formatting
   * overhead on the Arduino Nano.
   */
  long hundredths =
    ((long)rawTemperature * 100L) /
    340L +
    3653L;

  long whole = hundredths / 100L;
  long fraction = hundredths % 100L;

  if (fraction < 0) {
    fraction = -fraction;
  }

  output.print(whole);
  output.print('.');

  if (fraction < 10L) {
    output.print('0');
  }

  output.print(fraction);
}

// ============================================================
// STREAM JSON PAYLOAD
// ============================================================

void streamImuPayload(
  Print &output,
  byte actualCount
) {
  /*
   * JSON is streamed directly to the modem.
   *
   * This avoids building a large JSON String in Nano SRAM.
   */
  output.print(F("{\"device\":\""));
  output.print(CLIENT_ID);

  output.print(F("\",\"cycle\":"));
  output.print(cycleNumber);

  output.print(F(",\"sample_count\":"));
  output.print(actualCount);

  output.print(F(",\"sample_period_ms\":"));
  output.print(samplePeriodMs);

  output.print(F(",\"samples\":["));

  for (byte i = 0; i < actualCount; i++) {
    if (i > 0) {
      output.print(',');
    }

    output.print(F("{\"n\":"));
    output.print(i);

    output.print(F(",\"t_ms\":"));
    output.print(samples[i].relativeTimeMs);

    output.print(F(",\"ax\":"));
    output.print(samples[i].ax);

    output.print(F(",\"ay\":"));
    output.print(samples[i].ay);

    output.print(F(",\"az\":"));
    output.print(samples[i].az);

    output.print(F(",\"gx\":"));
    output.print(samples[i].gx);

    output.print(F(",\"gy\":"));
    output.print(samples[i].gy);

    output.print(F(",\"gz\":"));
    output.print(samples[i].gz);

    output.print(F(",\"temp_c\":"));

    printTemperatureC(
      output,
      samples[i].temperatureRaw
    );

    output.print('}');
  }

  output.print(F("]}"));
}

// ============================================================
// MQTT PUBLISH IMU DATA
// ============================================================

bool mqttPublishImuSamples(byte actualCount) {
  if (actualCount == 0) {
    Serial.println(
      F("No IMU samples available to publish.")
    );

    return false;
  }

  Serial.println();
  Serial.println(F("=== MQTT PUBLISH IMU DATA ==="));

  char command[100];

  snprintf(
    command,
    sizeof(command),
    "AT+QMTPUB=0,0,0,0,\"%s\"",
    TOPIC_PUB
  );

  flushTRM();

  Serial.println();
  Serial.print(F(">> "));
  Serial.println(command);

  trm.print(command);
  trm.print('\r');

  /*
   * Wait for the modem's payload prompt.
   */
  if (!waitFor(
        ">",
        10000UL
      )) {
    Serial.println();
    Serial.println(
      F("No MQTT publish prompt received.")
    );

    return false;
  }

  Serial.println();
  Serial.println(F("Sending JSON payload..."));

  streamImuPayload(
    trm,
    actualCount
  );

  /*
   * Ctrl+Z terminates the payload.
   */
  trm.write(0x1A);

  rxBuf = "";

  if (!waitFor(
        "+QMTPUB:",
        25000UL
      )) {
    Serial.println();
    Serial.println(
      F("MQTT publish confirmation not received.")
    );

    return false;
  }

  Serial.println();
  Serial.println(
    F("IMU MQTT MESSAGE SUCCESSFULLY SENT.")
  );

  return true;
}

// ============================================================
// ONE COMPLETE OPERATING PROCEDURE
// ============================================================

bool runProcedure() {
  cycleNumber++;

  Serial.println();
  Serial.println(F("========================================"));

  Serial.print(F("=== START PROCEDURE CYCLE "));
  Serial.print(cycleNumber);
  Serial.println(F(" ==="));

  Serial.println(F("========================================"));

  /*
   * Ensure actuators begin off.
   */
  turnLightOff();
  turnSirenOff();

  // ----------------------------------------------------------
  // 1. TURN ON TRM
  // ----------------------------------------------------------

  powerOnTRM();

  // ----------------------------------------------------------
  // 2. TEST MODEM COMMUNICATION
  // ----------------------------------------------------------

  if (!modemResponds()) {
    Serial.println();
    Serial.println(F("TRM142 did not respond."));

    sleepImu();
    powerOffTRM();

    return false;
  }

  // ----------------------------------------------------------
  // 3. WAIT FOR MOBILE NETWORK
  // ----------------------------------------------------------

  if (!waitForNetwork()) {
    sleepImu();
    powerOffTRM();

    return false;
  }

  // ----------------------------------------------------------
  // 4. CONNECT AND SUBSCRIBE TO MQTT
  // ----------------------------------------------------------

  if (!mqttConnect()) {
    sleepImu();
    powerOffTRM();

    return false;
  }

  // ----------------------------------------------------------
  // 5. CHECK AND EXECUTE MQTT COMMANDS
  // ----------------------------------------------------------

  /*
   * A retained command sent while the modem was offline
   * should arrive during this window.
   */
  listenForMqttCommands(COMMAND_LISTEN_MS);

  // ----------------------------------------------------------
  // 6. WAKE THE IMU
  // ----------------------------------------------------------

  bool imuReady = wakeImu();

  byte actualSamples = 0;

  // ----------------------------------------------------------
  // 7. COLLECT IMU MEASUREMENTS
  // ----------------------------------------------------------

  if (imuReady) {
    actualSamples = collectImuSamples();
  }

  // ----------------------------------------------------------
  // 8. SEND IMU MEASUREMENTS
  // ----------------------------------------------------------

  bool published =
    mqttPublishImuSamples(actualSamples);

  // ----------------------------------------------------------
  // 9. PUT IMU INTO SLEEP MODE
  // ----------------------------------------------------------

  if (imuReady) {
    sleepImu();
  }

  // ----------------------------------------------------------
  // 10. DISCONNECT MQTT
  // ----------------------------------------------------------

  mqttDisconnect();

  // ----------------------------------------------------------
  // 11. ATTEMPT TO TURN OFF TRM
  // ----------------------------------------------------------

  powerOffTRM();

  /*
   * Reassert safe actuator states.
   */
  turnLightOff();
  turnSirenOff();

  Serial.println();
  Serial.println(F("========================================"));

  if (published) {
    Serial.println(
      F("PROCEDURE COMPLETED SUCCESSFULLY.")
    );
  } else {
    Serial.println(
      F("PROCEDURE COMPLETED WITH AN ERROR.")
    );
  }

  Serial.println(F("========================================"));

  return published;
}

// ============================================================
// FIVE-MINUTE WAIT
// ============================================================

void waitOffPeriod() {
  Serial.println();
  Serial.println(F("========================================"));

  Serial.print(
    F("Waiting with IMU asleep and TRM OFF for ")
  );

  Serial.print(offTimeMs / 1000UL);
  Serial.println(F(" seconds."));

  Serial.println(F("========================================"));

  /*
   * Apply safe outputs once before entering the wait loop.
   *
   * Do not repeatedly call turnLightOff() here because WS2812
   * transmission temporarily disables interrupts and can disturb
   * AltSoftSerial communication.
   */
  turnLightOff();
  turnSirenOff();

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  unsigned long start = millis();
  int lastPrinted = -1;

  while (millis() - start < offTimeMs) {
    unsigned long elapsed = millis() - start;
    unsigned long remainingMs = offTimeMs - elapsed;

    if (remainingMs <= 10000UL) {
      int remainingSeconds =
        (remainingMs + 999UL) / 1000UL;

      if (remainingSeconds != lastPrinted) {
        lastPrinted = remainingSeconds;

        Serial.print(F("Starting next cycle in "));
        Serial.print(remainingSeconds);
        Serial.println(F("..."));
      }
    }

    delay(100UL);
  }

  Serial.println();
  Serial.println(F("Starting next procedure."));
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);

  trm.begin(TRM_BAUD);

  Wire.begin();

  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(LIGHT_DATA_PIN, OUTPUT);
  pinMode(SIREN_PIN, OUTPUT);
  pinMode(TRM_POWER_PIN, OUTPUT);

  /*
   * WS2812 data line must idle LOW.
   */
  digitalWrite(LIGHT_DATA_PIN, LOW);

  /*
   * Begin in a safe state.
   */
  turnLightOff();
  turnSirenOff();

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  delay(1500UL);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("=== SMART BUOY CYCLIC IMU + MQTT TEST ==="));
  Serial.println(F("========================================"));

  Serial.print(F("Broker: "));
  Serial.print(BROKER);
  Serial.print(':');
  Serial.println(BROKER_PORT);

  Serial.print(F("Publish topic: "));
  Serial.println(TOPIC_PUB);

  Serial.print(F("Command topic: "));
  Serial.println(TOPIC_SUB);

  Serial.print(F("Light output pin: D"));
  Serial.println(LIGHT_PIN);

  Serial.print(F("Light data/control pin: D"));
  Serial.println(LIGHT_DATA_PIN);

  Serial.print(F("Default wait: "));
  Serial.print(offTimeMs / 1000UL);
  Serial.println(F(" seconds"));

  Serial.print(F("Default sample count: "));
  Serial.println(sampleCount);

  Serial.print(F("Default sample period: "));
  Serial.print(samplePeriodMs);
  Serial.println(F(" ms"));

  loadLastCommandId();

  /*
   * Ensure the IMU begins asleep.
   */
  if (imuExists()) {
    sleepImu();
  } else {
    Serial.println(
      F("MPU6050 not detected during setup.")
    );
  }

  Serial.println();
  Serial.println(
    F("The first procedure will run immediately.")
  );
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  /*
   * Run immediately after startup.
   */
  runProcedure();

  /*
   * After the procedure finishes, remain off for five minutes.
   */
  waitOffPeriod();
}
