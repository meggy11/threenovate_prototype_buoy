#include <AltSoftSerial.h>

AltSoftSerial trm; // Arduino Nano: RX = D8, TX = D9

// ============================================================
// SERIAL CONFIGURATION
// ============================================================

#define USB_BAUD 115200
#define TRM_BAUD 19200

// ============================================================
// MOSFET OUTPUTS
// ============================================================

#define LIGHT_PIN 3
#define SIREN_PIN 2
#define TRM_POWER_PIN 4

/*
 * Current assumption:
 *
 * HIGH = MOSFET/device ON
 * LOW  = MOSFET/device OFF
 *
 * If MOSFET boards are active-low:
 *
 * #define MOSFET_ON_LEVEL LOW
 * #define MOSFET_OFF_LEVEL HIGH
 */
#define MOSFET_ON_LEVEL HIGH
#define MOSFET_OFF_LEVEL LOW

// ============================================================
// TIMING
// ============================================================

// Time given to TRM142 to boot after power-on.
const unsigned long TRM_BOOT_DELAY_MS = 20000UL;

// Delay between individual AT commands.
const unsigned long COMMAND_PAUSE_MS = 1000UL;

// Time all outputs remain off between cycles.
const unsigned long OFF_TIME_MS = 60000UL;

// ============================================================
// MQTT CONFIGURATION
// ============================================================

const char BROKER[] = "161.53.19.50";
const int BROKER_PORT = 18830;

const char CLIENT_ID[] = "nanoTRM142node01";

const char MQTT_USER[] = "testing";
const char MQTT_PASS[] = "11091995";

const char TOPIC_PUB[] = "threenovate/test";
const char TOPIC_SUB[] = "threenovate/cmd";

const char PAYLOAD[] =
  "{\"device\":\"nanoTRM142node01\","
  "\"status\":\"cyclic MQTT test\","
  "\"message\":\"hello from Arduino Nano and TRM142\"}";

// ============================================================
// RECEIVE BUFFER
// ============================================================

String rxBuf = "";

// ============================================================
// CYCLE COUNTER
// ============================================================

unsigned long cycleNumber = 0;

// ============================================================
// OUTPUT CONTROL
// ============================================================

void turnLightOff() {
  digitalWrite(LIGHT_PIN, MOSFET_OFF_LEVEL);
}

void turnSirenOff() {
  digitalWrite(SIREN_PIN, MOSFET_OFF_LEVEL);
}

void powerOffTRM() {
  Serial.println();
  Serial.println(F("=== POWER OFF TRM142 ==="));

  /*
   * Print any remaining modem response before power removal.
   */
  while (trm.available()) {
    Serial.write(trm.read());
  }

  digitalWrite(TRM_POWER_PIN, MOSFET_OFF_LEVEL);

  delay(500);

  Serial.println(F("TRM142 power OFF."));
}

void turnAllOutputsOff() {
  Serial.println();
  Serial.println(F("Turning all controlled outputs OFF..."));

  turnLightOff();
  turnSirenOff();
  powerOffTRM();

  Serial.println(F("Light OFF."));
  Serial.println(F("Siren OFF."));
  Serial.println(F("TRM142 OFF."));
}

// ============================================================
// TRM142 POWER ON
// ============================================================

void powerOnTRM() {
  Serial.println();
  Serial.println(F("=== POWER ON TRM142 ==="));

  digitalWrite(TRM_POWER_PIN, MOSFET_ON_LEVEL);

  Serial.print(F("Waiting "));
  Serial.print(TRM_BOOT_DELAY_MS / 1000UL);
  Serial.println(F(" seconds for modem startup..."));

  delay(TRM_BOOT_DELAY_MS);

  Serial.println(F("TRM142 startup delay finished."));
}

// ============================================================
// SERIAL HELPERS
// ============================================================

void flushTRM() {
  while (trm.available()) {
    trm.read();
  }

  rxBuf = "";
}

void readTRM(unsigned long milliseconds = 5000) {
  unsigned long start = millis();

  while (millis() - start < milliseconds) {
    while (trm.available()) {
      char c = trm.read();

      rxBuf += c;
      Serial.write(c);
    }
  }
}

bool waitFor(
  const char* expected,
  unsigned long timeout = 10000
) {
  unsigned long start = millis();

  while (millis() - start < timeout) {
    while (trm.available()) {
      char c = trm.read();

      rxBuf += c;
      Serial.write(c);

      if (rxBuf.indexOf(expected) >= 0) {
        /*
         * Read trailing characters until the modem has been
         * quiet for 500 ms.
         */
        unsigned long quietStart = millis();

        while (millis() - quietStart < 500UL) {
          while (trm.available()) {
            char trailing = trm.read();

            rxBuf += trailing;
            Serial.write(trailing);

            quietStart = millis();
          }
        }

        return true;
      }
    }
  }

  return false;
}

bool sendAT(
  const char* command,
  const char* expected = "OK",
  unsigned long timeout = 10000
) {
  flushTRM();

  Serial.println();
  Serial.print(F(">> "));
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
    Serial.println(F("\n[TIMEOUT / FAIL]"));
  }

  delay(COMMAND_PAUSE_MS);

  return success;
}

// ============================================================
// BASIC TRM142 CHECKS
// ============================================================

bool basicChecks() {
  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F("=== BASIC TRM142 CHECKS ==="));
  Serial.println(F("=============================="));

  bool success = true;

  success &= sendAT(
    "AT",
    "OK",
    10000
  );

  success &= sendAT(
    "ATI",
    "OK",
    10000
  );

  success &= sendAT(
    "AT+IPR?",
    "+IPR:",
    10000
  );

  success &= sendAT(
    "AT+CPIN?",
    "READY",
    15000
  );

  success &= sendAT(
    "AT+CSQ",
    "OK",
    10000
  );

  success &= sendAT(
    "AT+CEREG?",
    "+CEREG:",
    15000
  );

  success &= sendAT(
    "AT+COPS?",
    "OK",
    20000
  );

  success &= sendAT(
    "AT+QNWINFO",
    "OK",
    15000
  );

  success &= sendAT(
    "AT+CGPADDR=1",
    "+CGPADDR:",
    15000
  );

  return success;
}

// ============================================================
// MQTT CONNECTION
// ============================================================

bool mqttConnect() {
  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F("=== MQTT CONNECT ==="));
  Serial.println(F("=============================="));

  /*
   * These commands may return ERROR if no previous MQTT
   * session exists. That does not necessarily indicate a fault.
   */
  sendAT(
    "AT+QMTDISC=0",
    "OK",
    5000
  );

  sendAT(
    "AT+QMTCLOSE=0",
    "OK",
    5000
  );

  char command[220];

  // Open network connection to MQTT broker.
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
        40000
      )) {
    Serial.println(F("MQTT open failed."));
    return false;
  }

  // Connect using username and password.
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
        20000
      )) {
    Serial.println(F("MQTT connection failed."));
    return false;
  }

  // Subscribe to the command topic.
  snprintf(
    command,
    sizeof(command),
    "AT+QMTSUB=0,1,\"%s\",0",
    TOPIC_SUB
  );

  if (!sendAT(
        command,
        "+QMTSUB:",
        15000
      )) {
    Serial.println(F("MQTT subscription failed."));
    return false;
  }

  Serial.println(F("MQTT connected and subscribed."));

  return true;
}

// ============================================================
// MQTT PUBLISH
// ============================================================

bool mqttPublishTestMessage() {
  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F("=== MQTT PUBLISH ==="));
  Serial.println(F("=============================="));

  char command[180];

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

  if (!waitFor(">", 10000)) {
    Serial.println();
    Serial.println(F("No MQTT publish prompt received."));
    return false;
  }

  Serial.println();
  Serial.println(F("Sending payload:"));
  Serial.println(PAYLOAD);

  trm.print(PAYLOAD);

  /*
   * Ctrl+Z finishes the MQTT payload.
   */
  trm.write(0x1A);

  if (!waitFor(
        "+QMTPUB:",
        20000
      )) {
    Serial.println();
    Serial.println(F("MQTT publish confirmation not received."));
    return false;
  }

  Serial.println();
  Serial.println(F("MQTT MESSAGE SUCCESSFULLY SENT."));

  return true;
}

// ============================================================
// OPTIONAL COMMAND LISTENING
// ============================================================

void listenForMqttCommands(
  unsigned long listenMilliseconds
) {
  Serial.println();
  Serial.print(F("Listening on "));
  Serial.print(TOPIC_SUB);
  Serial.print(F(" for "));
  Serial.print(listenMilliseconds / 1000UL);
  Serial.println(F(" seconds..."));

  rxBuf = "";

  readTRM(listenMilliseconds);

  if (rxBuf.indexOf("+QMTRECV:") >= 0) {
    Serial.println();
    Serial.println(F("MQTT command/message received."));
  } else {
    Serial.println();
    Serial.println(F("No MQTT command received."));
  }
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
    10000
  );

  sendAT(
    "AT+QMTCLOSE=0",
    "OK",
    10000
  );

  Serial.println(F("MQTT disconnect procedure finished."));
}

// ============================================================
// ONE COMPLETE TEST CYCLE
// ============================================================

bool runTestCycle() {
  cycleNumber++;

  Serial.println();
  Serial.println(F("======================================"));
  Serial.print(F("=== STARTING TEST CYCLE "));
  Serial.print(cycleNumber);
  Serial.println(F(" ==="));
  Serial.println(F("======================================"));

  unsigned long cycleStart = millis();

  /*
   * Keep light and siren off for the entire test.
   */
  turnLightOff();
  turnSirenOff();

  /*
   * Turn on only the 4G modem.
   */
  powerOnTRM();

  bool checksSuccessful = basicChecks();

  if (!checksSuccessful) {
    Serial.println();
    Serial.println(
      F("WARNING: One or more basic checks failed.")
    );

    Serial.println(
      F("MQTT connection will still be attempted.")
    );
  }

  bool connected = mqttConnect();

  if (!connected) {
    Serial.println();
    Serial.println(F("Cycle failed during MQTT connection."));

    turnAllOutputsOff();

    return false;
  }

  /*
   * Give the modem a short moment after subscription.
   */
  delay(2000);

  bool published = mqttPublishTestMessage();

  /*
   * Optional: wait briefly for a command or another MQTT URC.
   * Remove this line if you want the shortest possible ON time.
   */
  listenForMqttCommands(5000);

  mqttDisconnect();

  /*
   * Shut everything down after communication.
   */
  turnAllOutputsOff();

  unsigned long cycleDuration =
    millis() - cycleStart;

  Serial.println();
  Serial.println(F("======================================"));

  if (published) {
    Serial.println(F("TEST CYCLE COMPLETED SUCCESSFULLY."));
  } else {
    Serial.println(F("TEST CYCLE COMPLETED WITH AN ERROR."));
  }

  Serial.print(F("Cycle duration: "));
  Serial.print(cycleDuration / 1000UL);
  Serial.println(F(" seconds."));

  Serial.println(F("All outputs are OFF."));
  Serial.println(F("======================================"));

  return published;
}

// ============================================================
// OFF PERIOD
// ============================================================

void waitOffPeriod() {
  Serial.println();
  Serial.print(F("Waiting with all outputs OFF for "));
  Serial.print(OFF_TIME_MS / 1000UL);
  Serial.println(F(" seconds..."));

  unsigned long start = millis();
  unsigned long lastPrint = 0;

  while (millis() - start < OFF_TIME_MS) {
    /*
     * Reassert the safe OFF states during the entire wait.
     */
    digitalWrite(LIGHT_PIN, MOSFET_OFF_LEVEL);
    digitalWrite(SIREN_PIN, MOSFET_OFF_LEVEL);
    digitalWrite(TRM_POWER_PIN, MOSFET_OFF_LEVEL);

    /*
     * Print remaining time every 10 seconds.
     */
    unsigned long elapsed = millis() - start;

    if (
      elapsed - lastPrint >= 10000UL
      || lastPrint == 0
    ) {
      lastPrint = elapsed;

      unsigned long remaining =
        (OFF_TIME_MS - elapsed + 999UL) / 1000UL;

      Serial.print(F("Remaining OFF time: "));
      Serial.print(remaining);
      Serial.println(F(" seconds."));
    }

    delay(100);
  }

  Serial.println(F("OFF period finished."));
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);
  trm.begin(TRM_BAUD);

  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(SIREN_PIN, OUTPUT);
  pinMode(TRM_POWER_PIN, OUTPUT);

  /*
   * Start with all loads switched off.
   */
  digitalWrite(LIGHT_PIN, MOSFET_OFF_LEVEL);
  digitalWrite(SIREN_PIN, MOSFET_OFF_LEVEL);
  digitalWrite(TRM_POWER_PIN, MOSFET_OFF_LEVEL);

  delay(2000);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("=== CYCLIC TRM142 MQTT POWER TEST ==="));
  Serial.println(F("========================================"));

  Serial.println(F("Arduino Nano USB serial: 115200"));
  Serial.println(F("TRM142 serial: 19200"));
  Serial.println(F("Nano RX: D8"));
  Serial.println(F("Nano TX: D9"));

  Serial.println(F("Light MOSFET: D3"));
  Serial.println(F("Siren MOSFET: D2"));
  Serial.println(F("TRM142 power MOSFET: D4"));

  Serial.print(F("Broker: "));
  Serial.print(BROKER);
  Serial.print(':');
  Serial.println(BROKER_PORT);

  Serial.print(F("Publish topic: "));
  Serial.println(TOPIC_PUB);

  Serial.print(F("Subscribe topic: "));
  Serial.println(TOPIC_SUB);

  Serial.print(F("TRM boot delay: "));
  Serial.print(TRM_BOOT_DELAY_MS / 1000UL);
  Serial.println(F(" seconds"));

  Serial.print(F("OFF period: "));
  Serial.print(OFF_TIME_MS / 1000UL);
  Serial.println(F(" seconds"));
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  runTestCycle();

  waitOffPeriod();
}
