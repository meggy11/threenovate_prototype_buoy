#include <AltSoftSerial.h>

AltSoftSerial trm; // Arduino Nano: RX = D8, TX = D9

// ============================================================
// SERIAL CONFIGURATION
// ============================================================

#define USB_BAUD 115200
#define TRM_BAUD 19200

// ============================================================
// TRM142 POWER CONTROL
// ============================================================

#define TRM_POWER_PIN 4

#define MOSFET_ON_LEVEL HIGH
#define MOSFET_OFF_LEVEL LOW

const unsigned long TRM_BOOT_DELAY_MS = 20000UL;
const unsigned long COMMAND_PAUSE_MS = 1000UL;

// ============================================================
// MQTT CONFIGURATION
// ============================================================

const char* BROKER = "31.147.205.19";
const int BROKER_PORT = 1883;

const char* MQTT_USER = "threenovate";
const char* MQTT_PASS = "threenovate";

const char* CLIENT_ID = "nanoTRM142test02";

const char* TOPIC = "threenovate/test";

const char* PAYLOAD =
  "{\"device\":\"nanoTRM142test02\",\"message\":\"hello from Arduino Nano\"}";

// ============================================================
// RECEIVE BUFFER
// ============================================================

String rxBuf = "";

// ============================================================
// TRM142 POWER
// ============================================================

void powerOnTRM() {
  Serial.println();
  Serial.println(F("=== POWER ON TRM142 ==="));

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_ON_LEVEL
  );

  Serial.print(F("Waiting "));
  Serial.print(TRM_BOOT_DELAY_MS / 1000UL);
  Serial.println(F(" seconds for modem boot..."));

  delay(TRM_BOOT_DELAY_MS);

  Serial.println(F("TRM142 boot delay finished."));
}

void powerOffTRM() {
  Serial.println();
  Serial.println(F("=== POWER OFF TRM142 ==="));

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  delay(500);

  Serial.println(F("TRM142 power disabled."));
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
         * Wait briefly for trailing characters.
         */
        unsigned long quietStart = millis();

        while (millis() - quietStart < 500) {
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
// BASIC MODEM CHECKS
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
// MQTT TEST
// ============================================================

bool mqttTest() {
  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F("=== MQTT TEST ==="));
  Serial.println(F("=============================="));

  /*
   * These commands may return ERROR if no old MQTT session
   * exists. That is not necessarily a problem.
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

  char command[200];

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

  if (
    !sendAT(
      command,
      "+QMTOPEN: 0,0",
      40000
    )
  ) {
    Serial.println(F("MQTT open failed."));

    return false;
  }

  // ----------------------------------------------------------
  // Authenticate and connect
  // ----------------------------------------------------------

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
      20000
    )
  ) {
    Serial.println(F("MQTT connection failed."));

    return false;
  }

  // ----------------------------------------------------------
  // Subscribe
  // ----------------------------------------------------------

  snprintf(
    command,
    sizeof(command),
    "AT+QMTSUB=0,1,\"%s\",0",
    TOPIC
  );

  if (
    !sendAT(
      command,
      "+QMTSUB:",
      15000
    )
  ) {
    Serial.println(F("MQTT subscription failed."));

    return false;
  }

  delay(2000);

  // ----------------------------------------------------------
  // Publish
  // ----------------------------------------------------------

  snprintf(
    command,
    sizeof(command),
    "AT+QMTPUB=0,0,0,0,\"%s\"",
    TOPIC
  );

  flushTRM();

  Serial.println();
  Serial.print(F(">> "));
  Serial.println(command);

  trm.print(command);
  trm.print('\r');

  if (!waitFor(">", 10000)) {
    Serial.println(F("\nNo MQTT publish prompt received."));

    return false;
  }

  Serial.println();
  Serial.println(F("Sending payload:"));
  Serial.println(PAYLOAD);

  trm.print(PAYLOAD);

  /*
   * Ctrl+Z ends the payload.
   */
  trm.write(0x1A);

  if (
    !waitFor(
      "+QMTPUB:",
      20000
    )
  ) {
    Serial.println(
      F("\nMQTT publish confirmation not received.")
    );

    return false;
  }

  Serial.println();
  Serial.println(F("MQTT publish confirmed."));

  // ----------------------------------------------------------
  // Wait for subscribed message
  // ----------------------------------------------------------

  Serial.println();
  Serial.println(
    F("Waiting up to 15 seconds for +QMTRECV...")
  );

  rxBuf = "";

  readTRM(15000);

  if (rxBuf.indexOf("+QMTRECV:") >= 0) {
    Serial.println();
    Serial.println(F("MQTT receive successful."));
  } else {
    Serial.println();
    Serial.println(
      F("Publish succeeded, but no subscribed message was received.")
    );
  }

  // ----------------------------------------------------------
  // Disconnect
  // ----------------------------------------------------------

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

  return true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);
  trm.begin(TRM_BAUD);

  pinMode(
    TRM_POWER_PIN,
    OUTPUT
  );

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  delay(1500);

  Serial.println();
  Serial.println(F("======================================"));
  Serial.println(F("=== TRM142 MQTT AUTOMATIC TEST ==="));
  Serial.println(F("======================================"));

  Serial.println(F("USB serial: 115200"));
  Serial.println(F("TRM serial: 19200"));
  Serial.println(F("Nano RX: D8"));
  Serial.println(F("Nano TX: D9"));
  Serial.println(F("TRM MOSFET: D4"));

  Serial.print(F("Broker: "));
  Serial.print(BROKER);
  Serial.print(':');
  Serial.println(BROKER_PORT);

  Serial.print(F("Topic: "));
  Serial.println(TOPIC);

  powerOnTRM();

  bool basicSuccess = basicChecks();

  if (!basicSuccess) {
    Serial.println();
    Serial.println(
      F("One or more basic modem checks failed.")
    );

    Serial.println(
      F("MQTT test will still be attempted.")
    );
  }

  bool mqttSuccess = mqttTest();

  Serial.println();
  Serial.println(F("=============================="));

  if (mqttSuccess) {
    Serial.println(F("MQTT TEST SUCCESSFUL"));
  } else {
    Serial.println(F("MQTT TEST FAILED"));
  }

  Serial.println(F("=============================="));

  Serial.println();
  Serial.println(F("=== MANUAL TERMINAL MODE ==="));
  Serial.println(F("Serial Monitor baud: 115200"));
  Serial.println(F("Line ending: Carriage Return"));
  Serial.println(F("You may now enter AT commands manually."));

  /*
   * TRM remains powered on for manual testing.
   * Call powerOffTRM() here if you want it switched off
   * after the automatic test.
   */
}

// ============================================================
// LOOP — MANUAL TERMINAL
// ============================================================

void loop() {
  /*
   * Serial Monitor -> TRM142
   */
  while (Serial.available()) {
    char c = Serial.read();

    trm.write(c);
  }

  /*
   * TRM142 -> Serial Monitor
   */
  while (trm.available()) {
    char c = trm.read();

    Serial.write(c);
  }
}
