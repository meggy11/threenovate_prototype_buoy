#include <AltSoftSerial.h>

AltSoftSerial trm;  // Arduino Nano: RX = D8, TX = D9

// ============================================================
// CONFIGURATION
// ============================================================

#define USB_BAUD 115200
#define TRM_BAUD 19200

#define TRM_POWER_PIN 4

/*
 * MOSFET logic:
 * HIGH = modem powered on
 * LOW  = modem powered off
 *
 * If your MOSFET works with inverted logic, swap these.
 */
#define MOSFET_ON_LEVEL HIGH
#define MOSFET_OFF_LEVEL LOW

const unsigned long TRM_BOOT_DELAY_MS = 20000UL;
const unsigned long COMMAND_PAUSE_MS = 1000UL;

// ============================================================
// TRM SERIAL HELPERS
// ============================================================

void flushTRM() {
  while (trm.available()) {
    trm.read();
  }
}

bool waitFor(
  const char* expected,
  unsigned long timeout = 10000
) {
  char buffer[240];
  int position = 0;

  buffer[0] = '\0';

  unsigned long start = millis();

  while (millis() - start < timeout) {
    while (trm.available()) {
      char c = trm.read();

      // Print modem response to Serial Monitor.
      Serial.write(c);

      if (position < (int)sizeof(buffer) - 1) {
        buffer[position++] = c;
        buffer[position] = '\0';
      } else {
        /*
         * Keep the newest part of the response if the
         * buffer becomes full.
         */
        memmove(
          buffer,
          buffer + 60,
          position - 60
        );

        position -= 60;

        buffer[position++] = c;
        buffer[position] = '\0';
      }

      if (strstr(buffer, expected) != NULL) {
        /*
         * Give the modem a short additional period to send
         * trailing characters such as CR/LF.
         */
        unsigned long quietStart = millis();

        while (millis() - quietStart < 300) {
          while (trm.available()) {
            Serial.write(trm.read());
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
    Serial.println(F("\n[RESPONSE OK]"));
  } else {
    Serial.println(F("\n[TIMEOUT / EXPECTED TEXT NOT FOUND]"));
  }

  delay(COMMAND_PAUSE_MS);

  return success;
}

// ============================================================
// MODEM POWER CONTROL
// ============================================================

void powerOnTRM() {
  Serial.println(F("Powering on TRM142..."));

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_ON_LEVEL
  );

  Serial.print(F("Waiting "));
  Serial.print(TRM_BOOT_DELAY_MS / 1000UL);
  Serial.println(F(" seconds for modem startup..."));

  delay(TRM_BOOT_DELAY_MS);

  Serial.println(F("TRM142 startup delay finished."));
}

void powerOffTRM() {
  Serial.println(F("Powering off TRM142..."));

  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  delay(500);

  Serial.println(F("TRM142 power disabled."));
}

// ============================================================
// AUTOMATIC TEST
// ============================================================

void runAutomaticTest() {
  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("=== TRM142 AUTOMATIC AT TEST ==="));
  Serial.println(F("================================"));

  sendAT(
    "AT",
    "OK",
    10000
  );

  sendAT(
    "ATI",
    "OK",
    10000
  );

  sendAT(
    "AT+IPR?",
    "+IPR:",
    10000
  );

  sendAT(
    "AT+CPIN?",
    "READY",
    15000
  );

  sendAT(
    "AT+CEREG?",
    "+CEREG:",
    15000
  );

  sendAT(
    "AT+CSQ",
    "OK",
    10000
  );

  sendAT(
    "AT+COPS?",
    "OK",
    15000
  );

  sendAT(
    "AT+QNWINFO",
    "OK",
    15000
  );

  sendAT(
    "AT+CGPADDR=1",
    "+CGPADDR:",
    15000
  );

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("Automatic test finished."));
  Serial.println(F("You can now enter AT commands manually."));
  Serial.println(F("Set Serial Monitor line ending to:"));
  Serial.println(F("Carriage Return or Both NL & CR"));
  Serial.println(F("================================"));
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

  /*
   * Make sure modem is initially off.
   */
  digitalWrite(
    TRM_POWER_PIN,
    MOSFET_OFF_LEVEL
  );

  delay(1500);

  Serial.println();
  Serial.println(F("=== TRM142 SERIAL TEST ==="));
  Serial.println(F("Arduino Nano RX: D8"));
  Serial.println(F("Arduino Nano TX: D9"));
  Serial.println(F("TRM power MOSFET: D4"));
  Serial.println(F("TRM serial baud: 19200"));

  powerOnTRM();

  runAutomaticTest();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  /*
   * Manual terminal bridge:
   *
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
