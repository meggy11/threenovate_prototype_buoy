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
#define LED_WHITE_LEVEL 255U

/*
 * Status brightness levels.
 *
 * RED/YELLOW can be bright because they are short-lived.
 * GREEN/BLUE are deliberately dimmer because the modem may be
 * drawing high current while booting or transmitting.
 */
#define STATUS_LED_LEVEL 255U
#define MODEM_BOOT_LED_LEVEL 64U
#define MODEM_TX_LED_LEVEL 32U

/*
 * Serial output controls.
 * Keep recurring output minimal for timing stability.
 */
#define VERBOSE_AT 0
#define PRINT_IMU_SAMPLE_EXAMPLES 1

/*
 * Runtime safety limits.
 */
#define MIN_INTERVAL_SECONDS 10UL
#define MAX_INTERVAL_SECONDS 86400UL

#define MIN_SAMPLE_HZ 1UL
#define MAX_SAMPLE_HZ 50UL

#define MIN_SAMPLE_PERIOD_MS 20UL
#define MAX_SAMPLE_PERIOD_MS 1000UL

#define MIN_ACTUATOR_SECONDS 1UL
#define MAX_ACTUATOR_SECONDS 300UL
#define MAX_PERSISTED_SAMPLE_PERIOD_MS 3600000UL

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

#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_DATA_START     0x3B
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_PWR_MGMT_2     0x6C
#define MPU6050_WHO_AM_I       0x75

/*
 * MPU6050 operating configuration.
 *
 * DLPF_CFG = 3:
 *   gyro bandwidth ~42 Hz
 *   accelerometer bandwidth ~44 Hz
 *
 * With DLPF enabled, the gyro output rate is 1 kHz.
 * SMPLRT_DIV = 19 therefore gives 1000 / (1 + 19) = 50 Hz.
 *
 * Full-scale ranges match the calibration:
 *   accelerometer +/-2 g
 *   gyroscope     +/-250 deg/s
 */
#define MPU6050_DLPF_CFG_VALUE 3U
#define MPU6050_SMPLRT_DIV_VALUE 19U
#define MPU6050_ACCEL_FS_VALUE 0x00U
#define MPU6050_GYRO_FS_VALUE  0x00U

/*
 * MPU6050 calibration offsets measured with the complete system
 * mounted in its normal flat orientation.
 *
 * Calibration configuration:
 * - Accelerometer: +/-2 g  (16384 LSB/g)
 * - Gyroscope:     +/-250 degrees/s (131 LSB/(degree/s))
 */
const float AX_OFFSET = -645.334f;
const float AY_OFFSET =  156.654f;
const float AZ_OFFSET = -2185.446f;

const float GX_OFFSET = -410.586f;
const float GY_OFFSET =   48.954f;
const float GZ_OFFSET =   18.579f;

/*
 * Arduino Nano has limited SRAM.
 * Keep this value relatively small.
 */
#define MAX_IMU_SAMPLES 25

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t gx;
  int16_t gy;
  int16_t gz;
};

ImuSample samples[MAX_IMU_SAMPLES];

/*
 * Temperature changes very slowly compared with a 0.5 s IMU burst,
 * so keep only one temperature value for the complete batch instead
 * of storing it in every sample.
 */
int16_t batchTemperatureRaw = 0;

/*
 * Default acquisition:
 *
 * 25 samples at 50 Hz = 0.5 second measurement burst.
 *
 * 50 Hz is useful for detecting short motion events such as:
 * - impacts
 * - rope tension changes / jerks
 * - rapid roll and pitch changes
 *
 * Keeping the default buffer at 25 samples limits SRAM use on
 * the Arduino Nano while still providing a useful motion window.
 */
byte sampleCount = 25;
unsigned long samplePeriodMs = 20UL; // 50 Hz

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
const unsigned long COMMAND_LISTEN_MS = 15000UL;

/*
 * Default actuator duration used when no valid duration is supplied.
 */
const unsigned long DEFAULT_ACTUATOR_ON_MS = 10000UL;

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

/*
 * MQTT +QMTRECV is captured with a fixed-size buffer instead of
 * a dynamic String. This avoids heap allocation/fragmentation while
 * the modem response buffer is also active.
 */

unsigned long cycleNumber = 0;

/*
 * Non-blocking actuator state.
 *
 * Commands can be received and processed while the light/siren
 * remains active. No delay(durationMs) is used for actuators.
 */
bool lightActive = false;
bool sirenActive = false;

unsigned long lightOffAtMs = 0;
unsigned long sirenOffAtMs = 0;

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
void executeCommandList(const String &payload);
void executeCommandCString(char *command);
void executeCommandListCString(char *payload);
void feedModemLineChar(char c);

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


// ============================================================
// SYSTEM STATUS LED COLORS
// ============================================================

/*
 * Status colors for the complete WS2812 strip:
 *
 * RED    = Arduino/system starting
 * GREEN  = TRM142 booting
 * YELLOW = IMU sampling
 * BLUE   = MQTT data transmission
 *
 * The same color is sent to every LED using the existing
 * bufferless WS2812 driver, so no LED framebuffer is allocated.
 */
void showStatusColor(
  uint8_t red,
  uint8_t green,
  uint8_t blue
) {
  digitalWrite(LIGHT_PIN, MOSFET_ON_LEVEL);
  delay(20UL);

  setAllLedsSameColor(
    red,
    green,
    blue
  );
}

void showStatusRed() {
  showStatusColor(STATUS_LED_LEVEL, 0, 0);
}

void showStatusGreen() {
  showStatusColor(0, MODEM_BOOT_LED_LEVEL, 0);
}

void showStatusYellow() {
  showStatusColor(
    STATUS_LED_LEVEL,
    STATUS_LED_LEVEL,
    0
  );
}

void showStatusBlue() {
  showStatusColor(0, 0, MODEM_TX_LED_LEVEL);
}

void blinkEmergencyRed(byte flashes) {
  for (byte i = 0; i < flashes; i++) {
    showStatusRed();
    delay(250UL);
    turnLightOff();
    delay(250UL);
  }
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
  lightActive = false;
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
  sirenActive = false;
  digitalWrite(SIREN_PIN, MOSFET_OFF_LEVEL);
}

void turnLightOnFor(unsigned long durationMs) {
  digitalWrite(LIGHT_PIN, MOSFET_ON_LEVEL);
  delay(50UL);

  setAllLedsSameColor(
    LED_WHITE_LEVEL,
    LED_WHITE_LEVEL,
    LED_WHITE_LEVEL
  );

  lightActive = true;
  lightOffAtMs = millis() + durationMs;

  Serial.print(F("LIGHT ON "));
  Serial.print(durationMs / 1000UL);
  Serial.println(F("s"));
}

void turnSirenOnFor(unsigned long durationMs) {
  digitalWrite(SIREN_PIN, MOSFET_ON_LEVEL);

  sirenActive = true;
  sirenOffAtMs = millis() + durationMs;

  Serial.print(F("SIREN ON "));
  Serial.print(durationMs / 1000UL);
  Serial.println(F("s"));
}

// ============================================================
// NON-BLOCKING ACTUATOR TIMER UPDATE
// ============================================================

void updateActuators() {
  unsigned long now = millis();

  if (
    lightActive &&
    (long)(now - lightOffAtMs) >= 0
  ) {
    turnLightOff();
    Serial.println(F("LIGHT OFF"));
  }

  if (
    sirenActive &&
    (long)(now - sirenOffAtMs) >= 0
  ) {
    turnSirenOff();
    Serial.println(F("SIREN OFF"));
  }
}


void finishStatusLight() {
  /*
   * Status colors temporarily share the same WS2812 strip as the
   * user-controlled white light. Restore white if a timed light
   * command is still active; otherwise switch the strip off.
   */
  if (
    lightActive &&
    (long)(millis() - lightOffAtMs) < 0
  ) {
    digitalWrite(LIGHT_PIN, MOSFET_ON_LEVEL);
    delay(5UL);

    setAllLedsSameColor(
      LED_WHITE_LEVEL,
      LED_WHITE_LEVEL,
      LED_WHITE_LEVEL
    );
  } else {
    turnLightOff();
  }
}

// ============================================================
// TRM POWER CONTROL
// ============================================================

void powerOnTRM() {

  /*
   * GREEN status = modem booting.
   */
  showStatusGreen();

  digitalWrite(TRM_POWER_PIN, MOSFET_ON_LEVEL);

  Serial.println(F("TRM boot..."));

  delay(TRM_BOOT_DELAY_MS);

  /*
   * Turn the status strip off before starting serial modem traffic.
   */
  turnLightOff();

  Serial.println(F("TRM ready."));
}

void powerOffTRM() {
  /*
   * RED status = orderly shutdown / power-off phase.
   */
  showStatusRed();

  digitalWrite(TRM_POWER_PIN, MOSFET_OFF_LEVEL);

  delay(500UL);

  turnLightOff();
}

void setSafeOffState() {
  turnLightOff();
  turnSirenOff();
  powerOffTRM();
}

// ============================================================
// EEPROM PERSISTENT CONFIGURATION
// ============================================================

const uint32_t EEPROM_CONFIG_MAGIC = 0x42555932UL; // "BUY2"
const byte EEPROM_CONFIG_VERSION = 1;

struct PersistentConfig {
  uint32_t magic;
  byte version;

  unsigned long offTimeMs;
  byte sampleCount;
  unsigned long samplePeriodMs;

  long lastCommandId;
};

PersistentConfig persistentConfig;

void savePersistentConfig() {
  persistentConfig.magic = EEPROM_CONFIG_MAGIC;
  persistentConfig.version = EEPROM_CONFIG_VERSION;

  persistentConfig.offTimeMs = offTimeMs;
  persistentConfig.sampleCount = sampleCount;
  persistentConfig.samplePeriodMs = samplePeriodMs;
  persistentConfig.lastCommandId = lastCommandId;

  /*
   * EEPROM.put() updates only bytes that actually changed,
   * reducing unnecessary EEPROM wear.
   */
  EEPROM.put(0, persistentConfig);
}

void loadPersistentConfig() {
  EEPROM.get(0, persistentConfig);

  bool valid =
    persistentConfig.magic == EEPROM_CONFIG_MAGIC &&
    persistentConfig.version == EEPROM_CONFIG_VERSION &&
    persistentConfig.offTimeMs >=
      (MIN_INTERVAL_SECONDS * 1000UL) &&
    persistentConfig.offTimeMs <=
      (MAX_INTERVAL_SECONDS * 1000UL) &&
    persistentConfig.sampleCount >= 1 &&
    persistentConfig.sampleCount <= MAX_IMU_SAMPLES &&
    persistentConfig.samplePeriodMs >= MIN_SAMPLE_PERIOD_MS &&
    persistentConfig.samplePeriodMs <= MAX_PERSISTED_SAMPLE_PERIOD_MS;

  if (!valid) {
    /*
     * EEPROM is blank, from an older firmware, or contains an
     * invalid configuration. Keep compiled defaults and save them.
     */
    lastCommandId = -1L;
    savePersistentConfig();

    Serial.println(F("EEPROM: initialized defaults."));
    return;
  }

  offTimeMs = persistentConfig.offTimeMs;
  sampleCount = persistentConfig.sampleCount;
  samplePeriodMs = persistentConfig.samplePeriodMs;
  lastCommandId = persistentConfig.lastCommandId;

  Serial.print(F("EEPROM: interval="));
  Serial.print(offTimeMs / 1000UL);
  Serial.print(F("s, samples="));
  Serial.print(sampleCount);
  Serial.print(F(", period="));
  Serial.print(samplePeriodMs);
  Serial.println(F("ms"));
}

void saveLastCommandId(long id) {
  if (id == lastCommandId) {
    return;
  }

  lastCommandId = id;
  savePersistentConfig();
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

bool readImuRegister(
  byte registerAddress,
  byte &value
) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(registerAddress);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(
        (int)MPU6050_ADDR,
        1,
        (int)true
      ) != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool wakeImu() {
  if (!imuExists()) {
    Serial.println(F("ERROR: MPU6050 not detected."));
    blinkEmergencyRed(3);
    return false;
  }

  byte whoAmI = 0;

  if (!readImuRegister(
        MPU6050_WHO_AM_I,
        whoAmI
      )) {
    Serial.println(F("ERROR: MPU6050 WHO_AM_I read failed."));
    blinkEmergencyRed(3);
    return false;
  }

  /*
   * WHO_AM_I can reflect the AD0-selected address identity.
   * We already verified I2C communication at MPU6050_ADDR, so
   * report the value without rejecting known board variants.
   */
  Serial.print(F("MPU6050 WHO_AM_I=0x"));
  Serial.println(whoAmI, HEX);

  /*
   * Wake sensor and use the X-axis gyroscope PLL clock source.
   * CLKSEL=1 is preferred over the internal oscillator for stable
   * sampling when the gyro is active.
   */
  if (!writeImuRegister(
        MPU6050_PWR_MGMT_1,
        0x01
      )) {
    Serial.println(F("ERROR: MPU6050 wake failed."));
    blinkEmergencyRed(3);
    return false;
  }

  /*
   * Enable all accelerometer and gyroscope axes.
   */
  if (!writeImuRegister(
        MPU6050_PWR_MGMT_2,
        0x00
      )) {
    Serial.println(F("ERROR: MPU6050 axis enable failed."));
    blinkEmergencyRed(3);
    return false;
  }

  /*
   * Configure digital low-pass filter.
   */
  if (!writeImuRegister(
        MPU6050_CONFIG,
        MPU6050_DLPF_CFG_VALUE
      )) {
    Serial.println(F("ERROR: MPU6050 DLPF config failed."));
    blinkEmergencyRed(3);
    return false;
  }

  /*
   * Configure the sensor's own sample rate to 50 Hz.
   */
  if (!writeImuRegister(
        MPU6050_SMPLRT_DIV,
        MPU6050_SMPLRT_DIV_VALUE
      )) {
    Serial.println(F("ERROR: MPU6050 sample rate config failed."));
    blinkEmergencyRed(3);
    return false;
  }

  /*
   * Measurement ranges used during calibration.
   */
  if (!writeImuRegister(
        MPU6050_ACCEL_CONFIG,
        MPU6050_ACCEL_FS_VALUE
      )) {
    Serial.println(F("ERROR: MPU6050 accel range failed."));
    blinkEmergencyRed(3);
    return false;
  }

  if (!writeImuRegister(
        MPU6050_GYRO_CONFIG,
        MPU6050_GYRO_FS_VALUE
      )) {
    Serial.println(F("ERROR: MPU6050 gyro range failed."));
    blinkEmergencyRed(3);
    return false;
  }

  delay(100UL);

  Serial.println(
    F("MPU6050 ready: 50Hz, DLPF=3, +/-2g, +/-250dps.")
  );

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
  ImuSample &sample
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

  /*
   * Read raw MPU6050 values first, then apply the measured
   * calibration offsets before storing the sample.
   */
  int16_t rawAx =
    (int16_t)((Wire.read() << 8) | Wire.read());

  int16_t rawAy =
    (int16_t)((Wire.read() << 8) | Wire.read());

  int16_t rawAz =
    (int16_t)((Wire.read() << 8) | Wire.read());

  batchTemperatureRaw =
    (int16_t)((Wire.read() << 8) | Wire.read());

  int16_t rawGx =
    (int16_t)((Wire.read() << 8) | Wire.read());

  int16_t rawGy =
    (int16_t)((Wire.read() << 8) | Wire.read());

  int16_t rawGz =
    (int16_t)((Wire.read() << 8) | Wire.read());

  /*
   * Store calibrated readings as int16_t to keep Nano SRAM usage low.
   *
   * For a flat, stationary system the expected values are roughly:
   *   AX ~= 0
   *   AY ~= 0
   *   AZ ~= +16384
   *   GX ~= 0
   *   GY ~= 0
   *   GZ ~= 0
   *
   * Small fluctuations are normal sensor noise.
   */
  sample.ax = (int16_t)(rawAx - AX_OFFSET);
  sample.ay = (int16_t)(rawAy - AY_OFFSET);
  sample.az = (int16_t)(rawAz - AZ_OFFSET);

  sample.gx = (int16_t)(rawGx - GX_OFFSET);
  sample.gy = (int16_t)(rawGy - GY_OFFSET);
  sample.gz = (int16_t)(rawGz - GZ_OFFSET);

  return true;
}

// ============================================================
// IMU SAMPLE COLLECTION
// ============================================================

byte collectImuSamples() {
  /*
   * YELLOW status = IMU acquisition in progress.
   */
  showStatusYellow();

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

    if (readImuSample(candidate)) {
      samples[successfulSamples] = candidate;
      successfulSamples++;
    } else {
      Serial.print(
        F("IMU read failed at requested sample ")
      );

      Serial.println(i);
    }
  }

  /*
   * Sampling is finished. Restore a user-requested white light
   * if its timer is still active.
   */
  finishStatusLight();

  Serial.print(F("IMU: "));
  Serial.print(successfulSamples);
  Serial.print('/');
  Serial.print(sampleCount);
  Serial.print(F(" samples @ "));
  Serial.print(
    samplePeriodMs > 0UL
      ? (1000UL / samplePeriodMs)
      : 0UL
  );
  Serial.println(F("Hz"));

#if PRINT_IMU_SAMPLE_EXAMPLES
  if (successfulSamples > 0) {
    Serial.print(F("IMU first A/G: "));
    Serial.print(samples[0].ax);
    Serial.print(',');
    Serial.print(samples[0].ay);
    Serial.print(',');
    Serial.print(samples[0].az);
    Serial.print(F(" / "));
    Serial.print(samples[0].gx);
    Serial.print(',');
    Serial.print(samples[0].gy);
    Serial.print(',');
    Serial.println(samples[0].gz);

    if (successfulSamples > 1) {
      byte last = successfulSamples - 1;

      Serial.print(F("IMU last A/G: "));
      Serial.print(samples[last].ax);
      Serial.print(',');
      Serial.print(samples[last].ay);
      Serial.print(',');
      Serial.print(samples[last].az);
      Serial.print(F(" / "));
      Serial.print(samples[last].gx);
      Serial.print(',');
      Serial.print(samples[last].gy);
      Serial.print(',');
      Serial.println(samples[last].gz);
    }
  }
#endif

  return successfulSamples;
}

// ============================================================
// MODEM COMPLETE-LINE MQTT PARSER — FIXED CHAR BUFFER
// ============================================================

#define MODEM_LINE_BUFFER_SIZE 160U

void feedModemLineChar(char c) {
  static char modemLine[MODEM_LINE_BUFFER_SIZE];
  static byte modemLineLength = 0;

  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    modemLine[modemLineLength] = '\0';

    if (
      modemLineLength > 0U &&
      strstr(modemLine, "+QMTRECV:") != NULL
    ) {
      /*
       * Expected:
       * +QMTRECV: 0,0,"threenovate/cmd","light on 30 id=1025"
       *
       * Extract the LAST quoted field (payload) in-place.
       */
      char *lastQuote = strrchr(modemLine, '"');

      if (lastQuote != NULL) {
        char *openQuote = lastQuote - 1;

        while (
          openQuote >= modemLine &&
          *openQuote != '"'
        ) {
          openQuote--;
        }

        if (
          openQuote >= modemLine &&
          *openQuote == '"'
        ) {
          *lastQuote = '\0';

          char *payload = openQuote + 1;

          Serial.print(F("MQTT RX: "));
          Serial.println(payload);

          executeCommandListCString(payload);
        } else {
          Serial.println(
            F("MQTT parse error: opening quote.")
          );
        }
      } else {
        Serial.println(
          F("MQTT parse error: closing quote.")
        );
      }
    }

    modemLineLength = 0;
    modemLine[0] = '\0';
    return;
  }

  if (
    modemLineLength <
    MODEM_LINE_BUFFER_SIZE - 1U
  ) {
    modemLine[modemLineLength++] = c;
  } else {
    /*
     * Oversized line: discard this line safely.
     */
    modemLineLength = 0;
    modemLine[0] = '\0';

    Serial.println(
      F("MODEM line too long; discarded.")
    );
  }
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

#if VERBOSE_AT
      Serial.write(c);
#endif

      feedModemLineChar(c);

      /*
       * Keep a bounded copy of the modem response.
       *
       * 300 bytes is enough for the subscription response plus a
       * typical retained +QMTRECV command while still staying
       * relatively small for the Nano.
       */
      if (rxBuf.length() < 300U) {
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

#if VERBOSE_AT
              Serial.write(d);
#endif

              feedModemLineChar(d);

              if (rxBuf.length() < 300U) {
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

#if VERBOSE_AT
  Serial.println();
  Serial.print(F(">> "));
  Serial.println(command);
#endif

  trm.print(command);
  trm.print('\r');

  bool success = waitFor(
    expected,
    timeoutMs
  );

  if (!success) {
    Serial.print(F("AT FAIL: "));
    Serial.println(command);
  }

  delay(COMMAND_PAUSE_MS);

  return success;
}

bool modemResponds() {

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
   * Retained +QMTRECV commands are parsed directly from complete
   * serial lines by feedModemLineChar() while waitFor() is active.
   * Do not parse them again from the bounded rxBuf.
   */
  Serial.println(F("MQTT connected."));

  return true;
}

// ============================================================
// MQTT DISCONNECT
// ============================================================

void mqttDisconnect() {

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

}

// ============================================================
// MQTT COMMAND PARSING — FIXED BUFFER / LOW SRAM
// ============================================================

long extractCommandIdCString(const char *command) {
  const char *idPosition = strstr(command, "id=");

  if (idPosition == NULL) {
    return -1L;
  }

  idPosition += 3;

  if (!isDigit(*idPosition)) {
    return -1L;
  }

  long value = 0;

  while (isDigit(*idPosition)) {
    value = (value * 10L) + (*idPosition - '0');
    idPosition++;
  }

  return value;
}


unsigned long extractUnsignedValueCString(
  const char *command,
  const char *key
) {
  const char *position = strstr(command, key);

  if (position == NULL) {
    return 0UL;
  }

  position += strlen(key);

  while (*position == ' ') {
    position++;
  }

  if (!isDigit(*position)) {
    return 0UL;
  }

  unsigned long value = 0UL;

  while (isDigit(*position)) {
    value =
      (value * 10UL)
      + (unsigned long)(*position - '0');

    position++;
  }

  return value;
}


// ============================================================
// MQTT COMMAND EXECUTION
// ============================================================

void executeCommandCString(char *command) {
  /*
   * Trim leading spaces.
   */
  while (*command == ' ') {
    command++;
  }

  /*
   * Lowercase in place without allocating another String.
   */
  for (char *p = command; *p != '\0'; p++) {
    *p = tolower(*p);
  }

  long commandId =
    extractCommandIdCString(command);

  Serial.print(F("CMD parse: "));
  Serial.println(command);

  if (
    commandId >= 0 &&
    commandId <= lastCommandId
  ) {
    Serial.print(F("CMD skip id="));
    Serial.println(commandId);
    return;
  }

  bool successful = false;

  // ----------------------------------------------------------
  // LIGHT
  // ----------------------------------------------------------

  if (strstr(command, "light on") != NULL) {
    unsigned long seconds =
      extractUnsignedValueCString(
        command,
        "light on"
      );

    if (seconds == 0UL) {
      seconds =
        DEFAULT_ACTUATOR_ON_MS / 1000UL;
    }

    Serial.print(F("LIGHT requested: "));
    Serial.print(seconds);
    Serial.println(F("s"));

    if (
      seconds >= MIN_ACTUATOR_SECONDS &&
      seconds <= MAX_ACTUATOR_SECONDS
    ) {
      turnLightOnFor(seconds * 1000UL);
      successful = true;
    }
  }

  // ----------------------------------------------------------
  // SIREN
  // ----------------------------------------------------------

  else if (strstr(command, "siren on") != NULL) {
    unsigned long seconds =
      extractUnsignedValueCString(
        command,
        "siren on"
      );

    if (seconds == 0UL) {
      seconds =
        DEFAULT_ACTUATOR_ON_MS / 1000UL;
    }

    Serial.print(F("SIREN requested: "));
    Serial.print(seconds);
    Serial.println(F("s"));

    if (
      seconds >= MIN_ACTUATOR_SECONDS &&
      seconds <= MAX_ACTUATOR_SECONDS
    ) {
      turnSirenOnFor(seconds * 1000UL);
      successful = true;
    }
  }

  // ----------------------------------------------------------
  // CYCLE INTERVAL
  // ----------------------------------------------------------

  else if (strstr(command, "interval ") != NULL) {
    unsigned long seconds =
      extractUnsignedValueCString(
        command,
        "interval"
      );

    if (
      seconds >= MIN_INTERVAL_SECONDS &&
      seconds <= MAX_INTERVAL_SECONDS
    ) {
      offTimeMs = seconds * 1000UL;
      savePersistentConfig();
      successful = true;
    }
  }

  // ----------------------------------------------------------
  // SAMPLE COUNT
  // ----------------------------------------------------------

  else if (
    strstr(command, "sample_count ") != NULL
  ) {
    unsigned long value =
      extractUnsignedValueCString(
        command,
        "sample_count"
      );

    if (
      value >= 1UL &&
      value <= MAX_IMU_SAMPLES
    ) {
      sampleCount = (byte)value;
      savePersistentConfig();
      successful = true;
    }
  }

  // ----------------------------------------------------------
  // SAMPLE FREQUENCY
  // ----------------------------------------------------------

  else if (
    strstr(command, "sample_hz ") != NULL
  ) {
    unsigned long hz =
      extractUnsignedValueCString(
        command,
        "sample_hz"
      );

    if (
      hz >= MIN_SAMPLE_HZ &&
      hz <= MAX_SAMPLE_HZ
    ) {
      samplePeriodMs = 1000UL / hz;

      if (
        samplePeriodMs <
        MIN_SAMPLE_PERIOD_MS
      ) {
        samplePeriodMs =
          MIN_SAMPLE_PERIOD_MS;
      }

      savePersistentConfig();
      successful = true;
    }
  }

  // ----------------------------------------------------------
  // SAMPLE PERIOD MS
  // ----------------------------------------------------------

  else if (
    strstr(command, "sample_period_ms ") != NULL
  ) {
    unsigned long value =
      extractUnsignedValueCString(
        command,
        "sample_period_ms"
      );

    if (
      value >= MIN_SAMPLE_PERIOD_MS &&
      value <= MAX_SAMPLE_PERIOD_MS
    ) {
      samplePeriodMs = value;
      savePersistentConfig();
      successful = true;
    }
  }

  // ----------------------------------------------------------
  // LEGACY SAMPLE PERIOD SECONDS
  // ----------------------------------------------------------

  else if (
    strstr(command, "sample_period ") != NULL
  ) {
    unsigned long seconds =
      extractUnsignedValueCString(
        command,
        "sample_period"
      );

    if (
      seconds >= 1UL &&
      seconds <= 3600UL
    ) {
      samplePeriodMs =
        seconds * 1000UL;

      savePersistentConfig();
      successful = true;
    }
  }

  if (successful) {
    if (commandId >= 0) {
      saveLastCommandId(commandId);
    }

    Serial.print(F("CMD OK"));

    if (commandId >= 0) {
      Serial.print(F(" id="));
      Serial.print(commandId);
    }

    Serial.println();
  } else {
    Serial.println(F("CMD invalid"));
  }
}


// ============================================================
// MULTI-COMMAND MQTT PAYLOAD EXECUTION
// ============================================================

#define MQTT_COMMAND_BUFFER_SIZE 96U

void executeCommandListCString(char *payload) {
  /*
   * Split the payload IN PLACE on ';' or newline.
   * No heap allocation and no temporary String objects.
   */
  char *start = payload;

  while (*start != '\0') {
    while (*start == ' ' || *start == '\r' || *start == '\n') {
      start++;
    }

    if (*start == '\0') {
      break;
    }

    char *end = start;

    while (
      *end != '\0' &&
      *end != ';' &&
      *end != '\n'
    ) {
      end++;
    }

    char saved = *end;
    *end = '\0';

    /*
     * Trim trailing spaces.
     */
    char *tail = end - 1;

    while (
      tail >= start &&
      (*tail == ' ' || *tail == '\r')
    ) {
      *tail = '\0';
      tail--;
    }

    if (*start != '\0') {
      executeCommandCString(start);
    }

    if (saved == '\0') {
      break;
    }

    start = end + 1;
  }
}


void executeCommandList(const String &payload) {
  /*
   * Reuse one fixed command buffer.
   *
   * No substring() calls and no extra command Strings are created.
   * This avoids silent heap-allocation failures on the Nano.
   */
  static char commandBuffer[
    MQTT_COMMAND_BUFFER_SIZE
  ];

  unsigned int commandLength = 0;

  for (
    unsigned int i = 0;
    i <= payload.length();
    i++
  ) {
    char c =
      (i < payload.length())
        ? payload.charAt(i)
        : '\0';

    bool separator =
      c == ';' ||
      c == '\n' ||
      c == '\0';

    if (separator) {
      if (commandLength > 0U) {
        /*
         * Trim trailing spaces.
         */
        while (
          commandLength > 0U &&
          commandBuffer[
            commandLength - 1U
          ] == ' '
        ) {
          commandLength--;
        }

        commandBuffer[
          commandLength
        ] = '\0';

        if (commandLength > 0U) {
          executeCommandCString(
            commandBuffer
          );
        }
      }

      commandLength = 0U;
      continue;
    }

    if (
      commandLength <
      MQTT_COMMAND_BUFFER_SIZE - 1U
    ) {
      commandBuffer[
        commandLength++
      ] = c;
    } else {
      Serial.println(
        F("CMD too long; discarded.")
      );

      /*
       * Discard until next separator.
       */
      commandLength = 0U;

      while (
        i < payload.length() &&
        payload.charAt(i) != ';' &&
        payload.charAt(i) != '\n'
      ) {
        i++;
      }
    }
  }
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

  Serial.print(F("MQTT RX: "));
  Serial.println(payload);

  executeCommandList(payload);
}

// ============================================================
// MQTT MESSAGES CAPTURED DURING AT COMMANDS
// ============================================================

void processQmtRecvFromBuffer(const String &buffer) {
  /*
   * Legacy compatibility function.
   * MQTT receive URCs are now parsed directly from complete serial
   * lines by feedModemLineChar().
   */
  (void)buffer;
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
  line.reserve(160U);

  unsigned long start = millis();

  while (millis() - start < listenMs) {
    while (trm.available()) {
      char c = trm.read();

#if VERBOSE_AT
      Serial.write(c);
#endif

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
// CONTINUOUS MQTT COMMAND POLLING
// ============================================================

void pollMqttCommands() {
  while (trm.available()) {
    char c = trm.read();

#if VERBOSE_AT
    Serial.write(c);
#endif

    feedModemLineChar(c);
  }
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
    output.print((unsigned long)i * samplePeriodMs);

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
      batchTemperatureRaw
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


  /*
   * BLUE status = MQTT payload transmission.
   */
  showStatusBlue();

  char command[100];

  snprintf(
    command,
    sizeof(command),
    "AT+QMTPUB=0,0,0,0,\"%s\"",
    TOPIC_PUB
  );

  flushTRM();

#if VERBOSE_AT
  Serial.println();
  Serial.print(F(">> "));
  Serial.println(command);
#endif

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

    finishStatusLight();

    return false;
  }


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

    finishStatusLight();

    return false;
  }

  Serial.println(F("MQTT sent."));

  finishStatusLight();

  return true;
}

// ============================================================
// ONE COMPLETE OPERATING PROCEDURE
// ============================================================

bool runMeasurementCycle() {
  cycleNumber++;

  Serial.print(F("CYCLE "));
  Serial.println(cycleNumber);

  /*
   * 1. Power on modem.
   */
  powerOnTRM();

  /*
   * 2. Verify modem and mobile network.
   */
  if (!modemResponds()) {
    Serial.println(F("ERROR: TRM142 did not respond."));
    blinkEmergencyRed(3);
    powerOffTRM();
    return false;
  }

  if (!waitForNetwork()) {
    Serial.println(F("ERROR: network unavailable."));
    blinkEmergencyRed(3);
    powerOffTRM();
    return false;
  }

  /*
   * 3. Connect MQTT and subscribe.
   *
   * Retained commands can already arrive during mqttConnect().
   * mqttConnect() processes any +QMTRECV captured in rxBuf.
   */
  if (!mqttConnect()) {
    Serial.println(F("ERROR: MQTT connect failed."));
    blinkEmergencyRed(3);
    powerOffTRM();
    return false;
  }

  /*
   * 4. Extra command window.
   *
   * Listen for another 15 seconds after MQTT is connected.
   * Any command received is processed immediately.
   *
   * Light and siren commands are non-blocking, so MQTT command
   * processing continues during this window even while an
   * actuator timer is active.
   */
  unsigned long listenStart = millis();

  while (millis() - listenStart < COMMAND_LISTEN_MS) {
    pollMqttCommands();
    updateActuators();
    delay(2UL);
  }

  /*
   * 5. Sample IMU.
   */
  bool imuReady = wakeImu();
  byte actualSamples = 0;

  if (imuReady) {
    actualSamples = collectImuSamples();
  }

  /*
   * 6. Publish measurements.
   */
  bool published =
    mqttPublishImuSamples(actualSamples);

  /*
   * 7. Sleep IMU.
   */
  if (imuReady) {
    sleepImu();
  }

  /*
   * 8. Disconnect MQTT and power off modem.
   */
  mqttDisconnect();
  powerOffTRM();

  if (!published) {
    Serial.println(F("ERROR: publish failed."));
    blinkEmergencyRed(3);
  }

  return published;
}

void waitUntilNextCycle() {
  unsigned long start = millis();

  /*
   * TRM is OFF during the low-power wait period.
   *
   * Actuator timers still continue locally. This allows a light
   * or siren command received during the command window to remain
   * active for its requested duration even after the modem is
   * powered off.
   */
  while (millis() - start < offTimeMs) {
    updateActuators();
    delay(10UL);
  }
}
// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);

  trm.begin(TRM_BAUD);

  Wire.begin();

  /*
   * Reserve String capacity up front to reduce repeated heap
   * reallocations. This does not eliminate fragmentation entirely,
   * but it reduces it substantially for the long-lived rxBuf.
   */
  rxBuf.reserve(300U);

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

  Serial.println(F("SmartBuoy ready."));
  Serial.print(F("MQTT "));
  Serial.print(BROKER);
  Serial.print(':');
  Serial.println(BROKER_PORT);

  loadPersistentConfig();

  if (imuExists()) {
    sleepImu();
  } else {
    Serial.println(F("ERROR: MPU6050 missing."));
  }

  turnLightOff();

  Serial.println(F("Ready."));
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  /*
   * Complete low-power cycle:
   *
   * TRM ON
   * -> network/MQTT connect
   * -> process retained commands
   * -> listen additional 15 seconds
   * -> sample IMU
   * -> publish
   * -> MQTT disconnect
   * -> TRM OFF
   */
  runMeasurementCycle();

  /*
   * Wait configured interval with TRM powered off.
   * Local actuator timers continue to be serviced.
   */
  waitUntilNextCycle();
}
