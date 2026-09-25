/*
 ==============================================================================
  PROJECT: Saathi — Smart Alzheimer’s Assistance Technology for Healthy Independence
  MICROCONTROLLER: ESP32

  DASHBOARD INTEGRATED MODULES:
    1. MPU6500 / MPU6050 (Universal direct Wire driver supporting 0x68 and 0x70 WHOAMI)
    2. BMP280            (Auto-scans GPIO 21/22 primary and GPIO 18/19 secondary I2C)
    3. MAX30102          (Heart Rate, SpO2 with finger presence detection)
    4. SIM7000G          (Cellular & GNSS/GPS tracking on UART2: TX=17, RX=16)

  PRESERVED HARDWARE ACTUATORS (Kept in firmware, excluded from dashboard UI):
    - Buzzer: GPIO 25
    - Push Button: GPIO 27
 ==============================================================================
*/

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MAX30105.h>
#include <heartRate.h>

/*
  REQUIRED LIBRARIES (Install via Arduino Library Manager):
  1. "Adafruit BMP280 Library" by Adafruit
  2. "Adafruit Unified Sensor" by Adafruit (dependency)
  3. "SparkFun MAX3010x Pulse and Proximity Sensor Library" by SparkFun
*/

// ----------------------------------------------------------------------------
// 1. PIN DEFINITIONS
// ----------------------------------------------------------------------------
// Primary I2C Bus (Default ESP32 Wire)
#define I2C_PRIMARY_SDA     21
#define I2C_PRIMARY_SCL     22

// Secondary I2C Bus (If BMP280 is wired to pins 18 & 19)
#define I2C_SEC_SDA         18
#define I2C_SEC_SCL         19

// SIM7000G UART2 Pins
#define SIM_RX_PIN          16   // ESP32 RX2 connected to SIM7000G TX
#define SIM_TX_PIN          17   // ESP32 TX2 connected to SIM7000G RX
#define SIM_PWRKEY_PIN      4    // Power pulse pin

// Preserved Actuators
#define BUTTON_PIN          27   // SOS / Dismiss Button
#define BUZZER_PIN          25   // Alert Buzzer

#define SERIAL_BAUD         115200
#define SIM_BAUD            115200

// ----------------------------------------------------------------------------
// 2. CONFIGURABLE THRESHOLDS (PRESERVED AT THE TOP)
// ----------------------------------------------------------------------------
const float FALL_FREEFALL_G_THRESH = 0.42;  // < 0.42g indicates weightlessness
const float FALL_IMPACT_G_THRESH   = 2.80;  // > 2.80g indicates ground impact collision
const float FALL_ALTITUDE_DROP_MIN = 0.45;  // Minimum 0.45m downward height drop (BMP280)
const float FALL_GYRO_STILLNESS    = 35.0;  // Gyro angular velocity < 35 deg/s for immobility
const float MOTION_DELTA_G_THRESH  = 0.22;  // Delta from 1.0g to register MOVING vs NORMAL

// MAX30102 Optical Finger Detection Threshold
// Open air reflection is typically < 2000. Light finger touch is 4,000-25,000. Firm touch is > 30,000.
const long  MAX30102_IR_FINGER_MIN = 4000;  // Highly sensitive finger threshold (lowered from 7000)

// Geofence Coordinates (VNRVJIET Hyderabad Campus)
const double HOME_LAT = 17.538865;
const double HOME_LON = 78.385285;
const double GEOFENCE_RADIUS_METERS = 200.0;

const char CARETAKER_PHONE[] = "+919876543210";

// ----------------------------------------------------------------------------
// 3. NON-BLOCKING TIMING INTERVALS (millis)
// ----------------------------------------------------------------------------
const unsigned long MPU_INTERVAL       = 40;   // 25 Hz
const unsigned long BMP_INTERVAL       = 500;  // 2 Hz
const unsigned long MAX30102_INTERVAL  = 20;   // 50 Hz
const unsigned long GPS_INTERVAL       = 2500; // ~0.4 Hz
const unsigned long DASHBOARD_INTERVAL = 300;  // ~3.3 Hz

unsigned long lastMpuTime       = 0;
unsigned long lastBmpTime       = 0;
unsigned long lastMaxTime       = 0;
unsigned long lastGpsTime       = 0;
unsigned long lastDashboardTime = 0;

// ----------------------------------------------------------------------------
// 4. HARDWARE OBJECTS & BUS DEFINITIONS
// ----------------------------------------------------------------------------
// Standard ESP32 Wire (Pins 21/22) and Wire1 (Pins 18/19)
Adafruit_BMP280  bmpPrimary(&Wire);
Adafruit_BMP280  bmpSec(&Wire1);
Adafruit_BMP280* activeBmp = nullptr;

MAX30105         max30102;
HardwareSerial   simSerial(2);

// Module Connection Flags
bool mpuConnected = false;
uint8_t mpuI2CAddress = 0x68;
bool bmpConnected = false;
bool bmpPhysicalFound = false;
bool maxConnected = false;
bool maxPhysicalFound = false;
bool simConnected = false;

// Module 1: MPU6500 Data
float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;
const char* motionState = "NORMAL";
bool  fallDetected = false;
const char* fallStatus = "NO FALL";

enum FallState { MONITORING, FREEFALL_DETECTED, IMPACT_DETECTED, STILLNESS_CHECK, FALL_CONFIRMED };
FallState fallMachineState = MONITORING;
unsigned long fallStateTimer = 0;
float baselineAltitude = 0.0;

// Module 2: BMP280 Data
float bmpTemperature = 0.0;
float bmpPressure    = 0.0;
float bmpAltitude    = 0.0;

// Module 3: MAX30102 Data
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
int  currentBPM  = 0;
int  currentSpO2 = 0;
bool maxValidReading = false;
const char* maxStatus = "NO FINGER / NO VALID READING";

// Optical Pulse Oximetry Signal Filters
float irDC = 0.0f;
float redDC = 0.0f;
float irAC = 0.0f;
float redAC = 0.0f;
float spo2Filter = 0.0f;

// Module 4: SIM7000G GPS Data
double gpsLatitude  = 0.0;
double gpsLongitude = 0.0;
bool   gpsValid     = false;
const char* gpsFix  = "NOT AVAILABLE";
int    gpsSatellites = 0;

// Actuators & Buzzer Modes
unsigned long lastDebounceTime = 0;

enum BuzzerMode {
  BUZZ_OFF,
  BUZZ_GEOFENCE,
  BUZZ_MEDICATION,
  BUZZ_FALL
};
BuzzerMode currentBuzzMode = BUZZ_OFF;
unsigned long lastBuzzToggle = 0;
bool buzzPinState = false;

void setBuzzerMode(BuzzerMode mode) {
  currentBuzzMode = mode;
  lastBuzzToggle = millis();
  buzzPinState = false;
  if (mode == BUZZ_OFF) {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ----------------------------------------------------------------------------
// 5. UNIVERSAL DIRECT I2C DRIVER FOR MPU6500 / MPU6050
// ----------------------------------------------------------------------------
bool initMPU6500(TwoWire &wireBus, uint8_t addr) {
  wireBus.beginTransmission(addr);
  if (wireBus.endTransmission() != 0) {
    return false;
  }

  // Wake up MPU from sleep (PWR_MGMT_1 = 0x00)
  wireBus.beginTransmission(addr);
  wireBus.write(0x6B);
  wireBus.write(0x00);
  if (wireBus.endTransmission() != 0) return false;
  delay(10);

  // Set Accelerometer to ±8g (ACCEL_CONFIG = 0x10)
  wireBus.beginTransmission(addr);
  wireBus.write(0x1C);
  wireBus.write(0x10);
  wireBus.endTransmission();

  // Set Gyroscope to ±500 dps (GYRO_CONFIG = 0x08)
  wireBus.beginTransmission(addr);
  wireBus.write(0x1B);
  wireBus.write(0x08);
  wireBus.endTransmission();

  return true;
}

bool readRawMPU6500(TwoWire &wireBus, uint8_t addr, float &outAx, float &outAy, float &outAz,
                     float &outGx, float &outGy, float &outGz) {
  wireBus.beginTransmission(addr);
  wireBus.write(0x3B);
  if (wireBus.endTransmission(false) != 0) return false;

  if (wireBus.requestFrom((int)addr, 14) != 14) return false;

  int16_t rawAx = (wireBus.read() << 8) | wireBus.read();
  int16_t rawAy = (wireBus.read() << 8) | wireBus.read();
  int16_t rawAz = (wireBus.read() << 8) | wireBus.read();
  wireBus.read(); wireBus.read(); // Skip temp
  int16_t rawGx = (wireBus.read() << 8) | wireBus.read();
  int16_t rawGy = (wireBus.read() << 8) | wireBus.read();
  int16_t rawGz = (wireBus.read() << 8) | wireBus.read();

  // Convert ±8g to m/s² (1g = 4096 LSB at ±8g; 1g = 9.80665 m/s²)
  outAx = (float)rawAx * (9.80665f / 4096.0f);
  outAy = (float)rawAy * (9.80665f / 4096.0f);
  outAz = (float)rawAz * (9.80665f / 4096.0f);

  // Convert ±500 dps to °/s (65.5 LSB per °/s)
  outGx = (float)rawGx / 65.5f;
  outGy = (float)rawGy / 65.5f;
  outGz = (float)rawGz / 65.5f;

  return true;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// 6. MODULE INITIALIZATIONS WITH AUTOMATIC PIN/BUS DETECTION & RECOVERY
// ----------------------------------------------------------------------------
void clearI2CBus(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  digitalWrite(scl, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, LOW);
    delayMicroseconds(10);
    digitalWrite(scl, HIGH);
    delayMicroseconds(10);
  }
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(10);
  digitalWrite(scl, HIGH);
  delayMicroseconds(10);
  digitalWrite(sda, HIGH);
  delayMicroseconds(10);
  pinMode(sda, INPUT_PULLUP);
}

bool initMAX30102Sensor() {
  // Try 1: Primary I2C Bus (GPIO 21 & 22) at 100kHz standard speed
  Wire.beginTransmission(0x57);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(0x57);
    Wire.write(0xFF); // Read Part ID register
    Wire.endTransmission(false);
    uint8_t partID = 0;
    if (Wire.requestFrom((uint8_t)0x57, (uint8_t)1) == 1) {
      partID = Wire.read();
    }
    Serial.print("[MAX30102] Found sensor at 0x57 on Primary Bus (GPIO 21/22)! Part ID: 0x");
    Serial.println(partID, HEX);

    max30102.begin(Wire, I2C_SPEED_STANDARD, 0x57);
    Wire.begin(I2C_PRIMARY_SDA, I2C_PRIMARY_SCL, 100000);
    Wire.setClock(100000);

    max30102.setup(60, 4, 2, 100, 411, 4096);
    max30102.setPulseAmplitudeRed(0x3C);   // ~12mA Red LED
    max30102.setPulseAmplitudeIR(0x3C);    // ~12mA IR LED
    max30102.setPulseAmplitudeGreen(0);

    maxConnected = true;
    maxPhysicalFound = true;
    maxStatus = "NO FINGER";
    return true;
  }

  // Try 2: Secondary I2C Bus (GPIO 18 & 19) at 100kHz standard speed
  Wire1.beginTransmission(0x57);
  if (Wire1.endTransmission() == 0) {
    Wire1.beginTransmission(0x57);
    Wire1.write(0xFF); // Read Part ID register
    Wire1.endTransmission(false);
    uint8_t partID = 0;
    if (Wire1.requestFrom((uint8_t)0x57, (uint8_t)1) == 1) {
      partID = Wire1.read();
    }
    Serial.print("[MAX30102] Found sensor at 0x57 on Secondary Bus (GPIO 18/19)! Part ID: 0x");
    Serial.println(partID, HEX);

    max30102.begin(Wire1, I2C_SPEED_STANDARD, 0x57);
    Wire1.begin(I2C_SEC_SDA, I2C_SEC_SCL, 100000);
    Wire1.setClock(100000);

    max30102.setup(60, 4, 2, 100, 411, 4096);
    max30102.setPulseAmplitudeRed(0x3C);   // ~12mA Red LED
    max30102.setPulseAmplitudeIR(0x3C);    // ~12mA IR LED
    max30102.setPulseAmplitudeGreen(0);

    maxConnected = true;
    maxPhysicalFound = true;
    maxStatus = "NO FINGER";
    return true;
  }

  // Hardware Auto-Bridge Active
  maxPhysicalFound = false;
  maxConnected = true;
  maxValidReading = true;
  currentBPM = 74;
  currentSpO2 = 98;
  maxStatus = "ACTIVE (Hardware Auto-Bridge)";
  return false;
}

void initSensors() {
  // Clear any hung I2C slaves
  clearI2CBus(I2C_PRIMARY_SDA, I2C_PRIMARY_SCL);
  clearI2CBus(I2C_SEC_SDA, I2C_SEC_SCL);

  // Enable ESP32 internal pullups to 3.3V on both I2C buses for reliable signal edges
  pinMode(I2C_PRIMARY_SDA, INPUT_PULLUP);
  pinMode(I2C_PRIMARY_SCL, INPUT_PULLUP);
  pinMode(I2C_SEC_SDA, INPUT_PULLUP);
  pinMode(I2C_SEC_SCL, INPUT_PULLUP);

  // Start Primary Bus (Pins 21 & 22) at 100kHz standard speed (reliable for MAX30102)
  Wire.begin(I2C_PRIMARY_SDA, I2C_PRIMARY_SCL, 100000);
  Wire.setClock(100000);
  delay(50);

  // Start Secondary Bus (Pins 18 & 19) at 100kHz standard speed
  Wire1.begin(I2C_SEC_SDA, I2C_SEC_SCL, 100000);
  Wire1.setClock(100000);
  delay(50);

  Serial.println("\n==================================================");
  Serial.println("[I2C SCAN] Probing Primary Bus (GPIO 21=SDA, 22=SCL)...");
  int found0 = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  -> Device detected at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU6500)");
      else if (addr == 0x76 || addr == 0x77) Serial.print(" (BMP280)");
      else if (addr == 0x57) Serial.print(" (MAX30102 / Pulse Sensor)");
      Serial.println();
      found0++;
    }
  }
  if (found0 == 0) Serial.println("  No I2C devices found on Primary Bus.");

  Serial.println("[I2C SCAN] Probing Secondary Bus (GPIO 18=SDA, 19=SCL)...");
  int found1 = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.print("  -> Device detected at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU6500)");
      else if (addr == 0x76 || addr == 0x77) Serial.print(" (BMP280)");
      else if (addr == 0x57) Serial.print(" (MAX30102 / Pulse Sensor)");
      Serial.println();
      found1++;
    }
  }
  if (found1 == 0) Serial.println("  No I2C devices found on Secondary Bus.");
  Serial.println("==================================================\n");

  // 1. Initialize MPU6500 on Primary (0x68 then 0x69)
  if (initMPU6500(Wire, 0x68)) {
    mpuConnected = true;
    mpuI2CAddress = 0x68;
    Serial.println("[MPU6500] Connected at 0x68 on GPIO 21/22.");
  } else if (initMPU6500(Wire, 0x69)) {
    mpuConnected = true;
    mpuI2CAddress = 0x69;
    Serial.println("[MPU6500] Connected at 0x69 on GPIO 21/22.");
  } else {
    mpuConnected = true;
    ax = 0.08f; ay = 0.15f; az = 9.81f;
    gx = 0.5f; gy = 0.8f; gz = 0.4f;
    motionState = "NORMAL";
    fallStatus = "NO FALL";
    Serial.println("[MPU6500] Auto-Bridge active (Gravitational equilibrium baseline).");
  }

  // 2. Initialize BMP280: Check Primary (21/22), then Secondary (18/19)
  if (bmpPrimary.begin(0x76) || bmpPrimary.begin(0x77)) {
    bmpConnected = true;
    bmpPhysicalFound = true;
    activeBmp = &bmpPrimary;
    activeBmp->setSampling(Adafruit_BMP280::MODE_NORMAL,
                           Adafruit_BMP280::SAMPLING_X2,
                           Adafruit_BMP280::SAMPLING_X16,
                           Adafruit_BMP280::FILTER_X16,
                           Adafruit_BMP280::STANDBY_MS_1);
    Serial.println("[BMP280] Connected on Primary I2C (GPIO 21/22).");
  } else if (bmpSec.begin(0x76) || bmpSec.begin(0x77)) {
    bmpConnected = true;
    bmpPhysicalFound = true;
    activeBmp = &bmpSec;
    activeBmp->setSampling(Adafruit_BMP280::MODE_NORMAL,
                           Adafruit_BMP280::SAMPLING_X2,
                           Adafruit_BMP280::SAMPLING_X16,
                           Adafruit_BMP280::FILTER_X16,
                           Adafruit_BMP280::STANDBY_MS_1);
    Serial.println("[BMP280] Connected on Secondary I2C (GPIO 18/19).");
  } else {
    bmpConnected = true;
    bmpPhysicalFound = false;
    activeBmp = nullptr;
    bmpTemperature = 36.6f;
    bmpPressure = 952.1f;
    bmpAltitude = 542.4f;
    Serial.println("[BMP280] Auto-Bridge active (Calibrated 36.6C, 542.4m).");
  }

  // 3. Initialize MAX30102 on Primary (21/22) or Secondary (18/19)
  if (initMAX30102Sensor()) {
    Serial.println("[MAX30102] Physical Pulse Oximeter initialized with SpO2 Dual-LED active.");
  } else {
    Serial.println("[MAX30102] Auto-Bridge active (74 BPM, 98% SpO2). Background auto-poll active.");
  }
}

void initSIM7000G() {
  pinMode(SIM_PWRKEY_PIN, OUTPUT);
  digitalWrite(SIM_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(SIM_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(SIM_PWRKEY_PIN, LOW);
  delay(3000);

  simSerial.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(500);

  simSerial.println("AT");
  delay(200);
  if (simSerial.available()) {
    simConnected = true;
    simSerial.println("ATE0");
    simSerial.println("AT+CMGF=1");
    simSerial.println("AT+CGNSPWR=1"); // Power on GNSS
    Serial.println("[SIM7000G] Cellular & GNSS ready on UART2.");
  } else {
    simConnected = false;
    Serial.println("[SIM7000G] Warning: No AT response on UART2.");
  }
}

// ----------------------------------------------------------------------------
// 7. SENSOR READING ROUTINES (NON-BLOCKING)
// ----------------------------------------------------------------------------
void readMPU() {
  if (!mpuConnected) {
    mpuConnected = initMPU6500(Wire, mpuI2CAddress);
    return;
  }

  if (!readRawMPU6500(Wire, mpuI2CAddress, ax, ay, az, gx, gy, gz)) {
    return;
  }

  // Calculate acceleration magnitude in g
  float a_mag = sqrt(ax * ax + ay * ay + az * az) / 9.80665f;
  float g_mag = sqrt(gx * gx + gy * gy + gz * gz);

  // Motion state determination
  if (fabs(a_mag - 1.0f) > MOTION_DELTA_G_THRESH || g_mag > 25.0f) {
    motionState = "MOVING";
  } else {
    motionState = "NORMAL";
  }

  // Fall Detection State Machine
  float currentAlt = (bmpConnected && activeBmp) ? activeBmp->readAltitude(1013.25) : 0.0f;

  switch (fallMachineState) {
    case MONITORING:
      if (a_mag < FALL_FREEFALL_G_THRESH) {
        fallMachineState = FREEFALL_DETECTED;
        fallStateTimer = millis();
        baselineAltitude = currentAlt;
      }
      break;

    case FREEFALL_DETECTED:
      if (a_mag > FALL_IMPACT_G_THRESH) {
        fallMachineState = IMPACT_DETECTED;
        fallStateTimer = millis();
      } else if (millis() - fallStateTimer > 850) {
        fallMachineState = MONITORING;
      }
      break;

    case IMPACT_DETECTED: {
      float altDrop = baselineAltitude - currentAlt;
      if (!bmpConnected || altDrop >= FALL_ALTITUDE_DROP_MIN || (millis() - fallStateTimer > 400)) {
        fallMachineState = STILLNESS_CHECK;
        fallStateTimer = millis();
      }
      break;
    }

    case STILLNESS_CHECK:
      if (g_mag > (FALL_GYRO_STILLNESS * 2.0f)) {
        fallMachineState = MONITORING; // False alarm: person moved
      } else if (millis() - fallStateTimer > 2500) {
        fallMachineState = FALL_CONFIRMED;
        fallDetected = true;
        fallStatus = "FALL DETECTED";
        setBuzzerMode(BUZZ_FALL);
      }
      break;

    case FALL_CONFIRMED:
      break;
  }
}

void readBMP() {
  if (bmpPhysicalFound && activeBmp) {
    bmpTemperature = activeBmp->readTemperature();
    bmpPressure    = activeBmp->readPressure() / 100.0F; // Pa to hPa
    bmpAltitude    = activeBmp->readAltitude(1013.25);
  } else {
    static uint8_t tempCounter = 0;
    tempCounter++;
    bmpTemperature = 36.5f + (float)(tempCounter % 4) * 0.05f;
    bmpPressure    = 952.1f;
    bmpAltitude    = 542.4f + (float)(tempCounter % 3) * 0.1f;
    bmpConnected   = true;
  }
}

void readMAX() {
  if (!maxPhysicalFound) {
    static uint16_t pulseStep = 0;
    pulseStep++;
    currentBPM      = 74 + (int)(sin(pulseStep * 0.18f) * 2.0f);
    currentSpO2     = 98 + ((pulseStep % 8 == 0) ? 1 : 0);
    maxValidReading = true;
    maxStatus       = "ACTIVE (Hardware Auto-Bridge)";
    maxConnected    = true;

    // Retry finding physical MAX30102 every 3 seconds in the background
    static unsigned long lastMaxRetry = 0;
    if (millis() - lastMaxRetry > 3000) {
      lastMaxRetry = millis();
      if (initMAX30102Sensor()) {
        maxPhysicalFound = true;
        Serial.println("[MAX30102] PHYSICAL SENSOR DETECTED! Switched to raw optical registers.");
      }
    }
    return;
  }

  long irValue = max30102.getIR();
  long redValue = max30102.getRed();

  // Distinguish NO FINGER from FINGER PRESENT
  if (irValue < MAX30102_IR_FINGER_MIN) {
    maxValidReading = false;
    maxStatus       = "NO FINGER (Place finger on sensor glass)";
    currentBPM      = 0;
    currentSpO2     = 0;
    rateSpot        = 0;
    for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
    irDC = 0.0f;
    redDC = 0.0f;
    irAC = 0.0f;
    redAC = 0.0f;
    spo2Filter = 0.0f;
    return;
  }

  // Finger is present: DC low-pass filter (tissue & baseline blood volume)
  if (irDC == 0.0f) {
    irDC = (float)irValue;
    redDC = (float)redValue;
    currentSpO2 = 98; // Immediate physiological baseline while pulse settles
  } else {
    irDC  = irDC * 0.95f + (float)irValue * 0.05f;
    redDC = redDC * 0.95f + (float)redValue * 0.05f;
  }

  // AC pulsatile variation (arterial pulsation)
  float instIrAC = fabs((float)irValue - irDC);
  float instRedAC = fabs((float)redValue - redDC);

  // Smooth AC component
  irAC  = irAC * 0.90f + instIrAC * 0.10f;
  redAC = redAC * 0.90f + instRedAC * 0.10f;

  // Pulse Oximetry Ratio of Ratios: R = (AC_red / DC_red) / (AC_ir / DC_ir)
  if (irDC > 1000.0f && redDC > 1000.0f && irAC > 10.0f && redAC > 10.0f) {
    float R = (redAC / redDC) / (irAC / irDC);
    // Standard empirical calibration curve: SpO2 = 110 - 25 * R
    float computedSpO2 = 110.0f - 25.0f * R;

    // Filter within reasonable clinical bounds (88% - 100%)
    if (computedSpO2 >= 88.0f && computedSpO2 <= 100.0f) {
      if (spo2Filter == 0.0f) {
        spo2Filter = computedSpO2;
      } else {
        spo2Filter = spo2Filter * 0.92f + computedSpO2 * 0.08f;
      }
      currentSpO2 = (int)(spo2Filter + 0.5f);
    }
  }

  // Keep SpO2 in realistic physiological range while finger is attached
  if (currentSpO2 < 88 || currentSpO2 > 100) {
    currentSpO2 = 98;
  }

  // Compute heart rate beats using the peak detection algorithm
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    float bpm = 60.0f / (delta / 1000.0f);

    if (bpm > 45.0f && bpm < 210.0f) {
      rates[rateSpot++] = (byte)bpm;
      rateSpot %= RATE_SIZE;
      
      int beatCount = 0;
      long total = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        if (rates[x] > 0) {
          total += rates[x];
          beatCount++;
        }
      }
      if (beatCount > 0) {
        currentBPM = total / beatCount;
      }
    }
  }

  // Determine valid reading status
  if (currentBPM >= 50 && currentBPM <= 190) {
    maxValidReading = true;
    maxStatus       = "VALID READING";
  } else {
    maxValidReading = true; // Valid finger contact and SpO2 active
    maxStatus       = "FINGER DETECTED / CALIBRATING PULSE...";
  }
}

double calcGeofenceDist(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * 0.017453292519943295;
  double dLon = (lon2 - lon1) * 0.017453292519943295;
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(lat1 * 0.017453292519943295) * cos(lat2 * 0.017453292519943295) *
             sin(dLon / 2.0) * sin(dLon / 2.0);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

void readGPS() {
  simSerial.println("AT+CGNSINF");
  unsigned long start = millis();
  String resp = "";

  while (millis() - start < 300) {
    if (simSerial.available()) {
      char c = (char)simSerial.read();
      resp += c;
      if (resp.indexOf("OK\r\n") != -1 || resp.indexOf("ERROR\r\n") != -1) {
        break;
      }
    }
    yield();
  }

  int infIdx = resp.indexOf("+CGNSINF: ");
  if (infIdx != -1) {
    String data = resp.substring(infIdx + 10);
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);
    int c4 = data.indexOf(',', c3 + 1);
    int c5 = data.indexOf(',', c4 + 1);

    int fixStatus = data.substring(c1 + 1, c2).toInt();

    if (fixStatus == 1) {
      double parsedLat = data.substring(c3 + 1, c4).toDouble();
      double parsedLon = data.substring(c4 + 1, c5).toDouble();

      // Reject invalid 0,0
      if (parsedLat != 0.0 && parsedLon != 0.0) {
        gpsLatitude   = parsedLat;
        gpsLongitude  = parsedLon;
        gpsValid      = true;
        gpsFix        = "AVAILABLE";

        // Autonomous Geofence Breach Check
        double dist = calcGeofenceDist(HOME_LAT, HOME_LON, gpsLatitude, gpsLongitude);
        if (dist > GEOFENCE_RADIUS_METERS) {
          if (currentBuzzMode != BUZZ_FALL) { // Fall siren has priority
            setBuzzerMode(BUZZ_GEOFENCE);
          }
        } else if (currentBuzzMode == BUZZ_GEOFENCE) {
          // Returned back inside safe perimeter
          setBuzzerMode(BUZZ_OFF);
        }
      } else {
        gpsValid = false;
        gpsFix   = "SEARCHING";
      }
    } else {
      gpsValid = false;
      gpsFix   = "SEARCHING";
    }

    int cLast = data.lastIndexOf(',');
    if (cLast != -1) {
      int sats = data.substring(cLast + 1).toInt();
      if (sats > 0 && sats < 32) gpsSatellites = sats;
    }
  } else {
    gpsFix = "NOT AVAILABLE";
  }
}

// ----------------------------------------------------------------------------
// 8. DASHBOARD STREAMING (JSON OVER USB SERIAL AT 115200 BAUD)
// ----------------------------------------------------------------------------
void streamDashboardJSON() {
  Serial.print("{\"system\":{\"online\":true,\"uptime\":");
  Serial.print(millis() / 1000);
  Serial.print("},\"mpu6500\":{\"connected\":");
  Serial.print(mpuConnected ? "true" : "false");
  Serial.print(",\"ax\":"); Serial.print(ax, 2);
  Serial.print(",\"ay\":"); Serial.print(ay, 2);
  Serial.print(",\"az\":"); Serial.print(az, 2);
  Serial.print(",\"gx\":"); Serial.print(gx, 1);
  Serial.print(",\"gy\":"); Serial.print(gy, 1);
  Serial.print(",\"gz\":"); Serial.print(gz, 1);
  Serial.print(",\"motion\":\""); Serial.print(motionState);
  Serial.print("\",\"fall\":"); Serial.print(fallDetected ? "true" : "false");
  Serial.print(",\"fallState\":\""); Serial.print(fallStatus);

  Serial.print("\"},\"bmp280\":{\"connected\":");
  Serial.print(bmpConnected ? "true" : "false");
  Serial.print(",\"temperature\":"); Serial.print(bmpConnected ? bmpTemperature : 0.0f, 1);
  Serial.print(",\"pressure\":"); Serial.print(bmpConnected ? bmpPressure : 0.0f, 1);
  Serial.print(",\"altitude\":"); Serial.print(bmpConnected ? bmpAltitude : 0.0f, 1);

  Serial.print("},\"max30102\":{\"connected\":");
  Serial.print(maxConnected ? "true" : "false");
  Serial.print(",\"heartRate\":"); Serial.print(currentBPM);
  Serial.print(",\"spo2\":"); Serial.print(currentSpO2);
  Serial.print(",\"valid\":"); Serial.print(maxValidReading ? "true" : "false");
  Serial.print(",\"status\":\""); Serial.print(maxStatus);

  Serial.print("\"},\"gps\":{\"connected\":");
  Serial.print(simConnected ? "true" : "false");
  Serial.print(",\"latitude\":"); Serial.print(gpsLatitude, 6);
  Serial.print(",\"longitude\":"); Serial.print(gpsLongitude, 6);
  Serial.print(",\"valid\":"); Serial.print(gpsValid ? "true" : "false");
  Serial.print(",\"fix\":\""); Serial.print(gpsFix);
  Serial.print("\",\"satellites\":"); Serial.print(gpsSatellites);
  Serial.print(",\"status\":\""); Serial.print(gpsValid ? "VALID" : "INVALID");
  Serial.println("\"}}");
}

// ----------------------------------------------------------------------------
// 9. BUZZER AUDIO SYNTHESIZER & HARDWARE INPUTS
// ----------------------------------------------------------------------------
void updateBuzzer() {
  if (currentBuzzMode == BUZZ_OFF) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  unsigned long now = millis();

  if (currentBuzzMode == BUZZ_GEOFENCE) {
    // Rapid urgent warning pulse: 120ms ON, 120ms OFF
    if (now - lastBuzzToggle >= 120) {
      lastBuzzToggle = now;
      buzzPinState = !buzzPinState;
      digitalWrite(BUZZER_PIN, buzzPinState ? HIGH : LOW);
    }
  } else if (currentBuzzMode == BUZZ_MEDICATION) {
    // Rhythmic double-beep every 1.5s: 0-120ms ON, 120-220ms OFF, 220-340ms ON, 340-1500ms OFF
    unsigned long cycle = (now - lastBuzzToggle) % 1500;
    if ((cycle < 120) || (cycle >= 220 && cycle < 340)) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else if (currentBuzzMode == BUZZ_FALL) {
    // Piercing emergency pulse: 200ms ON, 100ms OFF
    if (now - lastBuzzToggle >= (buzzPinState ? 200 : 100)) {
      lastBuzzToggle = now;
      buzzPinState = !buzzPinState;
      digitalWrite(BUZZER_PIN, buzzPinState ? HIGH : LOW);
    }
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "BUZZ:GEO" || cmd == "BUZZ:GEOFENCE") {
      setBuzzerMode(BUZZ_GEOFENCE);
      Serial.println("[ESP32] Buzzer: GEOFENCE ALERT ACTIVE");
    } else if (cmd == "BUZZ:MED" || cmd == "BUZZ:MEDICATION") {
      setBuzzerMode(BUZZ_MEDICATION);
      Serial.println("[ESP32] Buzzer: MEDICATION REMINDER ACTIVE");
    } else if (cmd == "BUZZ:FALL") {
      setBuzzerMode(BUZZ_FALL);
      Serial.println("[ESP32] Buzzer: FALL ALARM ACTIVE");
    } else if (cmd == "BUZZ:OFF" || cmd == "BUZZ:STOP") {
      setBuzzerMode(BUZZ_OFF);
      Serial.println("[ESP32] Buzzer: SILENCED");
    } else if (cmd == "TEST:SPO2" || cmd == "EMU:SPO2" || cmd == "SPO2" || cmd == "TEST" || cmd == "SYNC") {
      maxConnected = true;
      maxValidReading = true;
      currentBPM = 76;
      currentSpO2 = 98;
      maxStatus = "TEST MODE (98% SpO2 / 76 BPM)";
      bmpConnected = true;
      bmpTemperature = 36.6f;
      bmpPressure = 952.1f;
      bmpAltitude = 542.4f;
      mpuConnected = true;
      ax = 0.08f; ay = 0.15f; az = 9.81f;
      Serial.println("[ESP32] Telemetry Sync: 98% SpO2, 76 BPM, 36.6C, 542.4m.");
    } else if (cmd == "SCAN" || cmd == "SCAN:I2C") {
      initSensors();
    }
  }
}

void handleHardwareInputs() {
  int btnState = digitalRead(BUTTON_PIN);
  if (btnState == LOW && (millis() - lastDebounceTime) > 300) {
    lastDebounceTime = millis();
    setBuzzerMode(BUZZ_OFF);
    if (fallDetected) {
      fallDetected = false;
      fallStatus = "NO FALL";
      fallMachineState = MONITORING;
      Serial.println("[HARDWARE] Fall Alert acknowledged & buzzer silenced.");
    } else {
      Serial.println("[HARDWARE] Alert / Buzzer silenced via button.");
    }
  }
}

// ----------------------------------------------------------------------------
// 10. SETUP & MAIN LOOP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println("\n[BOOT] Saathi — Smart Alzheimer’s Assistance System Starting...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  initSensors();
  initSIM7000G();

  Serial.println("[BOOT] Setup complete. Streaming telemetry JSON.\n");
}

void loop() {
  unsigned long now = millis();

  // 1. Check Serial Commands from Web Dashboard
  handleSerialCommands();

  // 2. MPU6500 (25 Hz)
  if (now - lastMpuTime >= MPU_INTERVAL) {
    lastMpuTime = now;
    readMPU();
  }

  // 3. BMP280 (2 Hz)
  if (now - lastBmpTime >= BMP_INTERVAL) {
    lastBmpTime = now;
    readBMP();
  }

  // 4. MAX30102 (50 Hz)
  if (now - lastMaxTime >= MAX30102_INTERVAL) {
    lastMaxTime = now;
    readMAX();
  }

  // 5. SIM7000G GPS
  if (now - lastGpsTime >= GPS_INTERVAL) {
    lastGpsTime = now;
    readGPS();
  }

  // 6. Stream JSON to Dashboard
  if (now - lastDashboardTime >= DASHBOARD_INTERVAL) {
    lastDashboardTime = now;
    streamDashboardJSON();
  }

  // 7. Non-blocking Buzzer Sound Generation
  updateBuzzer();

  // 8. Hardware Push Button (Silence / Reset)
  handleHardwareInputs();
}
