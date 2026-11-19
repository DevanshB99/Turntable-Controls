#include <Wire.h>
#define MT6701_ADDR      0x06
#define MT6701_ANGLE_H   0x03
#define MT6701_ANGLE_L   0x04
#define MT6701_COUNTS    16384
#define SDA_PIN          4
#define SCL_PIN          5

const float DEG_PER_COUNT = 360.0f / MT6701_COUNTS;  // 0.02197°/count

unsigned long printInterval = 100;
unsigned long lastPrint     = 0;

uint16_t rawMin      = 0xFFFF;
uint16_t rawMax      = 0;
long     rawSum      = 0;
double     rawSumSq    = 0;
uint32_t sampleCount = 0;
uint16_t prevRaw     = 0xFFFF;

void setup() {
  Serial.begin(115200);
  delay(100);
  // This prevents MT6701 from pulling SDA/SCL low during boot
  pinMode(ENCODER_PWR_PIN, OUTPUT);
  digitalWrite(ENCODER_PWR_PIN, HIGH);
  delay(50);   // MT6701 startup time

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║       MT6701 14-bit Encoder Test         ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.printf("  LSB resolution : %.5f °/count\n", DEG_PER_COUNT);
  Serial.printf("  I2C address    : 0x%02X\n", MT6701_ADDR);
  Serial.printf("  SDA / SCL      : GPIO %d / GPIO %d\n", SDA_PIN, SCL_PIN);
  Serial.printf("  VDD pin        : GPIO %d\n", ENCODER_PWR_PIN);
  Serial.println("  Commands: RAW | STATS | RESET | SLOW | FAST");
  Serial.println("─────────────────────────────────────────────");

  if (!checkMT6701()) {
    Serial.println("ERROR: MT6701 not found on I2C bus!");
    while (true) { delay(1000); }
  }

  Serial.println("MT6701 READY — rotate shaft to test.");
  Serial.println("Format: ANGLE:<deg>  RAW:<count>  DELTA:<counts>");
  Serial.println("─────────────────────────────────────────────");
}

void loop() {
  // Handle serial commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "RAW") {
      uint16_t r = readRaw();
      Serial.printf("RAW:%u\n", r);
    }
    else if (cmd == "STATS") {
      printStats();
    }
    else if (cmd == "RESET") {
      resetStats();
      Serial.println("STATS RESET");
    }
    else if (cmd == "SLOW") {
      printInterval = 500;
      Serial.println("Print interval: 500ms (SLOW)");
    }
    else if (cmd == "FAST") {
      printInterval = 50;
      Serial.println("Print interval: 50ms (FAST)");
    }
  }

  // Continuous streaming
  unsigned long now = millis();
  if (now - lastPrint >= printInterval) {
    lastPrint = now;

    uint16_t raw = readRaw();

    if (raw == 0xFFFF) {
      Serial.println("ERROR: Read failed — check wiring");
      return;
    }

    float angle = raw * DEG_PER_COUNT;

    // Signed delta with wraparound correction
    int16_t delta = 0;
    if (prevRaw != 0xFFFF) {
      delta = (int16_t)raw - (int16_t)prevRaw;
      if (delta >  (MT6701_COUNTS / 2)) delta -= MT6701_COUNTS;
      if (delta < -(MT6701_COUNTS / 2)) delta += MT6701_COUNTS;
    }
    prevRaw = raw;

    updateStats(raw);

    Serial.printf("ANGLE:%.4f  RAW:%5u  DELTA:%+d\n", angle, raw, delta);
  }
}

uint16_t readRaw() {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(MT6701_ANGLE_H);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;

  Wire.requestFrom((uint8_t)MT6701_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;

  uint8_t highByte = Wire.read();   // Angle[13:6]
  uint8_t lowByte  = Wire.read();   // Angle[5:0] in bits [7:2]

  uint16_t raw = ((uint16_t)highByte << 6) | (lowByte >> 2);
  return raw;
}

bool checkMT6701() {
  Wire.beginTransmission(MT6701_ADDR);
  return (Wire.endTransmission() == 0);
}

void updateStats(uint16_t raw) {
  if (raw < rawMin) rawMin = raw;
  if (raw > rawMax) rawMax = raw;
  rawSum   += raw;
  rawSumSq += (double)raw * raw;
  sampleCount++;
}

void resetStats() {
  rawMin      = 0xFFFF;
  rawMax      = 0;
  rawSum      = 0;
  rawSumSq    = 0;
  sampleCount = 0;
}

void printStats() {
  if (sampleCount < 2) {
    Serial.println("Not enough samples — wait a few seconds then try STATS again.");
    return;
  }

  float mean   = (float)rawSum / sampleCount;
  float meanSq = (float)rawSumSq / sampleCount;
  float stddev = sqrt(meanSq - mean * mean);

  uint16_t span    = rawMax - rawMin;
  float    spanDeg = span   * DEG_PER_COUNT;
  float    stdDeg  = stddev * DEG_PER_COUNT;

  Serial.println("─────── PRECISION STATS ───────");
  Serial.printf("  Samples    : %lu\n",    sampleCount);
  Serial.printf("  Mean (raw) : %.2f counts → %.4f°\n", mean, mean * DEG_PER_COUNT);
  Serial.printf("  Std Dev    : %.3f counts → %.5f°\n", stddev, stdDeg);
  Serial.printf("  Min raw    : %u  (%.4f°)\n", rawMin, rawMin * DEG_PER_COUNT);
  Serial.printf("  Max raw    : %u  (%.4f°)\n", rawMax, rawMax * DEG_PER_COUNT);
  Serial.printf("  Peak-peak  : %u counts → %.5f°\n", span, spanDeg);
  Serial.printf("  LSB size   : %.5f° (1 count)\n", DEG_PER_COUNT);
  Serial.println("───────────────────────────────");
  Serial.println("  TIP: 1-2 count peak-peak at rest = excellent");
  Serial.println("       >5 count peak-peak = check magnet gap/centering");
}