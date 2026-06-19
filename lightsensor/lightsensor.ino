// LightSensor firmware for the ESP32-C3 SuperMini.
// Native USB CDC is used for Serial (build with CDCOnBoot=cdc).

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LittleFS.h>

// Calibration blob (spectral responsivity CSV + metadata header) lives in a
// LittleFS file. The host writes/reads it whole; integrity is checked with a
// CRC32 (standard reflected poly, matches Python's binascii.crc32).
const char* CAL_PATH = "/cal.csv";

// Running CRC32 (no final XOR until done). Feed bytes incrementally.
uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
  }
  return crc;
}

// ESP32-C3 SuperMini I2C pins (v2 PCB).
const int I2C_SDA = 10;
const int I2C_SCL = 21;

Adafruit_ADS1115 ads;

// OPA323 output saturates ~34 mV below the 3.3 V supply rail.
const float SENSOR_SAT_V = 3.2;

adsGain_t gains[] = {
  GAIN_TWOTHIRDS,  // 0: ±6.144V
  GAIN_ONE,        // 1: ±4.096V
  GAIN_TWO,        // 2: ±2.048V
  GAIN_FOUR,       // 3: ±1.024V
  GAIN_EIGHT,      // 4: ±0.512V
  GAIN_SIXTEEN     // 5: ±0.256V
};
float gainVoltages[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
int currentGain = 1;

void setup() {
  Serial.setRxBufferSize(8192);  // headroom for bulk calibration uploads
  Serial.begin(115200);
  delay(300);  // let USB-CDC enumerate and the ADS1115 finish power-on
  LittleFS.begin(true);  // format on first boot / corruption
  Wire.begin(I2C_SDA, I2C_SCL);
  ads.begin(0x48, &Wire);
  ads.setGain(gains[currentGain]);
  ads.setDataRate(RATE_ADS1115_860SPS);
}

// Read 'count' samples, average the raw values, and emit one line.
void sendReading(int count) {
  if (count < 1) count = 1;
  long sum = 0;
  for (int i = 0; i < count; i++) {
    sum += ads.readADC_SingleEnded(0);
  }
  int16_t raw = (int16_t)(sum / count);

  // ADC saturation: raw hit the top of the signed 16-bit range.
  bool adcSat = (raw >= 32767);

  // Sensor saturation: op-amp output near the supply rail. Only possible when
  // the gain full-scale exceeds SENSOR_SAT_V; otherwise the ADC overflows
  // before the sensor can saturate.
  bool sensorSat = false;
  if (gainVoltages[currentGain] > SENSOR_SAT_V) {
    int16_t satThreshold = (int16_t)(SENSOR_SAT_V / gainVoltages[currentGain] * 32767);
    sensorSat = (raw >= satThreshold);
  }

  Serial.print(raw);
  Serial.print(",");
  Serial.print(sensorSat ? 1 : 0);
  Serial.print(",");
  Serial.println(adcSat ? 1 : 0);
}

// After an 'r', read an optional decimal sample count terminated by a
// non-digit (newline). Returns 1 if no digits are sent. A short timeout
// guards against a missing terminator.
int readCount() {
  long n = 0;
  bool anyDigit = false;
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c >= '0' && c <= '9') {
        n = n * 10 + (c - '0');
        anyDigit = true;
        t0 = millis();
      } else {
        break;  // terminator (newline/other)
      }
    }
  }
  if (!anyDigit || n < 1) return 1;
  if (n > 1000) n = 1000;  // sanity clamp
  return (int)n;
}

// Read a non-negative decimal integer terminated by a non-digit (newline).
// Short timeout guards a missing terminator. Returns -1 if nothing read.
long readLong() {
  long n = -1;
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c >= '0' && c <= '9') {
        if (n < 0) n = 0;
        n = n * 10 + (c - '0');
        t0 = millis();
      } else {
        break;
      }
    }
  }
  return n;
}

// Receive N bytes from serial into the cal file. Host sends "W<N>\n" then N
// raw bytes. Replies "ok <crc32>" on success or "err" on timeout/short read.
void writeCalibration() {
  long n = readLong();
  if (n <= 0 || n > 65536) { Serial.println("err"); return; }
  // Buffer the whole blob in RAM first. Writing to flash chunk-by-chunk while
  // still receiving lets the USB-CDC RX buffer overflow (flash writes stall the
  // read loop), dropping bytes — so receive fully, then write once.
  uint8_t* data = (uint8_t*)malloc(n);
  if (!data) { Serial.println("err"); return; }
  long got = 0;
  unsigned long t0 = millis();
  while (got < n && millis() - t0 < 5000) {
    int r = Serial.readBytes(data + got, n - got);
    if (r > 0) {
      got += r;
      t0 = millis();
    }
  }
  if (got != n) { free(data); Serial.println("err"); return; }
  File f = LittleFS.open(CAL_PATH, "w");
  if (!f) { free(data); Serial.println("err"); return; }
  size_t wrote = f.write(data, n);
  f.close();
  uint32_t crc = crc32_update(0xFFFFFFFF, data, n) ^ 0xFFFFFFFF;
  free(data);
  if ((long)wrote == n) {
    Serial.print("ok ");
    Serial.println(crc);
  } else {
    LittleFS.remove(CAL_PATH);
    Serial.println("err");
  }
}

// Send the cal file: header line "<size> <crc32>\n" then <size> raw bytes.
// Reports "0 0" if no calibration is stored.
void readCalibration() {
  File f = LittleFS.open(CAL_PATH, "r");
  if (!f) { Serial.println("0 0"); return; }
  size_t sz = f.size();
  uint32_t crc = 0xFFFFFFFF;
  uint8_t buf[256];
  while (f.available()) {
    int r = f.read(buf, sizeof(buf));
    if (r > 0) crc = crc32_update(crc, buf, r);
  }
  Serial.print(sz);
  Serial.print(" ");
  Serial.println(crc ^ 0xFFFFFFFF);
  f.seek(0);
  while (f.available()) {
    int r = f.read(buf, sizeof(buf));
    if (r > 0) Serial.write(buf, r);
  }
  f.close();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      sendReading(readCount());

    } else if (cmd == 'p') {
      Serial.println("pong");  // CDC health check, no I2C

    } else if (cmd == 'g') {
      while (!Serial.available());
      char c = Serial.read();
      int g = c - '0';
      if (g >= 0 && g <= 5) {
        currentGain = g;
        ads.setGain(gains[currentGain]);
        Serial.println("ok");
      } else {
        Serial.println("err");
      }

    } else if (cmd == 'G') {
      Serial.println(currentGain);

    } else if (cmd == 'W') {
      writeCalibration();  // W<N>\n then N bytes -> "ok <crc>" / "err"

    } else if (cmd == 'C') {
      readCalibration();   // -> "<size> <crc>\n" then <size> bytes

    } else if (cmd == 'H') {
      File f = LittleFS.open(CAL_PATH, "r");  // has-cal: size or 0
      Serial.println(f ? (long)f.size() : 0);
      if (f) f.close();

    } else if (cmd == 'X') {
      Serial.println(LittleFS.remove(CAL_PATH) ? "ok" : "err");  // erase cal
    }
  }
}
