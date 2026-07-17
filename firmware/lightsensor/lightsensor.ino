// LightSensor firmware for the ESP32-C3 SuperMini.
// Native USB CDC is used for Serial (build with CDCOnBoot=cdc).

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LittleFS.h>

// Protocol/firmware identity. Bump PROTO_VERSION on any breaking change to the
// serial command set or response formats; bump FW_VERSION for any release.
#define PROTO_VERSION 2
#define FW_VERSION "2.1.0"

// Error codes returned as "err <code>" by command-style responses. 0 is never
// emitted (success uses "ok"). Keep in sync with lightsensor.py ERR_*.
//   1 bad argument        4 transfer timeout / short read
//   2 bad length          5 filesystem open failed
//   3 out of memory       6 write size mismatch
//   7 erase failed
#define ERR_BAD_ARG  1
#define ERR_BAD_LEN  2
#define ERR_NO_MEM   3
#define ERR_TIMEOUT  4
#define ERR_FS_OPEN  5
#define ERR_WRITE    6
#define ERR_ERASE    7

void sendErr(int code) {
  Serial.print("err ");
  Serial.println(code);
}

// Calibration blob (spectral responsivity CSV + metadata header) lives in a
// LittleFS file. The host writes/reads it whole; integrity is checked with a
// CRC32 (standard reflected poly, matches Python's binascii.crc32).
const char* CAL_PATH = "/cal.csv";

// Per-device electrical dark baseline. The nominal value is the R1/R3 divider:
// 3.3 V × 270 Ω / (13 kΩ + 270 Ω). A calibrated value overrides it in LittleFS.
const char* DARK_OFFSET_PATH = "/dark_offset_v";
const float DEFAULT_DARK_OFFSET_V = 3.3f * 270.0f / (13000.0f + 270.0f);
const float DARK_OFFSET_LIMIT_V = 0.25f;

// Running CRC32 (no final XOR until done). Feed bytes incrementally.
uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
  }
  return crc;
}

bool isValidDarkOffset(float offset) {
  return isfinite(offset) && offset >= -DARK_OFFSET_LIMIT_V && offset <= DARK_OFFSET_LIMIT_V;
}

float readDarkOffset() {
  File f = LittleFS.open(DARK_OFFSET_PATH, "r");
  if (!f) return DEFAULT_DARK_OFFSET_V;
  char text[32];
  size_t n = f.readBytesUntil('\n', text, sizeof(text) - 1);
  f.close();
  text[n] = '\0';
  char* end;
  float offset = strtof(text, &end);
  return end != text && *end == '\0' && isValidDarkOffset(offset)
    ? offset
    : DEFAULT_DARK_OFFSET_V;
}

bool writeDarkOffset(float offset) {
  File f = LittleFS.open(DARK_OFFSET_PATH, "w");
  if (!f) return false;
  size_t written = f.print(offset, 9);
  written += f.print('\n');
  f.close();
  return written > 1;
}

bool readDarkOffsetArgument(float* offset) {
  char text[32];
  size_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (Serial.available() <= 0) continue;
    char c = Serial.read();
    if (c == '\n' || c == '\r') break;
    if (n >= sizeof(text) - 1) return false;
    text[n++] = c;
    t0 = millis();
  }
  text[n] = '\0';
  char* end;
  float value = strtof(text, &end);
  if (end == text || *end != '\0' || !isValidDarkOffset(value)) return false;
  *offset = value;
  return true;
}

// ESP32-C3 SuperMini I2C pins (v3 wiring: SDA/SCL next to 3V3/GND).
const int I2C_SDA = 4;
const int I2C_SCL = 3;

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
// Autogain (autoexposure) mode. When on, `r` reads step the gain until the
// signal is in-band, then average. Any manual `g<n>` turns it off.
bool autoGain = false;
// Autoexposure band on % of full scale: wider than the 2x adjacent-gain
// ratio, so a single step always lands in-band (no oscillation).
const float AUTOGAIN_LOW_PCT = 40.0;
const float AUTOGAIN_HIGH_PCT = 90.0;

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

// True if a raw sample overflows the ADC counter at the current gain.
bool isAdcSat(int16_t raw) {
  return raw >= 32767;
}

// True if the op-amp output is near the supply rail. Only possible when the
// gain full-scale exceeds SENSOR_SAT_V; otherwise the ADC overflows first.
bool isSensorSat(int16_t raw) {
  if (gainVoltages[currentGain] <= SENSOR_SAT_V) return false;
  int16_t thr = (int16_t)(SENSOR_SAT_V / gainVoltages[currentGain] * 32767);
  return raw >= thr;
}

// Step the gain (single samples) until the signal is in-band or a gain rail
// (0 = widest, ngains-1 = most sensitive) is reached. Bounded by ngains.
void autoExpose() {
  int ngains = (int)(sizeof(gainVoltages) / sizeof(gainVoltages[0]));
  for (int iter = 0; iter < ngains; iter++) {
    int16_t raw = ads.readADC_SingleEnded(0);
    float pct = (float)raw / 32767.0 * 100.0;
    bool over = isSensorSat(raw) || isAdcSat(raw) || pct >= AUTOGAIN_HIGH_PCT;
    bool under = pct < AUTOGAIN_LOW_PCT;
    if (over && currentGain > 0) {
      currentGain--;  // wider range, less sensitive
      ads.setGain(gains[currentGain]);
    } else if (under && currentGain < ngains - 1) {
      currentGain++;  // narrower range, more sensitive
      ads.setGain(gains[currentGain]);
    } else {
      break;  // in-band, or railed
    }
  }
}

// Read 'count' samples (after autoexposing if enabled), average the raw
// values, and emit one line: "raw,sensor_sat,adc_sat,gain".
void sendReading(int count) {
  if (autoGain) autoExpose();
  if (count < 1) count = 1;
  long sum = 0;
  for (int i = 0; i < count; i++) {
    sum += ads.readADC_SingleEnded(0);
  }
  int16_t raw = (int16_t)(sum / count);

  Serial.print(raw);
  Serial.print(",");
  Serial.print(isSensorSat(raw) ? 1 : 0);
  Serial.print(",");
  Serial.print(isAdcSat(raw) ? 1 : 0);
  Serial.print(",");
  Serial.println(currentGain);
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
  if (n <= 0 || n > 65536) { sendErr(ERR_BAD_LEN); return; }
  // Buffer the whole blob in RAM first. Writing to flash chunk-by-chunk while
  // still receiving lets the USB-CDC RX buffer overflow (flash writes stall the
  // read loop), dropping bytes — so receive fully, then write once.
  uint8_t* data = (uint8_t*)malloc(n);
  if (!data) { sendErr(ERR_NO_MEM); return; }
  long got = 0;
  unsigned long t0 = millis();
  while (got < n && millis() - t0 < 5000) {
    int r = Serial.readBytes(data + got, n - got);
    if (r > 0) {
      got += r;
      t0 = millis();
    }
  }
  if (got != n) { free(data); sendErr(ERR_TIMEOUT); return; }
  File f = LittleFS.open(CAL_PATH, "w");
  if (!f) { free(data); sendErr(ERR_FS_OPEN); return; }
  size_t wrote = f.write(data, n);
  f.close();
  uint32_t crc = crc32_update(0xFFFFFFFF, data, n) ^ 0xFFFFFFFF;
  free(data);
  if ((long)wrote == n) {
    Serial.print("ok ");
    Serial.println(crc);
  } else {
    LittleFS.remove(CAL_PATH);
    sendErr(ERR_WRITE);
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

// Identity line for the host handshake. Space-separated key=value pairs after a
// fixed product token, e.g.:
//   lightsensor proto=1 fw=1.0.0 id=AABBCCDDEEFF sps=860 vsat=3.20 gains=6.144,...
// id is the 48-bit eFuse MAC (unique per chip), usable as a serial number.
// The firmware is the source of truth for the gain table and saturation
// voltage; it reports them here so the host driver can verify its mirror.
void sendIdentity() {
  uint64_t mac = ESP.getEfuseMac();
  char id[13];
  snprintf(id, sizeof(id), "%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);
  int ngains = (int)(sizeof(gainVoltages) / sizeof(gainVoltages[0]));
  Serial.print("lightsensor proto=");
  Serial.print(PROTO_VERSION);
  Serial.print(" fw=");
  Serial.print(FW_VERSION);
  Serial.print(" id=");
  Serial.print(id);
  Serial.print(" sps=860 vsat=");
  Serial.print(SENSOR_SAT_V);
  Serial.print(" dark=");
  Serial.print(readDarkOffset(), 6);
  Serial.print(" gains=");
  for (int i = 0; i < ngains; i++) {
    if (i) Serial.print(",");
    Serial.print(gainVoltages[i], 3);
  }
  Serial.println();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      sendReading(readCount());

    } else if (cmd == 'p') {
      Serial.println("pong");  // CDC health check, no I2C

    } else if (cmd == 'I') {
      sendIdentity();  // product/proto/fw/id line for host handshake

    } else if (cmd == 'D') {
      Serial.println(readDarkOffset(), 9);

    } else if (cmd == 'd') {
      float offset;
      if (!readDarkOffsetArgument(&offset)) {
        sendErr(ERR_BAD_ARG);
      } else if (writeDarkOffset(offset)) {
        Serial.println("ok");
      } else {
        sendErr(ERR_WRITE);
      }

    } else if (cmd == 'g') {
      while (!Serial.available());
      char c = Serial.read();
      int g = c - '0';
      if (g >= 0 && g <= 5) {
        currentGain = g;
        ads.setGain(gains[currentGain]);
        autoGain = false;  // manual gain turns autoexposure off
        Serial.println("ok");
      } else {
        sendErr(ERR_BAD_ARG);
      }

    } else if (cmd == 'G') {
      Serial.println(currentGain);

    } else if (cmd == 'a') {
      while (!Serial.available());
      char c = Serial.read();  // a1 = enable, a0 = disable
      if (c == '1') {
        autoGain = true;
        Serial.println("ok");
      } else if (c == '0') {
        autoGain = false;
        Serial.println("ok");
      } else {
        sendErr(ERR_BAD_ARG);
      }

    } else if (cmd == 'A') {
      Serial.print(autoGain ? 1 : 0);  // autogain status + current gain
      Serial.print(" ");
      Serial.println(currentGain);

    } else if (cmd == 'W') {
      writeCalibration();  // W<N>\n then N bytes -> "ok <crc>" / "err <code>"

    } else if (cmd == 'C') {
      readCalibration();   // -> "<size> <crc>\n" then <size> bytes

    } else if (cmd == 'H') {
      File f = LittleFS.open(CAL_PATH, "r");  // has-cal: size or 0
      Serial.println(f ? (long)f.size() : 0);
      if (f) f.close();

    } else if (cmd == 'X') {
      if (LittleFS.remove(CAL_PATH)) Serial.println("ok");
      else sendErr(ERR_ERASE);  // erase cal
    }
  }
}
