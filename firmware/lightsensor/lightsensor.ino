// LightSensor v3 production firmware for the RP2040/ADS1220 board.
// Native USB CDC carries binary protocol v3 frames.

#include <Arduino.h>
#include <LittleFS.h>
#include <SPI.h>
#include <USB.h>
#include <Wire.h>
#include <cmath>
#include <hardware/flash.h>
#include <pico/time.h>

namespace {

constexpr uint8_t PROTOCOL_VERSION = 3;
constexpr uint8_t FIRMWARE_MAJOR = 3;
constexpr uint8_t FIRMWARE_MINOR = 0;
constexpr uint8_t FIRMWARE_PATCH = 0;
constexpr uint8_t HARDWARE_MAJOR = 3;

constexpr uint8_t ADC_MISO = 0;
constexpr uint8_t ADC_CS = 1;
constexpr uint8_t ADC_SCK = 2;
constexpr uint8_t ADC_MOSI = 3;
constexpr uint8_t ADC_DRDY = 4;
constexpr uint8_t TMP_SDA = 10;
constexpr uint8_t TMP_SCL = 11;
constexpr uint8_t TMP_ADDRESS = 0x48;

constexpr uint32_t ADC_SPI_HZ = 1'000'000;
constexpr float ADC_REFERENCE_V = 2.048f;
constexpr int32_t ADC_POSITIVE_FULL_SCALE = 8'388'607;
constexpr int32_t ADC_NEGATIVE_FULL_SCALE = -8'388'608;
constexpr int32_t AUTOGAIN_LOW_CODE = 3'355'442;   // 40% of positive full scale.
constexpr int32_t AUTOGAIN_HIGH_CODE = 7'130'316;  // 85% of positive full scale.
constexpr int32_t TIA_POSITIVE_CLIP_CODE_GAIN_1 = 6'717'440;  // 1.64 V.
constexpr float TIA_POSITIVE_CLIP_V = 1.64f;
constexpr float DARK_LIMIT_V = 0.25f;
constexpr uint32_t ADC_TIMEOUT_US = 300'000;
constexpr uint32_t TEMPERATURE_PERIOD_US = 100'000;
constexpr uint32_t STREAM_ACK_TIMEOUT_MS = 1'000;

constexpr uint8_t ADS_CMD_RESET = 0x06;
constexpr uint8_t ADS_CMD_START_SYNC = 0x08;
constexpr uint8_t ADS_CMD_POWERDOWN = 0x02;
constexpr uint8_t ADS_CMD_RREG = 0x20;
constexpr uint8_t ADS_CMD_WREG = 0x40;

constexpr size_t MAX_DECODED_FRAME = 256;
constexpr size_t MAX_PAYLOAD = 248;
constexpr size_t MAX_ENCODED_FRAME = 258;
constexpr size_t MAX_WINDOW = 1024;

constexpr char DARK_PATH[] = "/dark.bin";
constexpr char DARK_TEMP_PATH[] = "/dark.tmp";
constexpr uint16_t PERSISTENCE_SCHEMA = 1;
constexpr uint16_t PERSISTENCE_KIND_DARK = 1;
constexpr size_t DARK_RECORD_SIZE = 20;

constexpr uint32_t CAP_RAW_STREAM = 1u << 0;
constexpr uint32_t CAP_VOLTS_STREAM = 1u << 1;
constexpr uint32_t CAP_FINITE_STREAM = 1u << 2;
constexpr uint32_t CAP_VOLTS_AUTOGAIN = 1u << 3;
constexpr uint32_t CAP_TEMPERATURE = 1u << 4;
constexpr uint32_t CAP_SESSION_DARK = 1u << 5;
constexpr uint32_t CAP_PERSISTENT_DARK = 1u << 6;
constexpr uint32_t CAP_STORAGE_RESET = 1u << 7;
constexpr uint32_t CAPABILITIES = CAP_RAW_STREAM | CAP_VOLTS_STREAM | CAP_FINITE_STREAM
  | CAP_VOLTS_AUTOGAIN | CAP_TEMPERATURE | CAP_SESSION_DARK
  | CAP_PERSISTENT_DARK | CAP_STORAGE_RESET;

enum MessageType : uint8_t {
  MSG_HELLO = 0x01,
  MSG_TIME_SYNC = 0x02,
  MSG_PING = 0x03,
  MSG_GET_STATUS = 0x04,
  MSG_LIST_PROFILES = 0x05,
  MSG_START_STREAM = 0x10,
  MSG_ACK_STREAM = 0x11,
  MSG_STOP_STREAM = 0x12,
  MSG_SET_SESSION_DARK = 0x20,
  MSG_CLEAR_SESSION_DARK = 0x21,
  MSG_SAVE_SESSION_DARK = 0x22,
  MSG_RESET_STORAGE = 0x30,

  MSG_HELLO_REPLY = 0x81,
  MSG_TIME_SYNCED = 0x82,
  MSG_PONG = 0x83,
  MSG_STATUS = 0x84,
  MSG_PROFILES = 0x85,
  MSG_STREAM_STARTED = 0x90,
  MSG_SAMPLE_RAW = 0x91,
  MSG_SAMPLE_VOLTS = 0x92,
  MSG_STREAM_STOPPED = 0x93,
  MSG_OK = 0xA0,
  MSG_ERROR = 0xFF,
};

enum ErrorCode : uint16_t {
  ERR_BAD_FRAME = 1,
  ERR_BAD_VERSION = 2,
  ERR_BAD_SCHEMA = 3,
  ERR_BAD_CRC = 4,
  ERR_UNKNOWN_TYPE = 5,
  ERR_BAD_ARGUMENT = 6,
  ERR_BAD_STATE = 7,
  ERR_TIME_NOT_SYNCED = 8,
  ERR_ACK_TIMEOUT = 9,
  ERR_ADC_CONFIG = 10,
  ERR_ADC_TIMEOUT = 11,
  ERR_OVERFLOW = 12,
  ERR_TEMPERATURE = 13,
  ERR_STORAGE_MOUNT = 14,
  ERR_STORAGE_INTEGRITY = 15,
  ERR_STORAGE_WRITE = 16,
  ERR_STORAGE_CONFIRMATION = 17,
  ERR_INTERNAL = 18,
};

enum DeviceState : uint8_t {
  STATE_STOPPED = 0,
  STATE_AWAITING_ACK = 1,
  STATE_STREAMING = 2,
};

enum StorageState : uint8_t {
  STORAGE_EMPTY = 0,
  STORAGE_VALID = 1,
  STORAGE_FAULT = 2,
};

enum StreamFormat : uint8_t {
  FORMAT_RAW = 0,
  FORMAT_VOLTS = 1,
};

enum StreamMode : uint8_t {
  MODE_CONTINUOUS = 0,
  MODE_FINITE = 1,
};

enum StopReason : uint8_t {
  STOP_REQUESTED = 0,
  STOP_FINITE_COMPLETE = 1,
  STOP_REPLACED = 2,
};

enum StatusFlag : uint8_t {
  STATUS_ADC_POSITIVE_CLIP = 1u << 0,
  STATUS_ADC_NEGATIVE_CLIP = 1u << 1,
  STATUS_TIA_POSITIVE_CLIP = 1u << 2,
  STATUS_AUTOGAIN_OVERRANGE = 1u << 4,
  STATUS_AUTOGAIN_UNDERRANGE = 1u << 5,
};

struct Profile {
  uint8_t id;
  const char *name;
  uint8_t register1;
  uint8_t register2;
  uint32_t measuredMilliSps;
};

constexpr Profile PROFILES[] = {
  {0, "normal_20_50_60", 0x04, 0x10, 19'958},
  {1, "normal_330", 0x84, 0x00, 327'876},
  {2, "turbo_2000", 0xD4, 0x00, 1'949'300},
};
constexpr size_t PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);

struct StreamConfig {
  uint8_t format;
  uint8_t mode;
  uint8_t profileId;
  uint8_t gainIndex;
  uint8_t autogain;
  uint16_t window;
  uint32_t outputCount;
};

class BufferWriter {
 public:
  BufferWriter(uint8_t *data, size_t capacity) : data_(data), capacity_(capacity) {}

  void putU8(uint8_t value) {
    if (position_ < capacity_) data_[position_++] = value;
    else ok_ = false;
  }

  void putU16(uint16_t value) {
    putU8(static_cast<uint8_t>(value));
    putU8(static_cast<uint8_t>(value >> 8));
  }

  void putU32(uint32_t value) {
    for (uint8_t shift = 0; shift < 32; shift += 8) putU8(static_cast<uint8_t>(value >> shift));
  }

  void putU64(uint64_t value) {
    for (uint8_t shift = 0; shift < 64; shift += 8) putU8(static_cast<uint8_t>(value >> shift));
  }

  void putF32(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    putU32(bits);
  }

  void putBytes(const uint8_t *bytes, size_t length) {
    for (size_t i = 0; i < length; i++) putU8(bytes[i]);
  }

  void putText(const char *text) {
    putBytes(reinterpret_cast<const uint8_t *>(text), strlen(text));
  }

  bool ok() const { return ok_; }
  size_t size() const { return position_; }

 private:
  uint8_t *data_;
  size_t capacity_;
  size_t position_ = 0;
  bool ok_ = true;
};

class BufferReader {
 public:
  BufferReader(const uint8_t *data, size_t length) : data_(data), length_(length) {}

  uint8_t getU8() {
    if (position_ >= length_) {
      ok_ = false;
      return 0;
    }
    return data_[position_++];
  }

  uint16_t getU16() {
    uint16_t value = getU8();
    value |= static_cast<uint16_t>(getU8()) << 8;
    return value;
  }

  uint32_t getU32() {
    uint32_t value = 0;
    for (uint8_t shift = 0; shift < 32; shift += 8) value |= static_cast<uint32_t>(getU8()) << shift;
    return value;
  }

  uint64_t getU64() {
    uint64_t value = 0;
    for (uint8_t shift = 0; shift < 64; shift += 8) value |= static_cast<uint64_t>(getU8()) << shift;
    return value;
  }

  float getF32() {
    uint32_t bits = getU32();
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
  }

  bool matches(const char *text) {
    size_t length = strlen(text);
    if (position_ + length > length_) {
      ok_ = false;
      return false;
    }
    bool equal = memcmp(data_ + position_, text, length) == 0;
    position_ += length;
    return equal;
  }

  bool finished() const { return ok_ && position_ == length_; }
  bool ok() const { return ok_; }

 private:
  const uint8_t *data_;
  size_t length_;
  size_t position_ = 0;
  bool ok_ = true;
};

SPISettings adcSpiSettings(ADC_SPI_HZ, MSBFIRST, SPI_MODE1);
volatile uint32_t drdyEvents = 0;
volatile uint64_t lastDrdyTimestampUs = 0;

uint8_t receiveEncoded[MAX_ENCODED_FRAME];
size_t receiveEncodedLength = 0;
bool receiveOverflow = false;
uint8_t receiveDecoded[MAX_DECODED_FRAME];
uint8_t transmitPayload[MAX_PAYLOAD];
uint8_t transmitDecoded[MAX_DECODED_FRAME];
uint8_t transmitEncoded[MAX_ENCODED_FRAME];

uint8_t flashUniqueId[FLASH_UNIQUE_ID_SIZE_BYTES] = {};
uint64_t deviceId = 0;
char usbSerial[17] = {};

DeviceState deviceState = STATE_STOPPED;
StorageState storageState = STORAGE_EMPTY;
bool filesystemMounted = false;
bool adcReady = false;
bool timeSynchronized = false;
uint64_t synchronizedUtcUs = 0;
uint64_t synchronizedMonotonicUs = 0;
uint16_t lastErrorCode = 0;

float deviceDarkVolts = 0.0f;
bool sessionDarkActive = false;
float sessionDarkVolts = 0.0f;
float temperatureC = NAN;
bool temperatureValid = false;
uint64_t nextTemperatureUs = 0;

StreamConfig streamConfig = {};
uint64_t streamStartDeviceUs = 0;
uint64_t streamStartUtcUs = 0;
uint32_t streamSequence = 0;
uint32_t deliveredOutputs = 0;
uint32_t streamAckDeadlineMs = 0;
uint64_t lastConversionActivityUs = 0;

int32_t windowValues[MAX_WINDOW];
uint16_t windowCount = 0;
uint16_t windowPosition = 0;
int64_t windowSum = 0;

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output, size_t capacity) {
  if (capacity == 0) return 0;
  size_t readIndex = 0;
  size_t writeIndex = 1;
  size_t codeIndex = 0;
  uint8_t code = 1;

  while (readIndex < length) {
    if (input[readIndex] == 0) {
      if (codeIndex >= capacity) return 0;
      output[codeIndex] = code;
      code = 1;
      codeIndex = writeIndex++;
      if (writeIndex > capacity) return 0;
      readIndex++;
    } else {
      if (writeIndex >= capacity) return 0;
      output[writeIndex++] = input[readIndex++];
      code++;
      if (code == 0xFF) {
        if (codeIndex >= capacity) return 0;
        output[codeIndex] = code;
        code = 1;
        codeIndex = writeIndex++;
        if (writeIndex > capacity) return 0;
      }
    }
  }
  if (codeIndex >= capacity) return 0;
  output[codeIndex] = code;
  return writeIndex;
}

size_t cobsDecode(const uint8_t *input, size_t length, uint8_t *output, size_t capacity) {
  size_t readIndex = 0;
  size_t writeIndex = 0;
  while (readIndex < length) {
    uint8_t code = input[readIndex++];
    if (code == 0) return 0;
    size_t copyLength = static_cast<size_t>(code - 1);
    if (readIndex + copyLength > length || writeIndex + copyLength > capacity) return 0;
    for (size_t i = 0; i < copyLength; i++) output[writeIndex++] = input[readIndex++];
    if (code != 0xFF && readIndex < length) {
      if (writeIndex >= capacity) return 0;
      output[writeIndex++] = 0;
    }
  }
  return writeIndex;
}

bool sendFrame(uint8_t messageType, const uint8_t *payload, size_t payloadLength) {
  if (payloadLength > MAX_PAYLOAD) return false;
  BufferWriter writer(transmitDecoded, sizeof(transmitDecoded));
  writer.putU8(PROTOCOL_VERSION);
  writer.putU8(messageType);
  writer.putU16(static_cast<uint16_t>(payloadLength));
  writer.putBytes(payload, payloadLength);
  if (!writer.ok()) return false;
  uint32_t checksum = crc32(transmitDecoded, writer.size());
  writer.putU32(checksum);
  if (!writer.ok()) return false;
  size_t encodedLength = cobsEncode(
    transmitDecoded, writer.size(), transmitEncoded, sizeof(transmitEncoded)
  );
  if (encodedLength == 0) return false;
  size_t written = Serial.write(transmitEncoded, encodedLength);
  written += Serial.write(static_cast<uint8_t>(0));
  return written == encodedLength + 1;
}

void adcSelect() {
  SPI.beginTransaction(adcSpiSettings);
  digitalWrite(ADC_CS, LOW);
  delayMicroseconds(1);
}

void adcDeselect() {
  digitalWrite(ADC_CS, HIGH);
  SPI.endTransaction();
}

void adsCommand(uint8_t command) {
  adcSelect();
  SPI.transfer(command);
  adcDeselect();
}

void adsReadRegisters(uint8_t registers[4]) {
  adcSelect();
  SPI.transfer(ADS_CMD_RREG | 0x03);
  for (size_t i = 0; i < 4; i++) registers[i] = SPI.transfer(0x00);
  adcDeselect();
}

void adsWriteRegisters(const uint8_t registers[4]) {
  adcSelect();
  SPI.transfer(ADS_CMD_WREG | 0x03);
  for (size_t i = 0; i < 4; i++) SPI.transfer(registers[i]);
  adcDeselect();
}

void clearDrdyEvents() {
  noInterrupts();
  drdyEvents = 0;
  lastDrdyTimestampUs = 0;
  interrupts();
}

void onAdcDrdy() {
  lastDrdyTimestampUs = time_us_64();
  drdyEvents++;
}

bool takeDrdyEvents(uint32_t &events, uint64_t &timestampUs) {
  noInterrupts();
  events = drdyEvents;
  timestampUs = lastDrdyTimestampUs;
  if (events > 0) drdyEvents = 0;
  interrupts();
  return events > 0;
}

int32_t adsReadRaw() {
  adcSelect();
  uint32_t raw = static_cast<uint32_t>(SPI.transfer(0x00)) << 16;
  raw |= static_cast<uint32_t>(SPI.transfer(0x00)) << 8;
  raw |= SPI.transfer(0x00);
  adcDeselect();
  if (raw & 0x0080'0000) raw |= 0xFF00'0000;
  return static_cast<int32_t>(raw);
}

void stopAdc() {
  adsCommand(ADS_CMD_POWERDOWN);
  clearDrdyEvents();
}

bool configureAdc(uint8_t profileId, uint8_t gainIndex) {
  if (profileId >= PROFILE_COUNT || gainIndex > 7) return false;
  const Profile &profile = PROFILES[profileId];
  uint8_t requested[4] = {
    static_cast<uint8_t>((gainIndex & 0x07) << 1),
    profile.register1,
    profile.register2,
    0x00,
  };
  stopAdc();
  delayMicroseconds(100);
  adsWriteRegisters(requested);
  uint8_t actual[4];
  adsReadRegisters(actual);
  if (memcmp(requested, actual, sizeof(actual)) != 0) return false;
  clearDrdyEvents();
  adsCommand(ADS_CMD_START_SYNC);
  lastConversionActivityUs = time_us_64();
  return true;
}

bool initializeAdc() {
  adsCommand(ADS_CMD_RESET);
  delayMicroseconds(500);
  uint8_t registers[4];
  adsReadRegisters(registers);
  bool reset = registers[0] == 0 && registers[1] == 0
    && registers[2] == 0 && registers[3] == 0;
  adsCommand(ADS_CMD_POWERDOWN);
  return reset;
}

bool writeTmpRegister(uint8_t address, uint16_t value) {
  Wire1.beginTransmission(TMP_ADDRESS);
  Wire1.write(address);
  Wire1.write(static_cast<uint8_t>(value >> 8));
  Wire1.write(static_cast<uint8_t>(value));
  return Wire1.endTransmission() == 0;
}

bool readTmpRegister(uint8_t address, uint16_t &value) {
  Wire1.beginTransmission(TMP_ADDRESS);
  Wire1.write(address);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(TMP_ADDRESS, static_cast<uint8_t>(2)) != 2) return false;
  value = static_cast<uint16_t>(Wire1.read()) << 8;
  value |= static_cast<uint16_t>(Wire1.read());
  return true;
}

bool configureTemperature() {
  uint16_t device;
  uint16_t configuration;
  return readTmpRegister(0x0F, device)
    && (device & 0x0FFF) == 0x0117
    && writeTmpRegister(0x01, 0x0000)
    && readTmpRegister(0x01, configuration)
    && configuration == 0x0000;
}

bool refreshTemperature() {
  uint16_t raw;
  if (!readTmpRegister(0x00, raw)) return false;
  temperatureC = static_cast<int16_t>(raw) * 0.0078125f;
  temperatureValid = std::isfinite(temperatureC)
    && temperatureC >= -55.0f && temperatureC <= 150.0f;
  return temperatureValid;
}

float activeDarkVolts() {
  return sessionDarkActive ? sessionDarkVolts : deviceDarkVolts;
}

uint8_t activeDarkSource() {
  return sessionDarkActive ? 1 : 0;
}

bool parseDarkRecord(const uint8_t record[DARK_RECORD_SIZE], float &value) {
  if (memcmp(record, "LSV3", 4) != 0) return false;
  BufferReader reader(record + 4, DARK_RECORD_SIZE - 4);
  uint16_t schema = reader.getU16();
  uint16_t kind = reader.getU16();
  uint32_t payloadLength = reader.getU32();
  value = reader.getF32();
  uint32_t expectedCrc = reader.getU32();
  return reader.finished()
    && schema == PERSISTENCE_SCHEMA
    && kind == PERSISTENCE_KIND_DARK
    && payloadLength == sizeof(float)
    && std::isfinite(value)
    && fabsf(value) <= DARK_LIMIT_V
    && crc32(record, DARK_RECORD_SIZE - sizeof(uint32_t)) == expectedCrc;
}

bool readDarkFile(const char *path, float &value) {
  File file = LittleFS.open(path, "r");
  if (!file || file.size() != DARK_RECORD_SIZE) {
    if (file) file.close();
    return false;
  }
  uint8_t record[DARK_RECORD_SIZE];
  bool complete = file.read(record, sizeof(record)) == sizeof(record);
  file.close();
  return complete && parseDarkRecord(record, value);
}

void loadPersistence() {
  deviceDarkVolts = 0.0f;
  sessionDarkActive = false;
  if (!filesystemMounted) {
    storageState = STORAGE_FAULT;
    return;
  }
  LittleFS.remove(DARK_TEMP_PATH);
  if (!LittleFS.exists(DARK_PATH)) {
    storageState = STORAGE_EMPTY;
    return;
  }
  float value;
  if (readDarkFile(DARK_PATH, value)) {
    deviceDarkVolts = value;
    storageState = STORAGE_VALID;
  } else {
    storageState = STORAGE_FAULT;
  }
}

bool writeDarkFile(float value) {
  if (!filesystemMounted || !std::isfinite(value) || fabsf(value) > DARK_LIMIT_V) return false;
  uint8_t record[DARK_RECORD_SIZE];
  BufferWriter writer(record, sizeof(record));
  writer.putText("LSV3");
  writer.putU16(PERSISTENCE_SCHEMA);
  writer.putU16(PERSISTENCE_KIND_DARK);
  writer.putU32(sizeof(float));
  writer.putF32(value);
  writer.putU32(crc32(record, DARK_RECORD_SIZE - sizeof(uint32_t)));
  if (!writer.ok() || writer.size() != sizeof(record)) return false;

  LittleFS.remove(DARK_TEMP_PATH);
  File file = LittleFS.open(DARK_TEMP_PATH, "w");
  if (!file) return false;
  bool written = file.write(record, sizeof(record)) == sizeof(record);
  file.flush();
  file.close();
  float verified;
  if (!written || !readDarkFile(DARK_TEMP_PATH, verified) || verified != value) {
    LittleFS.remove(DARK_TEMP_PATH);
    return false;
  }
  if (!LittleFS.rename(DARK_TEMP_PATH, DARK_PATH)) {
    LittleFS.remove(DARK_TEMP_PATH);
    return false;
  }
  deviceDarkVolts = value;
  storageState = STORAGE_VALID;
  return true;
}

bool resetStorage() {
  stopAdc();
  bool formatted = LittleFS.format();
  if (!formatted) return false;
  filesystemMounted = LittleFS.begin();
  if (!filesystemMounted) return false;
  storageState = STORAGE_EMPTY;
  deviceDarkVolts = 0.0f;
  sessionDarkVolts = 0.0f;
  sessionDarkActive = false;
  return true;
}

void resetWindow() {
  windowCount = 0;
  windowPosition = 0;
  windowSum = 0;
}

void addWindowValue(int32_t value) {
  if (windowCount < streamConfig.window) {
    windowValues[windowPosition] = value;
    windowSum += value;
    windowCount++;
  } else {
    windowSum -= windowValues[windowPosition];
    windowValues[windowPosition] = value;
    windowSum += value;
  }
  windowPosition = (windowPosition + 1) % streamConfig.window;
}

bool windowReady() {
  return windowCount == streamConfig.window;
}

void writeStreamConfig(BufferWriter &writer, const StreamConfig &config) {
  writer.putU8(config.format);
  writer.putU8(config.mode);
  writer.putU8(config.profileId);
  writer.putU8(config.gainIndex);
  writer.putU8(config.autogain);
  writer.putU16(config.window);
  writer.putU32(config.outputCount);
}

bool sendOk(uint32_t requestId, uint8_t operation) {
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU8(operation);
  return writer.ok() && sendFrame(MSG_OK, transmitPayload, writer.size());
}

void sendErrorFrame(
  uint32_t requestId,
  uint16_t errorCode,
  uint16_t detail,
  uint64_t token,
  uint32_t lastSequence
) {
  lastErrorCode = errorCode;
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU64(token);
  writer.putU32(lastSequence);
  writer.putU16(errorCode);
  writer.putU16(detail);
  if (writer.ok()) sendFrame(MSG_ERROR, transmitPayload, writer.size());
}

uint32_t lastEmittedSequence() {
  return deliveredOutputs == 0 ? UINT32_MAX : streamSequence - 1;
}

void fail(uint32_t requestId, uint16_t errorCode, uint16_t detail = 0) {
  uint64_t token = streamStartDeviceUs;
  uint32_t lastSequence = lastEmittedSequence();
  if (deviceState != STATE_STOPPED) stopAdc();
  deviceState = STATE_STOPPED;
  sendErrorFrame(requestId, errorCode, detail, token, lastSequence);
}

void sendStreamStopped(uint32_t requestId, uint8_t reason) {
  uint64_t token = streamStartDeviceUs;
  uint32_t count = deliveredOutputs;
  stopAdc();
  deviceState = STATE_STOPPED;
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU64(token);
  writer.putU32(count);
  writer.putU8(reason);
  if (writer.ok()) sendFrame(MSG_STREAM_STOPPED, transmitPayload, writer.size());
}

bool validStreamConfig(const StreamConfig &config) {
  if (config.format > FORMAT_VOLTS || config.mode > MODE_FINITE) return false;
  if (config.profileId >= PROFILE_COUNT || config.gainIndex > 7 || config.autogain > 1) return false;
  if (config.window == 0 || config.window > MAX_WINDOW) return false;
  if (config.mode == MODE_CONTINUOUS && config.outputCount != 0) return false;
  if (config.mode == MODE_FINITE && config.outputCount == 0) return false;
  if (config.format == FORMAT_RAW && config.autogain != 0) return false;
  return true;
}

bool beginStream(uint32_t requestId, const StreamConfig &config) {
  if (!timeSynchronized) {
    fail(requestId, ERR_TIME_NOT_SYNCED);
    return false;
  }
  if (!adcReady) {
    fail(requestId, ERR_ADC_CONFIG);
    return false;
  }
  if (!configureTemperature() || !refreshTemperature()) {
    fail(requestId, ERR_TEMPERATURE);
    return false;
  }
  if (!configureAdc(config.profileId, config.gainIndex)) {
    fail(requestId, ERR_ADC_CONFIG);
    return false;
  }

  streamConfig = config;
  resetWindow();
  streamSequence = 0;
  deliveredOutputs = 0;
  streamStartDeviceUs = time_us_64();
  streamStartUtcUs = synchronizedUtcUs + (streamStartDeviceUs - synchronizedMonotonicUs);
  nextTemperatureUs = streamStartDeviceUs + TEMPERATURE_PERIOD_US;
  deviceState = STATE_AWAITING_ACK;
  streamAckDeadlineMs = millis() + STREAM_ACK_TIMEOUT_MS;

  const Profile &profile = PROFILES[config.profileId];
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU64(streamStartDeviceUs);
  writer.putU64(streamStartUtcUs);
  writeStreamConfig(writer, config);
  writer.putU32(profile.measuredMilliSps);
  writer.putU8(static_cast<uint8_t>(config.gainIndex << 1));
  writer.putU8(profile.register1);
  writer.putU8(profile.register2);
  writer.putU8(0x00);
  writer.putU16(0);
  writer.putU8(0xFF);
  writer.putU16(13'107);
  writer.putU16(27'853);
  writer.putU16(0);
  writer.putU8(activeDarkSource());
  writer.putF32(activeDarkVolts());
  writer.putU32(TEMPERATURE_PERIOD_US);
  if (!writer.ok() || !sendFrame(MSG_STREAM_STARTED, transmitPayload, writer.size())) {
    stopAdc();
    deviceState = STATE_STOPPED;
    return false;
  }
  return true;
}

void restartActiveStream(uint32_t requestId, const StreamConfig &config) {
  if (deviceState != STATE_STOPPED) sendStreamStopped(0, STOP_REPLACED);
  beginStream(requestId, config);
}

void sendHello(uint32_t requestId) {
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU8(FIRMWARE_MAJOR);
  writer.putU8(FIRMWARE_MINOR);
  writer.putU8(FIRMWARE_PATCH);
  writer.putU8(HARDWARE_MAJOR);
  writer.putU32(CAPABILITIES);
  writer.putU16(MAX_DECODED_FRAME);
  writer.putU64(deviceId);
  writer.putU8(timeSynchronized ? 1 : 0);
  writer.putU8(storageState);
  if (writer.ok()) sendFrame(MSG_HELLO_REPLY, transmitPayload, writer.size());
}

void sendTimeSynced(uint32_t requestId, uint64_t utcUs, uint64_t monotonicUs) {
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU64(utcUs);
  writer.putU64(monotonicUs);
  if (writer.ok()) sendFrame(MSG_TIME_SYNCED, transmitPayload, writer.size());
}

void sendStatus(uint32_t requestId) {
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU8(deviceState);
  writer.putU8(timeSynchronized ? 1 : 0);
  writer.putU8(storageState);
  writer.putU8(activeDarkSource());
  writer.putF32(deviceDarkVolts);
  writer.putF32(sessionDarkActive ? sessionDarkVolts : NAN);
  writer.putF32(temperatureValid ? temperatureC : NAN);
  writer.putU64(time_us_64());
  writer.putU16(lastErrorCode);
  if (writer.ok()) sendFrame(MSG_STATUS, transmitPayload, writer.size());
}

void sendProfiles(uint32_t requestId) {
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(requestId);
  writer.putU8(PROFILE_COUNT);
  for (const Profile &profile : PROFILES) {
    size_t nameLength = strlen(profile.name);
    writer.putU8(profile.id);
    writer.putU8(static_cast<uint8_t>(nameLength));
    writer.putText(profile.name);
    writer.putU32(profile.measuredMilliSps);
    writer.putU8(0x00);
    writer.putU8(profile.register1);
    writer.putU8(profile.register2);
    writer.putU8(0x00);
    writer.putU8(0xFF);
    writer.putU16(0);
  }
  if (writer.ok()) sendFrame(MSG_PROFILES, transmitPayload, writer.size());
}

void sendSample(int32_t raw, uint64_t timestampUs, uint8_t gainIndex, uint8_t status) {
  BufferWriter writer(transmitPayload, sizeof(transmitPayload));
  writer.putU32(streamSequence);
  writer.putU64(timestampUs);
  writer.putU8(gainIndex);
  writer.putU8(status);
  if (streamConfig.format == FORMAT_RAW) {
    int32_t mean = static_cast<int32_t>(windowSum / streamConfig.window);
    writer.putU32(static_cast<uint32_t>(mean));
  } else {
    double normalizedMean = static_cast<double>(windowSum) / streamConfig.window;
    float volts = static_cast<float>(
      normalizedMean * ADC_REFERENCE_V / (8'388'608.0 * 128.0)
    ) - activeDarkVolts();
    writer.putF32(volts);
  }
  writer.putF32(temperatureC);
  uint8_t type = streamConfig.format == FORMAT_RAW ? MSG_SAMPLE_RAW : MSG_SAMPLE_VOLTS;
  if (!writer.ok() || !sendFrame(type, transmitPayload, writer.size())) {
    fail(0, ERR_OVERFLOW);
    return;
  }
  streamSequence++;
  deliveredOutputs++;
  if (streamConfig.mode == MODE_FINITE && deliveredOutputs >= streamConfig.outputCount) {
    sendStreamStopped(0, STOP_FINITE_COMPLETE);
  }
}

void processConversion(int32_t raw, uint64_t timestampUs) {
  uint8_t gainIndex = streamConfig.gainIndex;
  uint8_t status = 0;
  if (raw == ADC_POSITIVE_FULL_SCALE) status |= STATUS_ADC_POSITIVE_CLIP;
  if (raw == ADC_NEGATIVE_FULL_SCALE) status |= STATUS_ADC_NEGATIVE_CLIP;
  if (gainIndex == 0 && raw >= TIA_POSITIVE_CLIP_CODE_GAIN_1) {
    status |= STATUS_TIA_POSITIVE_CLIP;
  }

  int32_t magnitude = raw < 0 ? -raw : raw;
  uint8_t nextGain = gainIndex;
  bool adcClipped = raw == ADC_POSITIVE_FULL_SCALE || raw == ADC_NEGATIVE_FULL_SCALE;
  if (streamConfig.autogain) {
    if (magnitude > AUTOGAIN_HIGH_CODE) {
      if (gainIndex > 0) nextGain--;
      else status |= STATUS_AUTOGAIN_OVERRANGE;
    } else if (magnitude < AUTOGAIN_LOW_CODE) {
      if (gainIndex < 7) nextGain++;
      else status |= STATUS_AUTOGAIN_UNDERRANGE;
    }
  }

  bool suppressClippedTransition = streamConfig.autogain && adcClipped && nextGain != gainIndex;
  if (!suppressClippedTransition) {
    int32_t value = raw;
    if (streamConfig.format == FORMAT_VOLTS) value = raw * (1 << (7 - gainIndex));
    addWindowValue(value);
    if (windowReady()) sendSample(raw, timestampUs, gainIndex, status);
  }
  if (deviceState != STATE_STREAMING) return;

  if (nextGain != gainIndex) {
    streamConfig.gainIndex = nextGain;
    if (!configureAdc(streamConfig.profileId, nextGain)) {
      fail(0, ERR_ADC_CONFIG);
    }
  }
}

void processStreaming() {
  uint32_t events;
  uint64_t timestampUs;
  if (takeDrdyEvents(events, timestampUs)) {
    if (events != 1) {
      fail(0, ERR_OVERFLOW, static_cast<uint16_t>(min(events, 65'535u)));
      return;
    }
    int32_t raw = adsReadRaw();
    lastConversionActivityUs = timestampUs;
    processConversion(raw, timestampUs);
    if (deviceState != STATE_STREAMING) return;
  }

  uint64_t nowUs = time_us_64();
  if (nowUs - lastConversionActivityUs > ADC_TIMEOUT_US) {
    fail(0, ERR_ADC_TIMEOUT);
    return;
  }
  if (nowUs >= nextTemperatureUs) {
    if (!refreshTemperature()) {
      fail(0, ERR_TEMPERATURE);
      return;
    }
    nextTemperatureUs += TEMPERATURE_PERIOD_US;
    if (nextTemperatureUs <= nowUs) nextTemperatureUs = nowUs + TEMPERATURE_PERIOD_US;
  }
}

void processRequest(uint8_t messageType, const uint8_t *payload, size_t length, uint64_t receivedUs) {
  BufferReader reader(payload, length);
  uint32_t requestId = reader.getU32();
  if (!reader.ok() || requestId == 0) {
    fail(0, ERR_BAD_ARGUMENT);
    return;
  }

  switch (messageType) {
    case MSG_HELLO:
      if (!reader.finished()) fail(requestId, ERR_BAD_SCHEMA);
      else sendHello(requestId);
      break;

    case MSG_TIME_SYNC: {
      uint64_t utcUs = reader.getU64();
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
        break;
      }
      if (deviceState != STATE_STOPPED) sendStreamStopped(0, STOP_REPLACED);
      synchronizedUtcUs = utcUs;
      synchronizedMonotonicUs = receivedUs;
      timeSynchronized = true;
      sendTimeSynced(requestId, utcUs, receivedUs);
      break;
    }

    case MSG_PING:
      if (!reader.finished()) fail(requestId, ERR_BAD_SCHEMA);
      else {
        BufferWriter writer(transmitPayload, sizeof(transmitPayload));
        writer.putU32(requestId);
        sendFrame(MSG_PONG, transmitPayload, writer.size());
      }
      break;

    case MSG_GET_STATUS:
      if (!reader.finished()) fail(requestId, ERR_BAD_SCHEMA);
      else sendStatus(requestId);
      break;

    case MSG_LIST_PROFILES:
      if (!reader.finished()) fail(requestId, ERR_BAD_SCHEMA);
      else sendProfiles(requestId);
      break;

    case MSG_START_STREAM: {
      StreamConfig config = {};
      config.format = reader.getU8();
      config.mode = reader.getU8();
      config.profileId = reader.getU8();
      config.gainIndex = reader.getU8();
      config.autogain = reader.getU8();
      config.window = reader.getU16();
      config.outputCount = reader.getU32();
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
      } else if (!validStreamConfig(config)) {
        fail(requestId, ERR_BAD_ARGUMENT);
      } else {
        restartActiveStream(requestId, config);
      }
      break;
    }

    case MSG_ACK_STREAM: {
      uint64_t token = reader.getU64();
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
      } else if (deviceState != STATE_AWAITING_ACK || token != streamStartDeviceUs) {
        fail(requestId, ERR_BAD_STATE);
      } else {
        clearDrdyEvents();
        adsCommand(ADS_CMD_START_SYNC);
        lastConversionActivityUs = time_us_64();
        if (!sendOk(requestId, MSG_ACK_STREAM)) {
          fail(0, ERR_OVERFLOW);
        } else {
          deviceState = STATE_STREAMING;
        }
      }
      break;
    }

    case MSG_STOP_STREAM:
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
      } else if (deviceState == STATE_STOPPED) {
        fail(requestId, ERR_BAD_STATE);
      } else {
        sendStreamStopped(requestId, STOP_REQUESTED);
      }
      break;

    case MSG_SET_SESSION_DARK: {
      float value = reader.getF32();
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
      } else if (!std::isfinite(value) || fabsf(value) > DARK_LIMIT_V) {
        fail(requestId, ERR_BAD_ARGUMENT);
      } else {
        bool restart = deviceState != STATE_STOPPED;
        StreamConfig config = streamConfig;
        sessionDarkVolts = value;
        sessionDarkActive = true;
        if (restart) restartActiveStream(requestId, config);
        else sendOk(requestId, MSG_SET_SESSION_DARK);
      }
      break;
    }

    case MSG_CLEAR_SESSION_DARK: {
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
      } else {
        bool restart = deviceState != STATE_STOPPED;
        StreamConfig config = streamConfig;
        sessionDarkActive = false;
        sessionDarkVolts = 0.0f;
        if (restart) restartActiveStream(requestId, config);
        else sendOk(requestId, MSG_CLEAR_SESSION_DARK);
      }
      break;
    }

    case MSG_SAVE_SESSION_DARK: {
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
        break;
      }
      if (!sessionDarkActive) {
        fail(requestId, ERR_BAD_STATE);
        break;
      }
      bool restart = deviceState != STATE_STOPPED;
      StreamConfig config = streamConfig;
      if (restart) sendStreamStopped(0, STOP_REPLACED);
      float value = sessionDarkVolts;
      if (!writeDarkFile(value)) {
        fail(requestId, ERR_STORAGE_WRITE);
      } else {
        sessionDarkActive = false;
        sessionDarkVolts = 0.0f;
        if (restart) beginStream(requestId, config);
        else sendOk(requestId, MSG_SAVE_SESSION_DARK);
      }
      break;
    }

    case MSG_RESET_STORAGE: {
      uint64_t requestedDevice = reader.getU64();
      bool confirmation = reader.matches("ERASE");
      if (!reader.finished()) {
        fail(requestId, ERR_BAD_SCHEMA);
      } else if (requestedDevice != deviceId || !confirmation) {
        fail(requestId, ERR_STORAGE_CONFIRMATION);
      } else {
        if (deviceState != STATE_STOPPED) sendStreamStopped(0, STOP_REPLACED);
        if (!resetStorage()) fail(requestId, ERR_STORAGE_WRITE);
        else sendOk(requestId, MSG_RESET_STORAGE);
      }
      break;
    }

    default:
      fail(requestId, ERR_UNKNOWN_TYPE, messageType);
      break;
  }
}

void processEncodedFrame(uint64_t receivedUs) {
  size_t decodedLength = cobsDecode(
    receiveEncoded, receiveEncodedLength, receiveDecoded, sizeof(receiveDecoded)
  );
  if (decodedLength < 8) {
    fail(0, ERR_BAD_FRAME);
    return;
  }
  uint8_t version = receiveDecoded[0];
  uint8_t messageType = receiveDecoded[1];
  uint16_t payloadLength = static_cast<uint16_t>(receiveDecoded[2])
    | static_cast<uint16_t>(receiveDecoded[3]) << 8;
  if (version != PROTOCOL_VERSION) {
    fail(0, ERR_BAD_VERSION, version);
    return;
  }
  if (payloadLength > MAX_PAYLOAD || decodedLength != payloadLength + 8) {
    fail(0, ERR_BAD_SCHEMA);
    return;
  }
  uint32_t expectedCrc = static_cast<uint32_t>(receiveDecoded[decodedLength - 4])
    | static_cast<uint32_t>(receiveDecoded[decodedLength - 3]) << 8
    | static_cast<uint32_t>(receiveDecoded[decodedLength - 2]) << 16
    | static_cast<uint32_t>(receiveDecoded[decodedLength - 1]) << 24;
  if (crc32(receiveDecoded, decodedLength - 4) != expectedCrc) {
    fail(0, ERR_BAD_CRC);
    return;
  }
  processRequest(messageType, receiveDecoded + 4, payloadLength, receivedUs);
}

void pollSerial() {
  while (Serial.available() > 0) {
    uint8_t byte = static_cast<uint8_t>(Serial.read());
    if (byte == 0) {
      uint64_t receivedUs = time_us_64();
      if (receiveOverflow) {
        fail(0, ERR_BAD_FRAME);
      } else if (receiveEncodedLength > 0) {
        processEncodedFrame(receivedUs);
      }
      receiveEncodedLength = 0;
      receiveOverflow = false;
    } else if (!receiveOverflow) {
      if (receiveEncodedLength < sizeof(receiveEncoded)) {
        receiveEncoded[receiveEncodedLength++] = byte;
      } else {
        receiveOverflow = true;
      }
    }
  }
}

void captureDeviceIdentity() {
  flash_get_unique_id(flashUniqueId);
  deviceId = 0;
  for (uint8_t byte : flashUniqueId) deviceId = (deviceId << 8) | byte;
  for (size_t i = 0; i < sizeof(flashUniqueId); i++) {
    snprintf(usbSerial + i * 2, sizeof(usbSerial) - i * 2, "%02X", flashUniqueId[i]);
  }
}

void configureUsbIdentity() {
  USB.disconnect();
  USB.setManufacturer("LightSensor");
  USB.setProduct("LightSensor v3");
  USB.setSerialNumber(usbSerial);
  USB.connect();
}

}  // namespace

void setup() {
  captureDeviceIdentity();
  configureUsbIdentity();

  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);
  pinMode(ADC_DRDY, INPUT);
  SPI.setRX(ADC_MISO);
  SPI.setCS(ADC_CS);
  SPI.setSCK(ADC_SCK);
  SPI.setTX(ADC_MOSI);
  SPI.begin();
  attachInterrupt(digitalPinToInterrupt(ADC_DRDY), onAdcDrdy, FALLING);
  adcReady = initializeAdc();

  Wire1.setSDA(TMP_SDA);
  Wire1.setSCL(TMP_SCL);
  Wire1.begin();
  Wire1.setClock(400'000);
  temperatureValid = configureTemperature() && refreshTemperature();

  FSConfig filesystemConfig;
  filesystemConfig.setAutoFormat(false);
  LittleFS.setConfig(filesystemConfig);
  filesystemMounted = LittleFS.begin();
  loadPersistence();

  Serial.begin(115200);
}

void loop() {
  pollSerial();
  if (deviceState == STATE_AWAITING_ACK) {
    if (static_cast<int32_t>(millis() - streamAckDeadlineMs) >= 0) {
      fail(0, ERR_ACK_TIMEOUT);
    }
  } else if (deviceState == STATE_STREAMING) {
    processStreaming();
  }
}
