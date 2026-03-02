#include "BL0942.h"
#include <cinttypes>
#include <cmath>

#if BL0942DEBUG

#if defined(ESP32)
#include <esp_log.h>
#define BL0942_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define BL0942_LOGD(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#define BL0942_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define BL0942_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
#else
#include <cstdio>
#define BL0942_LOGI(tag, fmt, ...)                                             \
  do {                                                                         \
    char b[128];                                                               \
    snprintf(b, sizeof(b), fmt, ##__VA_ARGS__);                                \
    Serial.print("[INFO] ");                                                   \
    Serial.println(b);                                                         \
  } while (0)
#define BL0942_LOGW(tag, fmt, ...)                                             \
  do {                                                                         \
    char b[128];                                                               \
    snprintf(b, sizeof(b), fmt, ##__VA_ARGS__);                                \
    Serial.print("[WARN] ");                                                   \
    Serial.println(b);                                                         \
  } while (0)
#define BL0942_LOGE(tag, fmt, ...)                                             \
  do {                                                                         \
    char b[128];                                                               \
    snprintf(b, sizeof(b), fmt, ##__VA_ARGS__);                                \
    Serial.print("[ERROR] ");                                                  \
    Serial.println(b);                                                         \
  } while (0)
#define BL0942_LOGD(tag, fmt, ...)                                             \
  do {                                                                         \
  } while (0)
#endif

#else
#define BL0942_LOGI(tag, fmt, ...)
#define BL0942_LOGD(tag, fmt, ...)
#define BL0942_LOGW(tag, fmt, ...)
#define BL0942_LOGE(tag, fmt, ...)
#endif

namespace bl0942 {

static const char *const TAG = "bl0942";

static const uint8_t BL0942_READ_COMMAND = 0x58;
static const uint8_t BL0942_FULL_PACKET = 0xAA;
static const uint8_t BL0942_PACKET_HEADER = 0x55;
static const uint8_t BL0942_WRITE_COMMAND = 0xA8;

static const uint8_t BL0942_REG_I_RMSOS = 0x12;
static const uint8_t BL0942_REG_WA_CREEP = 0x14;
static const uint8_t BL0942_REG_I_FAST_RMS_TH = 0x15;
static const uint8_t BL0942_REG_I_FAST_RMS_CYC = 0x16;
static const uint8_t BL0942_REG_FREQ_CYC = 0x17;
static const uint8_t BL0942_REG_OT_FUNX = 0x18;
static const uint8_t BL0942_REG_MODE = 0x19;
static const uint8_t BL0942_REG_GAIN_CR = 0x1A;
static const uint8_t BL0942_REG_SOFT_RESET = 0x1C;
static const uint8_t BL0942_REG_USR_WRPROT = 0x1D;

static const uint32_t BL0942_REG_MODE_RESV = 0x03;
static const uint32_t BL0942_REG_MODE_CF_EN = 0x04;
static const uint32_t BL0942_REG_MODE_DEFAULT =
    BL0942_REG_MODE_RESV | BL0942_REG_MODE_CF_EN;

static const uint32_t BL0942_REG_SOFT_RESET_MAGIC = 0x5a5a5a;
static const uint32_t BL0942_REG_USR_WRPROT_MAGIC = 0x55;

// BL0942 "full packet" frame is 23 bytes:
// 0:  0x55
// 1-3:  I_RMS (24-bit LE)
// 4-6:  V_RMS (24-bit LE)
// 7-9:  I_FAST_RMS (24-bit LE)
// 10-12: WATT (24-bit LE, signed)
// 13-15: CF_CNT (24-bit LE)
// 16-17: FREQ (16-bit LE)
// 18: reserved
// 19: status
// 20: reserved
// 21: reserved
// 22: checksum
static constexpr size_t BL0942_FRAME_SIZE = 23;

static uint32_t u24le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static int32_t s24le(const uint8_t *p) {
  uint32_t v = u24le(p);
  if (v & 0x00800000)
    v |= 0xFF000000; // sign-extend 24->32
  return (int32_t)v;
}

static uint16_t u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

BL0942::BL0942(HardwareSerial &serial, uint8_t address)
    : serial_(serial), address_(address) {}

bool BL0942::setup(const ModeConfig &config,
                   const CalibrationConfig &calibration) {
  BL0942_LOGI(TAG, "Initializing BL0942 sensor...");
  calibration_ = calibration;
  use_delta_energy_ = (config.clear_mode == CNT_CLR_SEL_ENABLE);

  write_reg_(BL0942_REG_USR_WRPROT, BL0942_REG_USR_WRPROT_MAGIC);

  uint32_t mode = BL0942_REG_MODE_DEFAULT;
  mode |= config.rms_update_freq;
  mode |= config.rms_waveform;
  mode |= config.ac_freq;
  mode |= config.clear_mode;
  mode |= config.accumulation_mode;
  mode |= (uint32_t(config.uart_rate) << 8);

  write_reg_(BL0942_REG_MODE, mode);

  int readback = read_reg_(BL0942_REG_MODE);
  bool ok = (readback == (int)mode);
  if (!ok) {
    BL0942_LOGE(TAG, "BL0942 setup failed! wrote=0x%08" PRIX32 " read=0x%08X",
                mode, readback);
  } else {
    BL0942_LOGI(TAG, "BL0942 sensor initialized.");
  }

  write_reg_(BL0942_REG_USR_WRPROT, 0);
  return ok;
}

void BL0942::reset() {
  BL0942_LOGI(TAG, "Resetting BL0942 sensor...");
  write_reg_(BL0942_REG_USR_WRPROT, BL0942_REG_USR_WRPROT_MAGIC);
  write_reg_(BL0942_REG_SOFT_RESET, BL0942_REG_SOFT_RESET_MAGIC);
}

// checksum = ((READ|addr) + sum(frame[0..21])) ^ 0xFF
bool BL0942::validate_checksum_(const uint8_t *frame, size_t len) {
  if (!frame || len != BL0942_FRAME_SIZE)
    return false;

  uint8_t checksum = (uint8_t)(BL0942_READ_COMMAND | this->address_);
  for (size_t i = 0; i < len - 1; i++) {
    checksum += frame[i];
  }
  checksum ^= 0xFF;

  const uint8_t got = frame[len - 1];
  if (checksum != got) {
    BL0942_LOGW(TAG, "Invalid checksum! Expected: 0x%02X, Got: 0x%02X",
                checksum, got);
    return false;
  }
  return true;
}

void BL0942::received_package_(const uint8_t *f, size_t len) {
  if (!f || len != BL0942_FRAME_SIZE)
    return;
  if (f[0] != BL0942_PACKET_HEADER) {
    BL0942_LOGW(TAG, "Header mismatch. Expected 0x%02X, got 0x%02X",
                BL0942_PACKET_HEADER, f[0]);
    return;
  }

  const uint32_t i_rms_raw = u24le(&f[1]);
  const uint32_t v_rms_raw = u24le(&f[4]);
  const uint32_t i_fast_raw = u24le(&f[7]);
  (void)i_fast_raw;
  const int32_t watt_raw = s24le(&f[10]);
  uint32_t cf_cnt_raw = u24le(&f[13]);
  const uint16_t freq_raw = u16le(&f[16]);
  const uint8_t status = f[19];

  // Extend 24-bit counter to monotonic 32-bit when in TOTAL mode
  if (!use_delta_energy_) {
    cf_cnt_raw |= (this->prev_cf_cnt_ & 0xFF000000);
    if (cf_cnt_raw < this->prev_cf_cnt_) {
      cf_cnt_raw += 0x01000000; // rollover
    }
    this->prev_cf_cnt_ = cf_cnt_raw;
  }

  SensorData d{};
  d.voltage = (float)v_rms_raw / calibration_.voltage_reference;
  d.current = (float)i_rms_raw / calibration_.current_reference;
  d.watt = (float)watt_raw / calibration_.power_reference;
  d.energy = (float)cf_cnt_raw / calibration_.energy_reference;
  d.frequency = (freq_raw != 0) ? (1000000.0f / (float)freq_raw) : 0.0f;

  BL0942_LOGI(TAG,
              "BL0942: U %.3fV, I %.4fA, P %.3fW, CF %lu, %s %.6fkWh, freq "
              "%.3fHz, status 0x%02X "
              "(raw: i=%lu v=%lu w=%ld cf=%lu f=%u)",
              d.voltage, d.current, d.watt, (unsigned long)cf_cnt_raw,
              (use_delta_energy_ ? "ΔE" : "Total"), d.energy, d.frequency,
              status, (unsigned long)i_rms_raw, (unsigned long)v_rms_raw,
              (long)watt_raw, (unsigned long)cf_cnt_raw, (unsigned)freq_raw);

  if (dataCallback)
    dataCallback(d);
}

bool BL0942::loop() {
  // Resync to header 0x55
  while (serial_.available() > 0) {
    int b = serial_.peek();
    if (b < 0)
      return false;
    if ((uint8_t)b == BL0942_PACKET_HEADER)
      break;
    serial_.read(); // discard garbage until header
  }

  if (serial_.available() < (int)BL0942_FRAME_SIZE)
    return false;

  uint8_t frame[BL0942_FRAME_SIZE];
  size_t n = serial_.readBytes(frame, BL0942_FRAME_SIZE);
  if (n != BL0942_FRAME_SIZE)
    return false;

  if (!validate_checksum_(frame, BL0942_FRAME_SIZE))
    return false;

  received_package_(frame, BL0942_FRAME_SIZE);
  return true;
}

void BL0942::onDataReceived(OnDataReceivedCallback callback) {
  dataCallback = callback;
}

void BL0942::update() {
  serial_.write((uint8_t)(BL0942_READ_COMMAND | this->address_));
  serial_.write(BL0942_FULL_PACKET);
  serial_.flush();
}

void BL0942::write_reg_(uint8_t reg, uint32_t val) {
  uint8_t pkt[6];

  pkt[0] = (uint8_t)(BL0942_WRITE_COMMAND | this->address_);
  pkt[1] = reg;
  pkt[2] = (uint8_t)(val & 0xFF);
  pkt[3] = (uint8_t)((val >> 8) & 0xFF);
  pkt[4] = (uint8_t)((val >> 16) & 0xFF);
  pkt[5] = (uint8_t)((pkt[0] + pkt[1] + pkt[2] + pkt[3] + pkt[4]) ^ 0xFF);

  BL0942_LOGD(TAG, "Write reg 0x%02X = 0x%06" PRIX32, reg, (val & 0x00FFFFFFu));
  serial_.write(pkt, 6);
  serial_.flush();
}

int BL0942::read_reg_(uint8_t reg) {
  uint8_t resp[4]{};

  serial_.write((uint8_t)(BL0942_READ_COMMAND | this->address_));
  serial_.write(reg);
  serial_.flush();

  int bytesRead = serial_.readBytes(resp, 4);
  if (bytesRead != 4) {
    BL0942_LOGE(TAG, "Failed to read reg 0x%02X (read %d bytes)", reg,
                bytesRead);
    return -1;
  }

  const uint8_t expected = (uint8_t)((BL0942_READ_COMMAND | this->address_) +
                                     reg + resp[0] + resp[1] + resp[2]) ^
                           0xFF;
  if (resp[3] != expected) {
    BL0942_LOGE(TAG, "Reg 0x%02X checksum invalid (got 0x%02X exp 0x%02X)", reg,
                resp[3], expected);
    return -1;
  }

  // 24-bit value in resp[0..2], little-endian
  return (int)((uint32_t)resp[0] | ((uint32_t)resp[1] << 8) |
               ((uint32_t)resp[2] << 16));
}

void BL0942::print_registers() {
  auto dump = [&](const char *name, uint8_t reg) {
    int v = read_reg_(reg);
    if (v >= 0)
      BL0942_LOGI(TAG, "%s (0x%02X) = 0x%06X", name, reg, v);
    else
      BL0942_LOGW(TAG, "Failed to read %s (0x%02X)", name, reg);
  };

  dump("I_RMSOS", BL0942_REG_I_RMSOS);
  dump("WA_CREEP", BL0942_REG_WA_CREEP);
  dump("I_FAST_RMS_TH", BL0942_REG_I_FAST_RMS_TH);
  dump("I_FAST_RMS_CYC", BL0942_REG_I_FAST_RMS_CYC);
  dump("FREQ_CYC", BL0942_REG_FREQ_CYC);
  dump("OT_FUNX", BL0942_REG_OT_FUNX);
  dump("MODE", BL0942_REG_MODE);
  dump("GAIN_CR", BL0942_REG_GAIN_CR);
  dump("SOFT_RESET", BL0942_REG_SOFT_RESET);
  dump("USR_WRPROT", BL0942_REG_USR_WRPROT);
}

} // namespace bl0942