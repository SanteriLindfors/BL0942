#include "bl0942.h"
#include <cinttypes>

#define DEBUG 1

#if DEBUG

// ESP32 platform
#if defined(ESP32)
#include <esp_log.h>
#define BL0942_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define BL0942_LOGD(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#define BL0942_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define BL0942_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)

// Non-ESP32 (e.g., AVR, STM32)
#else
#include <cstdio> // for snprintf

#define BL0942_LOGI(tag, fmt, ...)                                             \
  do {                                                                         \
    char buf[128];                                                             \
    snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__);                            \
    Serial.print("[INFO] ");                                                   \
    Serial.println(buf);                                                       \
  } while (0)

#define BL0942_LOGW(tag, fmt, ...)                                             \
  do {                                                                         \
    char buf[128];                                                             \
    snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__);                            \
    Serial.print("[WARN] ");                                                   \
    Serial.println(buf);                                                       \
  } while (0)

#define BL0942_LOGE(tag, fmt, ...)                                             \
  do {                                                                         \
    char buf[128];                                                             \
    snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__);                            \
    Serial.print("[ERROR] ");                                                  \
    Serial.println(buf);                                                       \
  } while (0)

// DEBUG logs are disabled for non-ESP32
#define BL0942_LOGD(tag, fmt, ...)                                             \
  do {                                                                         \
  } while (0)

#endif

#else
// DEBUG == 0, disable all logging
#define BL0942_LOGI(tag, fmt, ...)
#define BL0942_LOGD(tag, fmt, ...)
#define BL0942_LOGW(tag, fmt, ...)
#define BL0942_LOGE(tag, fmt, ...)
#endif

// Datasheet:
// https://www.belling.com.cn/media/file_object/bel_product/BL0942/datasheet/BL0942_V1.06_en.pdf

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

BL0942::BL0942(HardwareSerial &serial, uint8_t address)
    : serial_(serial), address_(address_) {}

void BL0942::setup(const ModeConfig &config) {
  BL0942_LOGI(TAG, "Initializing BL0942 sensor...");

  write_reg_(BL0942_REG_USR_WRPROT, BL0942_REG_USR_WRPROT_MAGIC);

  uint32_t mode = BL0942_REG_MODE_DEFAULT;
  mode |= config.rms_update_freq;
  mode |= config.rms_waveform;
  mode |= config.ac_freq;
  mode |= config.clear_mode;
  mode |= config.accumulation_mode;
  mode |= (config.uart_rate << 8);

  write_reg_(BL0942_REG_MODE, mode);

  if (read_reg_(BL0942_REG_MODE) != mode) {
    BL0942_LOGE(TAG, "BL0942 setup failed!");
  } else {
    BL0942_LOGI(TAG, "BL0942 sensor initialized.");
  }

  write_reg_(BL0942_REG_USR_WRPROT, 0);
}

void BL0942::reset() {
  BL0942_LOGI(TAG, "Resetting BL0942 sensor...");

  write_reg_(BL0942_REG_USR_WRPROT, BL0942_REG_USR_WRPROT_MAGIC);
  write_reg_(BL0942_REG_SOFT_RESET, BL0942_REG_SOFT_RESET_MAGIC);
}

bool BL0942::loop() {
  DataPacket buffer;
  int avail = serial_.available();

  if (avail < sizeof(buffer)) {
    return false;
  }

  if (serial_.readBytes(reinterpret_cast<uint8_t *>(&buffer), sizeof(buffer)) ==
      sizeof(buffer)) {
    BL0942_LOGD(TAG, "Received data packet, validating checksum...");
    if (validate_checksum_(&buffer)) {
      BL0942_LOGD(TAG, "Checksum valid, processing data...");
      received_package_(&buffer);
      return true;
    } else {
      BL0942_LOGW(TAG, "Checksum invalid, ignoring packet.");
    }
  } else {
    BL0942_LOGW(TAG, "Failed to read the full data packet.");
  }
  return false;
}

bool BL0942::validate_checksum_(DataPacket *data) {
  uint8_t checksum = BL0942_READ_COMMAND | this->address_;
  uint8_t *raw = reinterpret_cast<uint8_t *>(data);
  for (size_t i = 0; i < sizeof(*data) - 1; i++) {
    checksum += raw[i];
  }
  checksum ^= 0xFF;

  if (checksum != data->checksum) {
    BL0942_LOGW(TAG, "Invalid checksum! Expected: 0x%02X, Got: 0x%02X",
                checksum, data->checksum);
    return false;
  }
  return checksum == data->checksum;
}

void BL0942::received_package_(DataPacket *data) {
  if (data->frame_header != BL0942_PACKET_HEADER) {
    BL0942_LOGW(TAG,
                "Invalid data. Header mismatch. Expected: 0x%02X, Got: 0x%02X",
                BL0942_PACKET_HEADER, data->frame_header);
    return;
  }

  uint32_t cf_cnt = data->cf_cnt;
  cf_cnt |= this->prev_cf_cnt_ & 0xff000000;
  if (cf_cnt < this->prev_cf_cnt_) {
    cf_cnt += 0x1000000;
  }
  this->prev_cf_cnt_ = cf_cnt;

  SensorData sensorData;
  sensorData.voltage = data->v_rms / BL0942_UREF;
  sensorData.current = data->i_rms / BL0942_IREF;
  sensorData.watt = data->watt / BL0942_PREF;
  sensorData.energy = cf_cnt / BL0942_EREF;
  sensorData.frequency = 1000000.0f / data->frequency;

  BL0942_LOGI(TAG,
              "BL0942: U %fV, I %fA, P %fW, Cnt %" PRId32
              ", ∫P %fkWh, frequency %fHz, status 0x%08X",
              sensorData.voltage, sensorData.current, sensorData.watt, cf_cnt,
              sensorData.energy, sensorData.frequency, data->status);

  if (dataCallback) {
    dataCallback(sensorData);
  }
}

void BL0942::onDataReceived(OnDataReceivedCallback callback) {
  dataCallback = callback;
}

void BL0942::update() {
  serial_.write(BL0942_READ_COMMAND | this->address_);
  serial_.write(BL0942_FULL_PACKET);
  serial_.flush();
}

void BL0942::write_reg_(uint8_t reg, uint32_t val) {
  uint8_t pkt[6];

  pkt[0] = BL0942_WRITE_COMMAND | this->address_;
  pkt[1] = reg;
  pkt[2] = (val & 0xff);
  pkt[3] = (val >> 8) & 0xff;
  pkt[4] = (val >> 16) & 0xff;
  pkt[5] = (pkt[0] + pkt[1] + pkt[2] + pkt[3] + pkt[4]) ^ 0xff;

  BL0942_LOGD(TAG, "Writing value 0x%02X to register 0x%02X", val, reg);

  BL0942_LOGD(
      TAG,
      "Packet to be sent: [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]",
      pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]);
  serial_.write(pkt, 6);
  serial_.flush();
}

int BL0942::read_reg_(uint8_t reg) {
  union {
    uint8_t b[4];
    uint32_t le32;
  } resp;

  serial_.write(BL0942_READ_COMMAND | this->address_);
  serial_.write(reg);
  serial_.flush();

  int bytesRead = serial_.readBytes(resp.b, 4);

  if (bytesRead == 4) {
    if (resp.b[3] == (uint8_t)((BL0942_READ_COMMAND + this->address_ + reg +
                                resp.b[0] + resp.b[1] + resp.b[2]) ^
                               0xff)) {
      resp.b[3] = 0;
      return resp.le32;
    } else {
      BL0942_LOGE(TAG, "Checksum invalid");
    }
  } else {
    BL0942_LOGE(TAG, "Failed to read enough bytes");
  }

  return -1;
}

void BL0942::print_registers() {
  // Read I_RMSOS (address 0x12)
  int i_rmsos = read_reg_(BL0942_REG_I_RMSOS);
  if (i_rmsos != -1) {
    BL0942_LOGI(TAG, "I_RMSOS Register Value: 0x%02X", i_rmsos);
  } else {
    BL0942_LOGW(TAG, "Failed to read I_RMSOS register");
  }

  // Read WA_CREEP (address 0x14)
  int wa_creep = read_reg_(BL0942_REG_WA_CREEP);
  if (wa_creep != -1) {
    BL0942_LOGI(TAG, "WA_CREEP Register Value: 0x%02X", wa_creep);
  } else {
    BL0942_LOGW(TAG, "Failed to read WA_CREEP register");
  }

  // Read I_FAST_RMS_TH (address 0x15)
  int i_fast_rms_th = read_reg_(BL0942_REG_I_FAST_RMS_TH);
  if (i_fast_rms_th != -1) {
    BL0942_LOGI(TAG, "I_FAST_RMS_TH Register Value: 0x%02X", i_fast_rms_th);
  } else {
    BL0942_LOGW(TAG, "Failed to read I_FAST_RMS_TH register");
  }

  // Read I_FAST_RMS_CYC (address 0x16)
  int i_fast_rms_cyc = read_reg_(BL0942_REG_I_FAST_RMS_CYC);
  if (i_fast_rms_cyc != -1) {
    BL0942_LOGI(TAG, "I_FAST_RMS_CYC Register Value: 0x%02X", i_fast_rms_cyc);
  } else {
    BL0942_LOGW(TAG, "Failed to read I_FAST_RMS_CYC register");
  }

  // Read FREQ_CYC (address 0x17)
  int freq_cyc = read_reg_(BL0942_REG_FREQ_CYC);
  if (freq_cyc != -1) {
    BL0942_LOGI(TAG, "FREQ_CYC Register Value: 0x%02X", freq_cyc);
  } else {
    BL0942_LOGW(TAG, "Failed to read FREQ_CYC register");
  }

  // Read OT_FUNX (address 0x18)
  int ot_funx = read_reg_(BL0942_REG_OT_FUNX);
  if (ot_funx != -1) {
    BL0942_LOGI(TAG, "OT_FUNX Register Value: 0x%02X", ot_funx);
  } else {
    BL0942_LOGW(TAG, "Failed to read OT_FUNX register");
  }

  // Read MODE (address 0x19)
  int mode = read_reg_(BL0942_REG_MODE);
  if (mode != -1) {
    BL0942_LOGI(TAG, "MODE Register Value: 0x%02X", mode);

    uint8_t cf_en = (mode >> 2) & 0x01;          // Bit 2
    uint8_t rms_update_sel = (mode >> 3) & 0x01; // Bit 3
    uint8_t fast_rms_sel = (mode >> 4) & 0x01;   // Bit 4
    uint8_t ac_freq_sel = (mode >> 5) & 0x01;    // Bit 5
    uint8_t cf_cnt_clr_sel = (mode >> 6) & 0x01; // Bit 6
    uint8_t cf_cnt_add_sel = (mode >> 7) & 0x01; // Bit 7
    uint8_t uart_rate_sel = (mode >> 8) & 0x03;  // Bits 8-9 (2 bits)

    BL0942_LOGI(TAG, "CF_EN: %d (Active energy and pulse output enable)",
                cf_en);
    BL0942_LOGI(TAG, "RMS_UPDATE_SEL: %d (Refresh time for RMS)",
                rms_update_sel);
    BL0942_LOGI(TAG, "FAST_RMS_SEL: %d (Fast RMS waveform)", fast_rms_sel);
    BL0942_LOGI(TAG, "AC_FREQ_SEL: %d (AC frequency selection)", ac_freq_sel);
    BL0942_LOGI(TAG, "CF_CNT_CLR_SEL: %d (Clear after read of CF_CNT)",
                cf_cnt_clr_sel);
    BL0942_LOGI(TAG,
                "CF_CNT_ADD_SEL: %d (Mode selection of active energy pulse "
                "accumulation)",
                cf_cnt_add_sel);
    BL0942_LOGI(TAG,
                "UART_RATE_SEL: %d (Baud rate selection: 0=4800bps, 1=9600bps, "
                "2=19200bps, 3=38400bps)",
                uart_rate_sel);
  } else {
    BL0942_LOGW(TAG, "Failed to read MODE register");
  }

  // Read GAIN_CR (address 0x1A)
  int gain_cr = read_reg_(BL0942_REG_GAIN_CR);
  if (gain_cr != -1) {
    BL0942_LOGI(TAG, "GAIN_CR Register Value: 0x%02X", gain_cr);
  } else {
    BL0942_LOGW(TAG, "Failed to read GAIN_CR register");
  }

  // Read SOFT_RESET (address 0x1C)
  int soft_reset = read_reg_(BL0942_REG_SOFT_RESET);
  if (soft_reset != -1) {
    BL0942_LOGI(TAG, "SOFT_RESET Register Value: 0x%02X", soft_reset);
  } else {
    BL0942_LOGW(TAG, "Failed to read SOFT_RESET register");
  }

  // Read USR_WRPROT (address 0x1D)
  int usr_wrprot = read_reg_(BL0942_REG_USR_WRPROT);
  if (usr_wrprot != -1) {
    BL0942_LOGI(TAG, "USR_WRPROT Register Value: 0x%02X", usr_wrprot);
  } else {
    BL0942_LOGW(TAG, "Failed to read USR_WRPROT register");
  }
}

} // namespace bl0942
