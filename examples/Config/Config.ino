#include <BL0942.h>

#define BL0942_RX 19  
#define BL0942_TX 18  

bl0942::BL0942 blSensor(Serial1);


void dataReceivedCallback(bl0942::SensorData &data) {
  Serial.printf("U: %.2f V, I: %.2f A, P: %.2f W\n", data.voltage, data.current, data.watt);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(4800, SERIAL_8N1, BL0942_RX, BL0942_TX);

  bl0942::ModeConfig config;

  // RMS refresh time (choose one)
  config.rms_update_freq = bl0942::UPDATE_FREQUENCY_400MS;  // or UPDATE_FREQUENCY_800MS

  // RMS waveform type
  config.rms_waveform = bl0942::RMS_WAVEFORM_FULL;          // or RMS_WAVEFORM_AC

  // Line frequency
  config.ac_freq = bl0942::LINE_FREQUENCY_50HZ;             // or LINE_FREQUENCY_60HZ

  // Clear energy counter on read
  config.clear_mode = bl0942::CNT_CLR_SEL_ENABLE;           // or CNT_CLR_SEL_DISABLE

  // Accumulation mode
  config.accumulation_mode = bl0942::ACCUMULATION_MODE_ABSOLUTE;  // or ALGEBRAIC

  // UART baud rate (MUST match Serial1.begin)
  config.uart_rate = bl0942::UART_RATE_4800;                // options: 4800, 9600, 19200, 38400


  blSensor.setup(config);
  blSensor.onDataReceived(dataReceivedCallback);
}

void loop() {
  blSensor.update();
  blSensor.loop(); 
  delay(3000);
}
