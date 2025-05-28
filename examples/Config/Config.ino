#include <BL0942.h>

#define BL0942_RX 16
#define BL0942_TX 17

bl0942::BL0942 blSensor(Serial1);

void dataReceivedCallback(bl0942::SensorData &data) {
  Serial.printf("U: %.2f V, I: %.2f A, P: %.2f W\n", data.voltage, data.current, data.watt);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(19200, SERIAL_8N1, BL0942_RX, BL0942_TX);

  bl0942::ModeConfig config;
  config.uart_rate = bl0942::UART_RATE_19200;
  config.ac_freq = bl0942::LINE_FREQUENCY_60HZ;
  config.clear_mode = bl0942::CNT_CLR_SEL_ENABLE;

  blSensor.setup(config);
  blSensor.onDataReceived(dataReceivedCallback);
  blSensor.update();
}

void loop() {
  blSensor.loop();
}
