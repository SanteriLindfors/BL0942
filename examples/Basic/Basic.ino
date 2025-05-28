#include <BL0942.h>

#define BL0942_RX 19  
#define BL0942_TX 18  

bl0942::BL0942 blSensor(Serial1);

void dataReceivedCallback(bl0942::SensorData &data) {
  Serial.print("Voltage: ");
  Serial.println(data.voltage);
  Serial.print("Current: ");
  Serial.println(data.current);
  Serial.print("Power: ");
  Serial.println(data.watt);
  Serial.print("Energy: ");
  Serial.println(data.energy);
  Serial.print("Frequency: ");
  Serial.println(data.frequency);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(4800, SERIAL_8N1, BL0942_RX, BL0942_TX); // Must be called by user

  blSensor.setup();  // Use default ModeConfig
  blSensor.onDataReceived(dataReceivedCallback);
}

void loop() {
  blSensor.update();
  blSensor.loop(); 
  delay(3000);
}