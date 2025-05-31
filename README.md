# BL0942 Library

This library allows you to interface with the **BL0942 single-phase energy metering chip** via UART. It provides access to voltage, current, power, energy, and frequency readings.

## Wiring

Connect your BL0942 **TX** **RX** **GND** **VCC**.

## Usage

1. Initialize:
   ```cpp
    bl0942::BL0942 blSensor(Serial1);
    ```
2. Call `Serial1.begin()` yourself with desired pins and baud rate:
   ```cpp
    Serial1.begin(4800, SERIAL_8N1, BL0942_RX, BL0942_TX);
    ```
3. Initialize and configure the sensor::
    ```cpp
    blSensor.setup();  // or setup(custom_config)
    blSensor.onDataReceived([](bl0942::SensorData &data) {
        Serial.printf("Power: %.2f", data.watt);
    });
    ```
4. Call `blSensor.loop()` regularly to receive and `blSensor.update()` to request an update.
   ```cpp
   blSensor.update();
   blSensor.loop();
   ```

## API 

#### `BL0942(HardwareSerial &serial, uint8_t address = 0)`
Constructs a new BL0942 driver instance.
- `serial`: A reference to a HardwareSerial port (e.g. `Serial1`)
- `address`: Optional UART address if your module uses addressable communication (default: `0`)

> Note: You must call Serial1.begin() yourself before using this library.



####  `void setup(const ModeConfig &config = ModeConfig{})`

Configures the BL0942 sensor by writing a configuration packet to its internal registers. This setup includes frequency, waveform selection, UART speed, and data clearing behavior.

```cpp
bl0942::ModeConfig cfg;
cfg.uart_rate = bl0942::UART_RATE_19200;
cfg.ac_freq = bl0942::LINE_FREQUENCY_60HZ;
blSensor.setup(cfg);
```

#### Parameters (`ModeConfig`)

Each field in `ModeConfig` affects a corresponding bit in the sensor’s MODE register:

```cpp
struct ModeConfig {
  UpdateFrequency   rms_update_freq     = UPDATE_FREQUENCY_400MS;
  RmsWaveform       rms_waveform        = RMS_WAVEFORM_FULL;
  LineFrequency     ac_freq             = LINE_FREQUENCY_50HZ;
  ClearMode         clear_mode          = CNT_CLR_SEL_DISABLE;
  AccumulationMode  accumulation_mode   = ACCUMULATION_MODE_ABSOLUTE;
  UartRate          uart_rate           = UART_RATE_4800;
};
```

| Field               | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| `rms_update_freq`   | RMS update frequency (400ms or 800ms). Affects how often RMS is refreshed.  |
| `rms_waveform`      | Choose full or AC-only waveform for RMS measurement.                        |
| `ac_freq`           | Sets expected AC line frequency (50Hz or 60Hz).                             |
| `clear_mode`        | Whether the energy counter (`cf_cnt`) clears after each read.               |
| `accumulation_mode` | Use algebraic (signed) or absolute energy accumulation.                     |
| `uart_rate`         | UART baud rate (4800, 9600, 19200, 38400). Must match your `Serial.begin()`. |

##### Energy Reading Mode

BL0942 supports two energy counting modes controlled by the `clear_mode` setting in `ModeConfig`:

| Mode                    | Description                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| `CNT_CLR_SEL_ENABLE`    | The energy counter is **cleared after each read**. Sensor returns **ΔE** (delta energy) since last read. |
| `CNT_CLR_SEL_DISABLE`   | The energy counter is **never cleared**, and value accumulates forever. Sensor returns **total ∫P** (integrated total energy).  |

You can choose the behavior in your `ModeConfig`:

```cpp
bl0942::ModeConfig cfg;
cfg.clear_mode = bl0942::CNT_CLR_SEL_ENABLE;  // For delta energy
// or
cfg.clear_mode = bl0942::CNT_CLR_SEL_DISABLE; // For total energy

blSensor.setup(cfg);
```


##### `void reset()`
Performs a soft reset of the BL0942 sensor, restoring internal configuration to factory defaults.

##### `void onDataReceived(OnDataReceivedCallback callback)`
Registers a callback function that will be triggered when new sensor data is received and successfully parsed.

The `SensorData` struct is passed to your callback whenever new data is received from the BL0942 sensor. It contains the most recent measurement values:

```cpp
struct SensorData {
  float voltage;   // Voltage RMS in volts
  float current;   // Current RMS in amperes
  float watt;      // Active power in watts
  float energy;    // Energy in kilowatt-hours (kWh)
  float frequency; // Line frequency in Hz
};
```
The value of `SensorData.energy` depends on the `clear_mode` configured in `ModeConfig` during `setup()`:

| `clear_mode` Setting     | `SensorData.energy` Value                                      |
|--------------------------|---------------------------------------------------------------|
| `CNT_CLR_SEL_ENABLE`     | **ΔE** — Energy (kWh) accumulated since the last update       |
| `CNT_CLR_SEL_DISABLE`    | **∫P** — Total accumulated energy (kWh) since boot            |

This means:

- If you're using `CNT_CLR_SEL_ENABLE`, you get the **energy accumulated between each update request**.
- If using `CNT_CLR_SEL_DISABLE`, the value will **accumulate indefinitely** (or until the device resets).



```cpp
blSensor.onDataReceived([](bl0942::SensorData &data) {
  Serial.printf("U: %.2f V, I: %.2f A, P: %.2f W\\n", data.voltage, data.current, data.watt);
});
```
##### `void update()`
Manually sends a request to the BL0942 chip to send the latest measurement data. This triggers a response packet that must be handled in `loop()`.

##### `bool loop()`
Call this in your `loop()` function continuously. It checks the UART buffer, reads incoming packets, validates the checksum, and triggers the registered callback if data is received.

##### `void print_registers()`
Reads and prints all key registers from the BL0942 for debugging and diagnostics. This includes mode configuration, gain, and sensor status values.
## Example

```cpp
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
```