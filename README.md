# Height Measurement via Barometer using ESP32

## Overview

This project demonstrates real-time height measurement using barometric pressure sensors (BMP280) with ESP32 microcontrollers. The system consists of two nodes communicating via ESP-NOW protocol:

- **Master Node**: Measures absolute atmospheric pressure and broadcasts it to the slave node
- **Slave Node**: Measures local pressure, calculates relative height difference, and provides drift compensation with tare functionality

The system is designed for applications requiring precise height measurements such as:
- Industrial automation height monitoring
- Sports performance tracking
- Building elevation mapping
- UAV altitude reference systems

## Hardware Requirements

- **2x ESP32 development boards** (any variant with Wi-Fi capability)
- **2x BMP280 barometric pressure sensors** (I²C interface)
- **Jumper wires** for connections
- **Breadboard** (optional for prototyping)

### Wiring Diagram (Both Nodes)

| ESP32 Pin | BMP280 Pin |
|-----------|------------|
| 3.3V      | VCC        |
| GND       | GND        |
| GPIO21 (SDA) | SDA     |
| GPIO22 (SCL) | SCL     |

## Software Dependencies

- Arduino IDE (with ESP32 board support)
- Libraries to install via Library Manager:
  - `ESP32` board package
  - `Adafruit BMP280 Library`
  - `Adafruit Unified Sensor`
- ESP-NOW protocol (built-in ESP32 functionality)

## Project Structure

```
Height-Measurement-via-Barometer-using-ESP32/
├── master_transmitter/
│   └── master_transmitter.ino  # Master node code (transmits reference pressure)
├── slave_receiver/
│   └── slave_receiver.ino      # Slave node code (calculates height)
├── README.md
└── .gitignore
```

## System Architecture

The system uses a master-slave architecture where:
1. **Master Node**: Continuously measures and broadcasts absolute atmospheric pressure
2. **Slave Node**: 
   - Receives master pressure readings
   - Performs burst sampling of local pressure
   - Calculates relative height using the barometric formula
   - Applies drift filtering and tare offset compensation
   - Outputs height readings via serial monitor

## Key Features

### 🎯 Drift Filtering Implementation

Barometric altitude measurements are quite accurate, as long as you keep track of the local atmospheric pressure changes.  The system implements multiple drift mitigation techniques:

1. **Burst Sampling**: Takes 300 consecutive readings and averages them to reduce noise and transient errors
2. **Hardware Filtering**: BMP280's internal IIR filter (configurable via `FILTER_X4` parameter)
3. **Reference Synchronization**: Uses master node pressure as a moving reference to compensate for weather-induced pressure changes
4. **Tare Calibration**: User-triggered zero-height calibration to eliminate cumulative drift

### ⚙️ Configurable Parameters

#### In Slave Receiver Code:
```cpp
const int BURST_SIZE = 300;    // Number of readings to average per measurement frame
float heightOffset = 0;        // Tare offset (automatically calculated)
```

#### In BMP280 Configuration (Both Nodes):
```cpp
bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,      // Operating mode
    Adafruit_BMP280::SAMPLING_X2,      // Pressure oversampling (master: X16, slave: X2)
    Adafruit_BMP280::SAMPLING_X4,      // Temperature oversampling
    Adafruit_BMP280::FILTER_X4,        // IIR filter coefficient (key for drift reduction)
    Adafruit_BMP280::STANDBY_MS_1      // Standby time between measurements
);
```

### 📏 Height Calculation Formula

The system uses the international barometric formula for height calculation:

```
height = 44330.0 * (1.0 - pow(local_pressure / reference_pressure, 0.1903))
```

Where:
- `local_pressure` = Pressure measured at slave node (hPa)
- `reference_pressure` = Pressure measured at master node (hPa)
- Result is in meters

### 🎚️ Tare Functionality

The slave node supports runtime tare calibration:
- **Trigger**: Send 't' or 'z' character via serial monitor
- **Process**: Takes 300 new readings to calculate current height offset
- **Effect**: All subsequent readings are relative to the tared position
- **Use Case**: Essential for setting zero reference when mounting on moving platforms or after environmental changes

## Setup Instructions

### 1. Hardware Setup
1. Connect BMP280 sensors to both ESP32 boards following the wiring diagram
2. Power both boards (USB or external power source)

### 2. Software Configuration
#### Master Node Setup:
1. Open `master_transmitter/master_transmitter.ino`
2. **Critical**: Replace the slave MAC address with your actual slave board's MAC:
```cpp
uint8_t slaveAddress[] = {0x10, 0x51, 0xDB, 0x85, 0xC2, 0xD4}; // REPLACE WITH YOUR SLAVE'S MAC
```
3. Upload to master ESP32 board

#### Slave Node Setup:
1. Open `slave_receiver/slave_receiver.ino`
2. Upload to slave ESP32 board
3. Open serial monitor (115200 baud) to see height readings

### 3. Calibration Procedure
1. **Initial Setup**: Place both nodes at the same height initially
2. **Power On**: Start master node first, then slave node
3. **Tare Calibration**: 
   - Wait for "System Ready. Type 't' to Tare." message
   - Press 't' in serial monitor to set current position as zero height
4. **Operation**: Move slave node to desired positions and observe height changes

## Performance Characteristics

### ✅ Advantages
- **Real-time measurements** with ~100ms update rate
- **Wireless operation** via ESP-NOW (no Wi-Fi network required)
- **High accuracy** with burst sampling and filtering
- **Automatic weather compensation** through master reference

### ⚠️ Limitations & Known Issues
- **Drift**: The forward-only filtering approach introduces drift over time and slow response to rapid height changes
- **Temperature sensitivity**: BMP280 measurements are affected by temperature changes
- **Range limitation**: ESP-NOW effective range ~100m in open space
- **Power consumption**: Continuous high-speed sampling drains batteries quickly

## Advanced Configuration Options

### 1. Filter Tuning
Modify the BMP280 filter coefficient for different drift/noise trade-offs:
```cpp
// Options: FILTER_OFF, FILTER_X2, FILTER_X4, FILTER_X8, FILTER_X16
Adafruit_BMP280::FILTER_X4  // Higher values = more filtering, slower response
```

### 2. Sampling Rate Adjustment
Balance accuracy vs. responsiveness:
```cpp
const int BURST_SIZE = 300;  // Higher = better accuracy, slower response
// Try: 50 (fast response), 100 (balanced), 500 (high accuracy)

// In loop() function:
delay(100);  // Output rate control - adjust for desired update frequency
```

### 3. Oversampling Configuration
```cpp
// Master node (reference accuracy critical):
Adafruit_BMP280::SAMPLING_X16  // Maximum pressure oversampling

// Slave node (speed vs accuracy trade-off):
Adafruit_BMP280::SAMPLING_X2   // Lower for faster response
```

## Troubleshooting

### Common Issues & Solutions:

| Issue | Solution |
|-------|----------|
| "BMP fail" on startup | Check wiring, ensure correct I²C address (0x76 or 0x77) |
| "Waiting for Master Node Sync..." | Verify master is running, check MAC address configuration |
| No serial output | Ensure baud rate is set to 115200, check USB connection |
| Erratic height readings | Increase BURST_SIZE, check for air currents near sensor |
| Large drift over time | Perform tare calibration more frequently, ensure stable temperature |

### Diagnostic Commands:
- **'t' or 'z'**: Perform tare calibration (sets current height as zero)
- **Monitor serial output**: Watch for sync messages and error indications

## Future Improvements

1. **Advanced Filtering**: Implement Kalman filter or complementary filter for better drift reduction
2. **Temperature Compensation**: Add active temperature stabilization or compensation algorithms
3. **Multi-point Calibration**: Support multiple reference points for non-linear environments
4. **Battery Optimization**: Add sleep modes and wake-on-change functionality
5. **Data Logging**: SD card or cloud storage for height measurement history
6. **Visual Display**: OLED or LCD display for standalone operation

## Contributing

Contributions are welcome! Please feel free to submit pull requests for:
- Improved filtering algorithms
- Additional sensor support (BME280, BMP388, etc.)
- Better calibration procedures
- Power optimization improvements
- Documentation enhancements

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Adafruit for the excellent BMP280 library and sensor breakout boards
- Espressif for the ESP32 platform and ESP-NOW protocol implementation
- The open-source community for continuous improvements to ESP32 ecosystem

---

**Note**: This system can measure altitude by combining air pressure and temperature data through the BMP280 sensor.  For best results, keep both nodes in stable environmental conditions and perform regular tare calibrations when absolute accuracy is critical.
