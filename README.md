# MEKF Arduino Sensor Logger

This project implements an embedded data logger for the MPU6050 IMU using an Arduino-compatible board. It reads accelerometer and gyroscope data, packages it, and streams it over serial. Additionally, the system provides periodic audible feedback via a buzzer. This project was intended to be used in conjunction with the Multiplicative Extended Kalman Filter repo found at: https://github.com/AhmedKhan04/MEKF_REPO  

---

## Features

* **IMU Integration**: Reads 3-axis accelerometer and gyroscope data from the MPU6050 sensor.
* **Data Packet Structure**: Sends structured packets containing:

  * Timestamp (`t`) in milliseconds
  * Accelerometer readings (`ax`, `ay`, `az`)
  * Gyroscope readings (`gx`, `gy`, `gz`)
* **Serial Output**: Data is streamed over the Arduino serial port at 115200 baud.
* **Audible Feedback**: Buzzer beeps every 30 seconds to indicate system status.
* **Configurable Sensor Ranges**:

  * Accelerometer: ±2G, ±4G, ±8G, ±16G
  * Gyroscope: ±250, ±500, ±1000, ±2000 deg/s
  * Digital filter bandwidth: 5–260 Hz

---

## Hardware Requirements

* Arduino-compatible board (e.g., Arduino Uno, Mega, Nano)
* MPU6050 IMU sensor
* Piezo buzzer (connected to pin 9)
* I2C connections for MPU6050 (`SDA`, `SCL`)

---

## Software Requirements

* Arduino IDE or VS Code with Arduino extension
* Libraries:

  * [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)
  * [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor)
  * Wire (built-in)

---

## Wiring

| Component   | Arduino Pin |
| ----------- | ----------- |
| MPU6050 SDA | A4          |
| MPU6050 SCL | A5          |
| Buzzer      | D9          |
| GND         | GND         |
| VCC         | 3.3V / 5V   |

> Adjust pins if using a board with different I2C or digital outputs.

---

## Usage

1. Connect the MPU6050 and buzzer according to the wiring table.
2. Open the project in Arduino IDE or VS Code.
3. Install required libraries.
4. Upload the sketch to the Arduino.
5. Open the Serial Monitor (115200 baud) or connect via a serial reader to capture the data packets.

---

## Data Packet Format

```cpp
struct Packet {
    uint32_t t;     // Timestamp in milliseconds
    float ax, ay, az;  // Accelerometer (m/s²)
    float gx, gy, gz;  // Gyroscope (rad/s)
};
```

* Packets are sent as raw bytes over serial.
* Example usage: read packets on a PC and parse for logging or MEKF simulation.

---

## Behavior

* Reads IMU data at ~200 Hz (5 ms interval).
* Beeps every 30 seconds to indicate the system is running.
* Includes initialization routines for accelerometer and gyro range, and filter bandwidth.

---

## Notes

* Ensure your serial buffer is read fast enough on the PC to avoid overflow.
* For simulation or testing, you can parse the raw packet bytes using Python, C++, or MATLAB.
* The loop uses `millis()` to manage periodic beeps without blocking sensor reads.

---

## License

MIT License — free to use and modify.
