# SAATHI Hardware Components & Pin Connections

## Project Overview

SAATHI (Smart Alzheimer's Assistance Technology for Healthy Independence) is an IoT-based assistive system designed to support Alzheimer's patients through emergency assistance, fall detection, health monitoring, GPS tracking, geofencing, and caregiver alerts.

## Hardware Components

| Component | Purpose |
|---|---|
| SIM7000G module | Cellular communication and GPS/location tracking |
| SOS Push Button | Manual emergency alert |
| MPU6050 | Motion sensing and fall detection |
| BMP280 | Pressure and environmental monitoring |
| MAX30102 | Heart-rate and SpO2 monitoring |

## Functional Mapping

### SIM7000G
Used for:
- Cellular communication
- GPS/location tracking
- Geofencing support
- Sending and receiving data through the cellular network

### SOS Button
Used to:
- Allow the user to manually trigger an emergency alert
- Notify the caregiver when assistance is required

### MPU6050
Used for:
- Accelerometer and gyroscope measurements
- Detecting abnormal motion patterns
- Fall detection

### BMP280
Used for:
- Barometric pressure measurement
- Environmental/altitude-related monitoring

### MAX30102
Used for:
- Heart-rate measurement
- SpO2 measurement

## Pin Connections

The exact GPIO assignments should match the final SAATHI.ino firmware. They are intentionally not listed here until verified against the final hardware wiring, to avoid documenting incorrect connections.

| Component | Interface | Connection |
|---|---|---|
| SIM7000G | UART / module interfaces | As defined in `firmware/SAATHI.ino` |
| SOS Button | Digital GPIO | As defined in `firmware/SAATHI.ino` |
| MPU6050 | I2C | As defined in `firmware/SAATHI.ino` |
| BMP280 | I2C | As defined in `firmware/SAATHI.ino` |
| MAX30102 | I2C | As defined in `firmware/SAATHI.ino` |

## Notes

- Check the final firmware and physical wiring before documenting specific GPIO numbers.
- Ensure the SIM7000G module has an appropriate power supply for cellular operation.
- Keep API keys, Wi-Fi credentials, Telegram bot tokens, and other private credentials out of the public repository.
