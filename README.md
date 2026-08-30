# BATT_CAN - DroneCAN Battery Node

A DroneCAN battery node implementation for APM open-source flight controllers, reading data from battery protection boards via Modbus and broadcasting battery information over DroneCAN protocol.

## Overview

This project enables direct communication between battery protection boards (with serial output) and flight controllers via DroneCAN protocol, bypassing the need for separate current sensors. It runs on ESP32/ESP32-C3 microcontrollers.

**Key Features:**
- Read battery protection board data via Modbus RTU (RS485)
- Broadcast battery info via DroneCAN protocol at configurable rates
- Send NodeStatus heartbeat at 1 Hz
- Send BatteryInfo at configurable rate (default 2 Hz)
- Serial command interface for parameter configuration
- Persistent parameter storage in EEPROM

## Hardware Support

| Board | Framework |
|-------|-----------|
| ESP32 DOIT DevKit V1 | Arduino |
| ESP32-C3 DevKitM-1 | Arduino |

## Supported Boards

- **ESP32 DOIT DevKit V1**
- **ESP32-C3 DevKitM-1** (with USB CDC support)

## Dependencies

- [ACAN_ESP32](https://github.com/pierremolinaro/ACAN_ESP32) - CAN driver for ESP32
- [ModbusMaster](https://github.com/4-20ma/ModbusMaster) - Modbus RTU master library

## Wiring

### ESP32 CAN Interface (TWAI)
| Function | ESP32 Pin | ESP32-C3 Pin |
|----------|-----------|--------------|
| CAN TX   | GPIO 5    | GPIO 4       |
| CAN RX   | GPIO 4    | GPIO 5       |

### Modbus RS485 Interface
| Function | ESP32 Pin |
|----------|----------|
| RS485 TX | GPIO 0   |
| RS485 RX | GPIO 1   |

## Modbus Registers

Reads data from battery protection board at two register ranges:

| Address | Registers | Description |
|---------|-----------|-------------|
| 0x1290  | 8         | Voltage (mV), Power (mW), Current (mA), Temperature (°C) |
| 0x12A6  | 10        | SOC (%), Remaining Capacity (mAH), Full Capacity (mAH), Cycle Count, SOH (%) |

## Configuration Commands

Connect via serial at 115200 baud and use these commands:

| Command | Description |
|---------|-------------|
| `help` / `?` | Show available commands |
| `status` | Display current system settings |
| `set modbus_addr <1-247>` | Set Modbus station address |
| `set can_id <1-127>` | Set CAN node ID |
| `set can_baud <baudrate>` | Set CAN baudrate (1000000/500000/250000) |
| `set telem_rate <0.1-50.0>` | Set telemetry rate in Hz |
| `set battery_index <0-255>` | Set battery index |
| `set print <on/off>` | Enable/disable debug output |
| `save` | Save parameters to EEPROM |
| `restart` | Reboot the device |
| `factory_reset` | Restore default parameters |

## Default Parameters

| Parameter | Default Value |
|-----------|---------------|
| Modbus Address | 0x01 |
| CAN Node ID | 73 |
| CAN Baudrate | 1000000 bps |
| Telemetry Rate | 2.0 Hz |
| Battery Index | 0 |

## DroneCAN Messages

### NodeStatus (1 Hz broadcast)
- Uptime
- Health status
- Operating mode

### BatteryInfo (configurable rate broadcast)
- Voltage (V)
- Current (A)
- Temperature (K)
- State of Charge (%)
- State of Health (%)
- Remaining Capacity (Ah)
- Full Charge Capacity (Ah)
- Average Power (W)
- Battery ID
- Status flags (charging/discharging/fully charged)

## Building

```bash
# Build for ESP32
pio run -e esp32doit-devkit-v1

# Build for ESP32-C3
pio run -e esp32C3

# Upload to device
pio run -e esp32doit-devkit-v1 --target upload
```

## Project Structure

```
BATT_CAN/
├── include/
│   ├── BatteryNode.h      # Battery node class header
│   └── config.h          # Configuration constants
├── src/
│   ├── BatteryNode.cpp   # Battery node implementation
│   └── main.cpp          # Main application
├── lib/                   # Local libraries
├── test/                  # Test files
├── platformio.ini        # PlatformIO configuration
```

## Applications

- High-voltage battery packs (6S, 14S, etc.)
- Large current measurement scenarios
- Direct battery data integration with APM flight controllers
- DroneCAN-based UAV battery monitoring systems

## License

MIT License

## Author

Dukang
