# Solar Panel Automatic Dust Cleaning System

## Overview

This project is an autonomous solar panel cleaning system based on an ESP32 microcontroller and the GP2Y1010AU0F dust sensor.

The system detects dust accumulation on the solar panel and automatically activates a cleaning mechanism using a servo motor. A voltage divider is implemented at pin 5 (analog output) of the GP2Y1010AU0F sensor to ensure the output voltage is suitable for the ESP32 ADC input.

The system also integrates a relay-controlled water pump to assist the cleaning process and a current sensor to monitor the electrical current consumed by the load.

## System Features

- **Dust Detection:** Uses the GP2Y1010AU0F optical dust sensor to detect the presence of dust.
- **Automatic Cleaning Mechanism:** Activates a servo motor to operate the wiper mechanism when dust is detected.
- **Water Pump Control:** Uses a relay to activate the water pump when dust is detected or when the servo-driven wiper is operating.
- **Current Monitoring:** Measures the electrical current flowing through the load using a current sensor.
- **Autonomous Power System:** Operates independently using a UPS-based power system, with a solar panel as the primary energy source and a battery as the secondary power source.

## Hardware Components

| Component | Function |
|---|---|
| ESP32 | Main microcontroller |
| GP2Y1010AU0F | Optical dust detection |
| Voltage Divider | Analog output voltage adjustment |
| Servo Motor | Wiper movement |
| Relay Module | Water pump control |
| Current Sensor | Load current monitoring |
| Solar Panel | Primary power source |
| Battery / UPS | Backup power supply |
| Water Pump | Assists the panel cleaning process |

## System Operation

1. The GP2Y1010AU0F sensor continuously monitors dust conditions.
2. The ESP32 processes the sensor readings to determine whether dust is detected.
3. When dust is detected, the servo motor activates the wiper mechanism.
4. The relay activates the water pump to assist the cleaning process.
5. The current sensor monitors the electrical current consumed by the load.
6. The system operates autonomously using solar energy, with battery backup provided by the UPS system.

## Development

This project is developed using the ESP32 platform and is intended to provide an autonomous solution for maintaining solar panel cleanliness while monitoring electrical load consumption.

