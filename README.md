# AquaBot_ATO - Automatic Top-Off System

## Project Description

AquaBot_ATO is an innovative automatic top-off system designed specifically for aquariums. It efficiently regulates water levels while ensuring the safety of aquarium operations. By utilizing modern technology and intelligent safety mechanisms, AquaBot_ATO guarantees that your aquarium is always optimally supplied with water, eliminating the risk of overflow.

## System Overview

The core of the AquaBot_ATO system is an Arduino Nano, which controls and monitors all functions. The system is powered by a 12V power supply, with a step-down converter reducing the voltage to 5V for the Arduino. 

### Water Level Sensors

Stainless steel probes are used as water level sensors, divided into two separate systems:

1. **Main Control:** The first sensor is directly connected to the Arduino and measures the water level hourly. A 10k resistor ensures that the Arduino reads the sensor value through an analog input. The sensor checks for a stable high or low value over a period of 5 seconds to determine if it is in contact with water.

2. **Emergency Shutdown:** The second sensor is decoupled from the Arduino and serves as a safety mechanism. If the water level reaches above a critical point, a MOSFET (IRLZ44N) shuts down the pump to prevent overflow. 

## Safety Mechanisms

AquaBot_ATO implements two essential safety mechanisms to prevent aquarium overflow:

- **Pump Control:** The Arduino allows the pump to run for a maximum of 10% longer than the previous pumping duration. If this time is exceeded, the Arduino deactivates the MOSFET and triggers an alarm that requires confirmation before a new refill operation can start.

- **Emergency Shutdown:** If the water level exceeds the sensor and activates the emergency shutdown, a second MOSFET is deactivated, cutting off the power supply to the pump to prevent overflow.

## User Interaction and Status Display

The system status is indicated by an RGB LED on the sensor and displayed on a connected OLED display. A simple push button allows users to manually interact with the device and check the system status.

- **Constant Green:** Water is filled up to the desired level.
- **Blinking Blue:** Pump is running / water is filling up.
- **Blinking Red:** Error detected in the system.
- **Constant Yellow:** Refill reservoir is about to be empty (warning).
- **Blinking Yellow:** Refill reservoir is critically low (immediate attention required).

## Protection Measures

To protect the electronics from voltage spikes and inductive backflows, a capacitor (100-400 µF electrolytic capacitor) and a diode (1N5401) are integrated. These components ensure the longevity and reliability of the system, especially during the pump's on/off cycles.

## Conclusion

AquaBot_ATO is a well-thought-out and safe solution for automatic water level control in aquariums. With its advanced safety mechanisms and user-friendly interface, it provides aquarium owners with the assurance that their fish and plants are always in an optimal environment.

## Components Used

- **Microcontroller:** Arduino Nano
- **Power Supply:** 12V power supply with a step-down converter (12V to 5V)
- **Water Level Sensors:** Stainless steel probes
- **MOSFETs:** IRLZ44N
- **Diode:** 1N5401
- **Capacitor:** 100-400 µF electrolytic capacitor
- **Resistor:** 10k resistor

Feel free to contribute to the project or reach out with any questions or suggestions!