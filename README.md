# Stabilization of a bicycle using a reaction wheel

![](Bici_RW.gif)

Project focused on hardware integration and development of an embedded controller for stabilization of a bicycle using a reaction wheel.

## Hardware

- Microcontroller: Teensy 4.1 (ARM Cortex)
- Sensor: Inertial measurement Unit MinIMU-9 v5 communicated with uC via I2C. Integration from the [library provided](https://github.com/pololu/minimu-9-ahrs-arduino)
- Actuator: M8325S driven by [OdriveS1](https://docs.odriverobotics.com/v/latest/hardware/s1-datasheet.html). Communication with uC via UART
- Custon steel reaction wheel
- Commercial electric bycicle as mechanical platform

## Features

- State feedback control
- Control gains can be tuned at runtime via UART with uC
- Frequency and PRBS torque mode for parameter estimation of reaction wheel
- Runtime telemetry via UART for monitoring and debugging
