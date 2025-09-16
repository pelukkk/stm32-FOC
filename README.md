# Field-Oriented Control (FOC) on STM32 From Scratch

This project demonstrates a complete, low-level implementation of **Field-Oriented Control (FOC)** for controlling a **BLDC motor** using an **STM32F4 microcontroller**, without relying on high-level motor control libraries.

Built using **PlatformIO**, **STM32 HAL/LL drivers**, and **FreeRTOS**, this project is designed for learning and practical experimentation in precision motor control.

---

## Features Implemented

- Clarke & Park Transforms  
- Position feedback via AS5047 encoder  
- Phase current sampling via ADC  
- PI current controller  
- SVPWM (Space Vector PWM) generation  
- USB CDC interface for real-time tuning  
- Interrupt-driven timing and FreeRTOS tasking  

---

## Development Tools

- [PlatformIO](https://platformio.org/)
- STM32CubeMX
- STM32Cube HAL & LL Drivers  
- FreeRTOS (via PlatformIO package)  
- VS Code + Serial Monitor for debugging
- Altium Designer (for Hardware) you can see in STM32-FOC-hardware.zip
- ACTUATOR CONTROL APP in controller_app.zip

---

## UPDATE LOG

16-9-2025
- Update audio mode

20-8-2025
- ADC injected simultaneous mode to read currents and DC voltage faster
- Add command to change title, legends, and erase graph in serial plotter

9-8-2025
- Add eccentricity calibration for magnetic encoder
- CLI for auto calibration
- Change PID manual tuning settings
- Save configuration to flash memory
- Default config

29-7-2025
- Add parity check for AS5047P

---