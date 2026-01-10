# BLDC Speed Control on STM32 (Bare-Metal)

Firmware project to control the speed of a BLDC motor using STM32F4 (bare-metal drivers, no RTOS in phase 1).
Goal: educational reference implementation (not production-ready).

## Status
- Phase 0: project skeleton ✅
- Phase 1: Interfacing with BLDC Motor and encoder (AS5600 implemented) + basic speed loop (in progress)
- Phase 2: commutation / FOC (planned)

## Features (planned / in progress)

- [ ] SysTick as a system clock (SysTick)
- [ ] Speed measurement (AS5600 analog output via ADC)
- [ ] Logging system (USART)
- [ ] PWM generation + open-loop commutation test (timer)
- [ ] FOC (no current control in this version) + PI speed controller
- [ ] Simple user interface (UART CLI)
- [ ] Fault handling

## Known Limitations
- No current sensing / current control in this phase (hardware limitation).

## Hardware
- MCU: STM32F4 (Nucleo-F446RE)
- Motor: BLDC (specs: TBD)
- Driver/Power stage: TBD
- Sensors: AS5600 magnetic encoder
- Power supply: TBD

## Calibration
- AS5600 calibration (TBD)


## Toolchain
- STM32CubeIDE: (Version: 2.0.0)
- Compiler: arm-none-eabi-gcc (via CubeIDE)
- Programmer: ST-Link

## Build & Flash
1. Open the project in STM32CubeIDE.
2. Build: Project → Build Project.
3. Flash: Run → Debug (or Run).

## Project Structure
- `Inc/`
  - `board/` board support (pins, clocks, init)
  - `config/` project configuration
  - `drivers/` gpio/exti/systick/adc/pwm (TIM2) drivers (public APIs)
- `Src/` implementation (`board.c`, drivers, `irq_handlers.c`, `main.c`)

## Configuration
See `Inc/config/project_config.h`.

## Safety Notice
This project can drive real power hardware. Use current limiting, isolation, and proper safety procedures.
You are responsible for any damage or injury.

## License
This project is licensed under the Apache-2.0 License — see `LICENSE`.

## Contributing
Issues and pull requests are welcome. Please open an issue first for major changes.
