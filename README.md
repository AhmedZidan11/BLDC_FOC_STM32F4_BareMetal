# BLDC Speed Control on STM32 (Bare-Metal)

This project is a bare-metal BLDC motor control firmware for STM32F446RE.
It is written in C with CMSIS / register-level code only. No HAL, no LL, and no RTOS are used.

The current goal is to build a small but real working reference for sensored closed-loop speed control of a BLDC motor, with code that stays readable and easy to inspect.

This is an educational and portfolio-oriented project. It is not production-ready.

## Current Status

The project is already beyond the skeleton stage.

Current active application:
- startup electrical alignment
- sensored closed-loop mechanical speed control
- AS5600 analog angle measurement through ADC
- electrical angle reconstruction from measured mechanical angle
- voltage-mode FOC actuation
- filtered speed feedback
- speed control loop with telemetry output over USART2

What is working now:
- bare-metal board bring-up on Nucleo-F446RE
- TIM1 3PWM output stage
- SysTick-based timing and scheduler utilities
- ADC-based AS5600 analog sensor path
- angle publish filtering with wrap-aware continuity handling
- closed-loop speed run with long stable logging sessions
- lightweight UART telemetry for runtime evaluation

What is intentionally **not** in this version:
- current sensing
- current control loop
- production fault management
- full command-line interface
- final tuning / optimization

## Project Scope

This repository focuses on a simple and inspectable control stack:

1. read mechanical angle from AS5600 analog output
2. publish a stable mechanical angle sample
3. reconstruct electrical angle
4. estimate mechanical speed
5. run a speed controller
6. apply q-axis voltage command through voltage-mode FOC

The project is meant to show firmware structure, low-level driver work, control-loop integration, and practical debugging on real hardware.

## Hardware

Current test setup:
- MCU board: **STM32 Nucleo-F446RE**
- MCU family: **STM32F4**
- Sensor: **AS5600** magnetic angle sensor (analog output)
- Motor: small BLDC / gimbal-style motor used for bench tests
- Power stage: external 3-phase BLDC driver / inverter board

Notes:
- this version uses the external power stage only as a power stage
- no current shunt feedback is used in the current project phase

## Software / Control Notes

Important design choices in the current version:
- bare-metal C only
- CMSIS / direct register access
- fixed-rate sensor sampling
- speed-loop update period: **1 ms**
- angle publish interval: **500 us**
- UART telemetry interval: **100 ms**
- speed control target in the current test setup: **500000 mrpm**
- speed PI is currently operated in **P-only mode** (`Ki = 0`) for the present validation stage

This means the current control result is good enough for a working prototype, but not yet the final tuned version.

## Repository Structure

- `Inc/`
  - `board/` board support and hardware mapping
  - `config/` application and project configuration
  - `drivers/` low-level bare-metal drivers
  - `motor/` motor-control related modules
- `Src/`
  - board, driver, and motor module implementations
  - `irq_handlers.c`
  - `main.c`

Main modules used by the active application include:
- GPIO / ADC / USART2 / SysTick / PWM TIM1 drivers
- `as5600_analog`
- `motor_electrical_angle`
- `motor_speed_feedback`
- `motor_speed_pi`
- `motor_speed_reference_estimator`
- `motor_foc_voltage`

## Build and Flash

1. Open the project in **STM32CubeIDE**.
2. Build the project.
3. Flash and run it through **ST-Link**.
4. Open the USART log on the PC side and capture runtime telemetry.

## Runtime Telemetry

The normal runtime log uses compact `S,...` lines for longer test runs.

Current line format:

`S,timestamp,mechanical_angle_deg_x10,velocity_filtered_mrpm,velocity_target_final_mrpm,velocity_reference_mrpm,velocity_error_mrpm,uq_command_permyriad`

This is used to inspect speed tracking and controller behavior over time.

## Configuration

Main application settings are in:
- `Inc/config/app_motor_test_config.h`

Project-wide settings are in:
- `Inc/config/project_config.h`

## Known Limitations

- no current sensing or current control
- speed loop is still in an early tuning stage
- telemetry is intended for evaluation, not for a polished user interface
- hardware setup is still a bench-test setup, not a packaged system
- this version should be treated as a working prototype, not a finished motor-control product

## Suggested Next Steps

Planned next work after this stage:
- refine speed-controller tuning
- improve README / plots / documentation for portfolio presentation
- document hardware setup more clearly
- add more formal test evidence and result plots
- later evaluate whether current control should be added in a future phase

## Safety Notice

This firmware can drive real motor power hardware.
Use current limiting, safe supply settings, and proper bench-test precautions.
You are responsible for safe wiring and safe operation.

## License

This project is licensed under the Apache-2.0 License. See `LICENSE` for details.

## Contributing

Issues and pull requests are welcome.
For larger changes, please open an issue first so the change can be discussed before implementation.
