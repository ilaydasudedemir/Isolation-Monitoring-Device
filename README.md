# Insulation Monitoring Device (IMD)

Embedded firmware for an STM32F103C8T6 (Blue Pill) that measures, in real time, the insulation resistance between an electric vehicle's HV battery pack and chassis, and reports it over CAN bus. Built for a Formula Student / electric vehicle team project.

## Features
- ADC measurement of HV+ and HV- rail voltage relative to chassis (16-sample averaging)
- Insulation resistance calculated from the measured voltages
- Safe default behavior on short-circuit / excessive imbalance (resistance = 0)
- Buzzer + AKS (Accumulator/Master Disconnect) signal triggered when resistance drops below threshold
- Live voltage/resistance/CAN status shown on an SSD1306 OLED
- Insulation resistance + status code broadcast over CAN bus (ID `0x101`)
- The measurement loop runs on `HAL_GetTick()`-based timing
- Fail-safe `Error_Handler()`: instead of locking up silently on an init failure, it drives the buzzer + AKS output active before halting

## Hardware
| | |
|---|---|
| MCU | STM32F103C8T6 (LQFP48), 72 MHz |
| Display | SSD1306 128x64 OLED (I2C1) |
| Communication | CAN1 (~500 kbit/s region, `Prescaler=4, BS1=15TQ, BS2=2TQ`) |
| Measurement | ADC1 channel 7, software-triggered, voltage divider (R_TOP=1.41MOhm, R_BOTTOM=100kOhm) |

### Pin Map
| Pin | Role |
|---|---|
| PB0 | HV+ measurement line select (relay/mux) |
| PB1 | HV- measurement line select (relay/mux) |
| PA3 | Short-circuit / imbalance warning output |
| PC13 | CAN status LED |
| BUZZER_Pin | Audible alarm |
| AKS_OUT_Pin | Fault signal to the Accumulator/Master Disconnect system |
| I2C1 (SDA/SCL) | SSD1306 OLED |
| CAN1 (RX/TX) | Vehicle CAN bus |

## Build
1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. `File -> Import -> Existing Projects into Workspace` and select this folder.
3. `Project -> Build All`, then `Run -> Debug` to flash the board.

## Code Notes
- The measurement loop runs as a state machine: `ISO_STATE_STARTUP_BEEP -> SELECT_HVP -> READ_HVP -> SELECT_HVN -> READ_HVN -> PROCESS -> IDLE`. The startup beep, relay/mux settle times, and end-of-cycle wait are all handled non-blockingly via `HAL_GetTick()`.
- Threshold values (`HV_PRESENT_THRESHOLD_V`, `SHORT_CIRCUIT_RATIO`, `MIN_V_DIFF_V`) are defined as named constants.
- Insulation resistance limit per spec: `R_limit = V_bat,max(72V) x 100 Ohm/V = 8400 Ohm`.
- `ssd1306_tests.c/.h` (the library's unused demo files) were removed from the repo.

## Project Structure
```
Core/
  Inc/  main.h, ssd1306*.h, ...
  Src/  main.c, ssd1306*.c, ...
Drivers/        STM32 HAL + CMSIS
Isolation_Monitoring_Device.ioc   STM32CubeMX configuration
```

## Notes
- Engineered as the primary high-voltage safety monitoring firmware for an electric vehicle architecture.
