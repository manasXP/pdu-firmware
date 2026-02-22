# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Embedded firmware for a **30 kW Power Delivery Unit (PDU)** targeting **STM32G474RET6** (Cortex-M4F, 170 MHz). Converts 3-phase AC (260–530 VAC) to adjustable DC (150–1000 VDC) for EV fast charging. Supports 5-module CAN stacking for 150 kW total.

## Build Commands

```bash
make all              # Build firmware (debug, -Og -g3)
make RELEASE=1        # Build firmware (release, -O2)
make flash            # Flash via ST-Link (st-flash)
make lint             # MISRA-C static analysis (cppcheck --addon=misra)
make clean            # Remove build/ directory
```

**Toolchain:** `arm-none-eabi-gcc` (C11, Cortex-M4 hard-float). No test framework yet.

## Architecture

**Layered design:** Hardware (Drivers/CMSIS + HAL) → Core (peripheral init) → App (7 modules)

### Application Modules (`App/`)

| Module | ISR/Rate | Purpose |
|--------|----------|---------|
| **StateMachine** | 1 kHz (TIM6) | 10-state FSM: POWER_ON → STANDBY → PLL_LOCK → SOFT_START_PFC → SOFT_START_LLC → RUN → DERATE → FAULT → SHUTDOWN → DISABLED |
| **Control** | PFC 65 kHz, LLC 150 kHz (HRTIM) | PFC dq-frame PI control, LLC PFM control. 5 PI instances with anti-windup |
| **ADC** | HRTIM-triggered | 5 ADC instances, 13 channels (voltages, currents, temps), 16x oversampling |
| **Protection** | Hardware <200 ns | 24 fault sources, 4 severity levels (NONE/WARNING/MAJOR/CRITICAL) |
| **CAN** | 10 ms broadcast | FDCAN1 500 kbps, module stacking protocol, droop-based current sharing |
| **PowerSequence** | — | PFC/LLC soft-start ramps, shutdown sequence, burst mode |
| **Diagnostics** | Polled | UART CLI (115200 baud, USART2), fault logging, calibration |

### Execution Model

- `main()` initializes HAL peripherals then calls each `App_*_Init()`, runs `App_SM_Run()` + `App_Diagnostics_Poll()` in the main loop
- HRTIM period ISRs drive control loops — **ISR budget is <3 µs at 65 kHz**
- State transitions broadcast over CAN via `State_Transition()`

### Key Thresholds (`Core/Inc/main.h`)

PFC_OCP: 78 A, BUS_OVP: 966 V, OUT_OVP: 1050 V, OTP_DERATE: 100°C, OTP_SHUTDOWN: 115°C, PLL_LOCK_TIMEOUT: 2 s, STARTUP_TIMEOUT: 6 s

## Coding Conventions

- **MISRA-C:2012** subset enforced via `make lint`
- **Fixed-point arithmetic** preferred in ISR context (avoid float in time-critical paths)
- **Naming:** Public API `Module_Function()`, static helpers `module_helper()`, types suffixed `_t`
- **Formatting:** `.clang-format` — LLVM base, 4-space indent, Allman braces, 100-char line limit, right pointer alignment (`char *p`)
- **Headers:** include guards `#ifndef MODULE_H`, `extern "C"` guards, Doxygen `@file`/`@brief`/`@param`/`@return`

## Documentation

Design docs live in `docs/` (git submodule `pdu-docs`). Key references:
- `docs/docs/06-Firmware Architecture.md` — overall firmware design
- `docs/docs/06-Firmware-Design/` — detailed per-module design specs (state machine, fault handling, ADC pipeline, CAN protocol, burst mode)
- `docs/research/` — topology trade studies, thermal parameters

## Workflow

All epics and stories are tracked as GitHub Issues in this repository. Before starting work, reference the relevant issue. After completing work on a story, update the corresponding GitHub issue with a summary of changes and close it if fully resolved (use `gh issue comment` and `gh issue close`).

## Skills

The **embedded-systems** skill (`.claude/skills/embedded-systems/`) provides reference patterns for STM32 bare-metal programming, RTOS (FreeRTOS), communication protocols (I2C, SPI, UART, CAN), memory optimization, and power management. Consult its reference files when implementing drivers, peripherals, or optimizing resource usage.

## CI

GitHub Actions (`.github/workflows/build.yml`): builds with ARM GCC and runs `make lint` on push to `main`/`develop` and PRs to `main`.
