# PDU Firmware — 30 kW EV DC Fast Charger Module

Embedded firmware for a **30 kW Power Delivery Unit (PDU)** targeting the STM32G474RE microcontroller. The PDU converts 3-phase AC (260–530 VAC) to adjustable DC (150–1000 VDC) for EV fast charging, with 5-module CAN stacking to 150 kW.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                   STM32G474RE                       │
│                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │  HRTIM   │  │ ADC 1-5  │  │   App State      │  │
│  │ A,B,C →  │  │ Injected │  │   Machine        │  │
│  │  PFC PWM │  │ + DMA    │  │  (10 states)     │  │
│  │ D,E,F →  │  │ regular  │  │                  │  │
│  │  LLC PWM │  │          │  │  CC/CV/CP modes  │  │
│  └──────────┘  └──────────┘  └──────────────────┘  │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │  CORDIC  │  │  FDCAN   │  │   Fault Manager  │  │
│  │ sin/cos  │  │ 500 kbps │  │  24 fault sources│  │
│  │ for PLL  │  │ stacking │  │  HW < 200 ns     │  │
│  └──────────┘  └──────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────┘
```

## Key Features

- **Vienna PFC** — 3-phase interleaved, 65 kHz, dq-frame current control with SRF-PLL
- **LLC DC-DC** — 3-phase interleaved, PFM control (100–300 kHz), ZVS across full range
- **CC/CV charging** — IEC 61851-23 compliant, 0–100 A / 150–1000 VDC
- **CAN stacking** — 5-module parallel operation (150 kW), droop-based current sharing (<5% imbalance)
- **Protection** — Hardware OCP (<200 ns), OVP (<1 µs), OTP, short-circuit, ground fault (IEC 62955)
- **Communications** — OCPP 1.6 / ISO 15118 interface via charger controller CAN
- **Burst mode** — LLC burst mode at light load for >85% efficiency at 1 kW

## Repository Structure

```
pdu-firmware/
├── README.md
├── docs/                  ← pdu-docs submodule (design documentation)
├── Core/                  ← STM32 HAL/LL init, main.c, system config
│   ├── Inc/
│   └── Src/
├── App/                   ← Application code
│   ├── StateMachine/      ← 10-state application FSM
│   ├── Control/           ← PFC dq control, LLC PFM, PI controllers
│   ├── ADC/               ← ADC pipeline, DMA, filters
│   ├── Protection/        ← Fault state machine, HRTIM fault inputs
│   ├── CAN/               ← CAN protocol, master FSM, stacking
│   ├── PowerSequence/     ← Soft-start, shutdown, burst mode
│   └── Diagnostics/       ← UART CLI, fault logging, calibration
├── Drivers/               ← STM32G4 HAL/LL drivers
├── Middlewares/           ← Third-party (if any)
├── pdu-firmware.ioc       ← STM32CubeMX project
├── Makefile               ← ARM GCC build
├── .clang-format          ← Code style
└── .github/
    └── workflows/
        └── build.yml      ← CI: build + MISRA-C static analysis
```

## Hardware Target

| Parameter | Value |
|-----------|-------|
| MCU | STM32G474RET6 (Cortex-M4F, 170 MHz) |
| HRTIM | 6 timers (A–C: PFC, D–F: LLC), 5.44 GHz DLL |
| ADC | 5 instances, injected (HRTIM-triggered) + regular (DMA) |
| CAN | FDCAN1, 500 kbps, 120 Ω terminated |
| Debug | SWD/JTAG, UART 115200 baud |

## Building

```bash
# Prerequisites: ARM GCC toolchain (arm-none-eabi-gcc)
make all

# Flash via ST-Link
make flash

# Static analysis (MISRA-C subset)
make lint
```

## Documentation

Design documentation is linked as a git submodule at `docs/`. After cloning:

```bash
git clone --recurse-submodules https://github.com/manasXP/pdu-firmware.git

# Or if already cloned:
git submodule update --init --recursive
```

Key design documents:
- [Firmware Architecture](docs/docs/06-Firmware%20Architecture.md) — HRTIM map, ADC allocation, control loops, CAN protocol
- [Application State Machine](docs/docs/06-Firmware-Design/01-Application%20State%20Machine.md) — 10-state FSM
- [Power-On Sequence](docs/docs/06-Firmware-Design/02-Power-On%20Sequence%20and%20Ramp%20Control.md) — Startup/shutdown timing
- [Fault Recovery](docs/docs/06-Firmware-Design/03-Fault%20State%20Machine%20and%20Recovery.md) — 24 fault sources, severity matrix
- [CAN Stacking](docs/docs/06-Firmware-Design/05-CAN%20Master%20and%20Module%20Stacking.md) — Master FSM, current sharing
- [Project Management](docs/docs/12-Project-Management/__init.md) — Epics, sprints, milestones

## Coding Standards

- MISRA-C:2012 subset (safety-critical modules)
- Fixed-point arithmetic in ISR context (no floating point in control loops)
- ISR budget: <3 µs for HRTIM period ISR at 65 kHz
- Naming: `Module_Function()` for public, `module_helper()` for static

## Related Repositories

| Repo | Description |
|------|-------------|
| [pdu-docs](https://github.com/manasXP/pdu-docs) | Design documentation, KiCad schematics/PCB, project management |
| [pdu-firmware](https://github.com/manasXP/pdu-firmware) | This repo — STM32G474RE embedded firmware |

## License

*TBD*
