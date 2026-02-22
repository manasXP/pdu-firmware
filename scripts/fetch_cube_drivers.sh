#!/usr/bin/env bash
# fetch_cube_drivers.sh — Download STM32CubeG4 HAL/LL drivers and CMSIS headers
#
# Usage: ./scripts/fetch_cube_drivers.sh
#
# Clones only the required submodules from STM32CubeG4 using sparse checkout,
# then copies the needed subset into the project tree.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CUBE_REPO="https://github.com/STMicroelectronics/STM32CubeG4.git"
CUBE_TAG="v1.6.0"
CUBE_DIR="${PROJECT_ROOT}/.cube_cache"

DRIVERS_DIR="${PROJECT_ROOT}/Drivers"
CORE_SRC="${PROJECT_ROOT}/Core/Src"

# Check if drivers are already populated
if [ -f "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c" ]; then
    echo "[fetch_cube_drivers] Drivers already present — skipping download."
    exit 0
fi

echo "[fetch_cube_drivers] Fetching STM32CubeG4 ${CUBE_TAG} ..."

if [ ! -d "${CUBE_DIR}/.git" ]; then
    rm -rf "${CUBE_DIR}"

    # Sparse clone: only Drivers/CMSIS/Include (non-submodule)
    git clone --depth 1 --branch "${CUBE_TAG}" --single-branch \
        --filter=blob:none --sparse \
        "${CUBE_REPO}" "${CUBE_DIR}"

    cd "${CUBE_DIR}"
    git sparse-checkout set Drivers/CMSIS/Include

    # Initialize only the submodules we need (shallow)
    git submodule init Drivers/STM32G4xx_HAL_Driver
    git submodule init Drivers/CMSIS/Device/ST/STM32G4xx
    git submodule update --depth 1

    cd "${PROJECT_ROOT}"
else
    echo "[fetch_cube_drivers] Using cached checkout at ${CUBE_DIR}"
fi

CUBE_HAL_SRC="${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src"
CUBE_HAL_INC="${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc"
CUBE_CMSIS_INC="${CUBE_DIR}/Drivers/CMSIS/Include"
CUBE_DEVICE_INC="${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include"
CUBE_DEVICE_SRC="${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates"
CUBE_STARTUP="${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc"

# ---------- CMSIS Core Headers ----------
echo "[fetch_cube_drivers] Copying CMSIS core headers ..."
mkdir -p "${DRIVERS_DIR}/CMSIS/Include"
cp "${CUBE_CMSIS_INC}/cmsis_compiler.h"  "${DRIVERS_DIR}/CMSIS/Include/"
cp "${CUBE_CMSIS_INC}/cmsis_gcc.h"       "${DRIVERS_DIR}/CMSIS/Include/"
cp "${CUBE_CMSIS_INC}/cmsis_version.h"   "${DRIVERS_DIR}/CMSIS/Include/"
cp "${CUBE_CMSIS_INC}/core_cm4.h"        "${DRIVERS_DIR}/CMSIS/Include/"
cp "${CUBE_CMSIS_INC}/mpu_armv7.h"       "${DRIVERS_DIR}/CMSIS/Include/"

# ---------- STM32G4xx Device Headers ----------
echo "[fetch_cube_drivers] Copying STM32G4xx device headers ..."
mkdir -p "${DRIVERS_DIR}/CMSIS/Device/ST/STM32G4xx/Include"
cp "${CUBE_DEVICE_INC}/stm32g4xx.h"        "${DRIVERS_DIR}/CMSIS/Device/ST/STM32G4xx/Include/"
cp "${CUBE_DEVICE_INC}/stm32g474xx.h"      "${DRIVERS_DIR}/CMSIS/Device/ST/STM32G4xx/Include/"
cp "${CUBE_DEVICE_INC}/system_stm32g4xx.h" "${DRIVERS_DIR}/CMSIS/Device/ST/STM32G4xx/Include/"

# ---------- HAL / LL Driver Sources ----------
echo "[fetch_cube_drivers] Copying HAL/LL driver sources ..."
mkdir -p "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc"
mkdir -p "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Src"

# HAL headers — copy all
cp "${CUBE_HAL_INC}"/*.h "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc/"
if [ -d "${CUBE_HAL_INC}/Legacy" ]; then
    mkdir -p "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc/Legacy"
    cp "${CUBE_HAL_INC}/Legacy/"*.h "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc/Legacy/"
fi

# HAL sources — only the modules we need
HAL_MODULES=(
    stm32g4xx_hal
    stm32g4xx_hal_cortex
    stm32g4xx_hal_rcc
    stm32g4xx_hal_rcc_ex
    stm32g4xx_hal_pwr
    stm32g4xx_hal_pwr_ex
    stm32g4xx_hal_flash
    stm32g4xx_hal_flash_ex
    stm32g4xx_hal_flash_ramfunc
    stm32g4xx_hal_gpio
    stm32g4xx_hal_dma
    stm32g4xx_hal_dma_ex
    stm32g4xx_hal_hrtim
    stm32g4xx_hal_adc
    stm32g4xx_hal_adc_ex
    stm32g4xx_hal_fdcan
    stm32g4xx_hal_tim
    stm32g4xx_hal_tim_ex
    stm32g4xx_hal_uart
    stm32g4xx_hal_uart_ex
    stm32g4xx_hal_cordic
    stm32g4xx_hal_fmac
    stm32g4xx_ll_rcc
    stm32g4xx_ll_utils
)

for mod in "${HAL_MODULES[@]}"; do
    if [ -f "${CUBE_HAL_SRC}/${mod}.c" ]; then
        cp "${CUBE_HAL_SRC}/${mod}.c" "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Src/"
    else
        echo "[fetch_cube_drivers] WARNING: ${mod}.c not found — skipping"
    fi
done

# ---------- System file ----------
echo "[fetch_cube_drivers] Copying system_stm32g4xx.c ..."
cp "${CUBE_DEVICE_SRC}/system_stm32g4xx.c" "${CORE_SRC}/system_stm32g4xx.c"

# ---------- Startup assembly ----------
echo "[fetch_cube_drivers] Copying startup assembly ..."
STARTUP_FILE=$(find "${CUBE_STARTUP}" -name "startup_stm32g474*.s" -print -quit 2>/dev/null || true)
if [ -n "${STARTUP_FILE}" ]; then
    cp "${STARTUP_FILE}" "${CORE_SRC}/startup_stm32g474retx.s"
else
    echo "[fetch_cube_drivers] ERROR: Startup file not found"
    exit 1
fi

# ---------- Linker script ----------
if [ ! -f "${PROJECT_ROOT}/STM32G474RETx_FLASH.ld" ]; then
    echo "[fetch_cube_drivers] Generating linker script ..."
    cat > "${PROJECT_ROOT}/STM32G474RETx_FLASH.ld" << 'LDEOF'
/*
 * STM32G474RETx_FLASH.ld — Linker script for STM32G474RET6
 * Flash: 512 KB,  SRAM: 128 KB (96 KB SRAM1 + 32 KB SRAM2)
 * CCM SRAM: 32 KB (mapped at 0x10000000)
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
    RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
}

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(.RamFunc)
        *(.RamFunc*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    _siccmram = LOADADDR(.ccmram);

    .ccmram :
    {
        . = ALIGN(4);
        _sccmram = .;
        *(.ccmram)
        *(.ccmram*)
        . = ALIGN(4);
        _eccmram = .;
    } >CCMRAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
LDEOF
fi

echo "[fetch_cube_drivers] Done. Driver files installed."
