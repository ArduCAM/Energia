/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== Board.c ========
 *  This file is responsible for setting up the board specific items for the
 *  CC3200_LP Launch Pad board.
 */
 
#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
//#include <inc/hw_gpio.h>

#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/pin.h>
#include <driverlib/prcm.h>
#include <driverlib/udma.h>
#include <driverlib/gpio.h>

#include "Board.h"

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[64];
static bool dmaInitialized = false;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct hwiStruct;

/*
 *  ======== Board_errorDMAHwi ========
 */
static Void Board_errorDMAHwi(UArg arg)
{
//    System_printf("DMA error code: %d\n", MAP_uDMAErrorStatusGet());
    MAP_uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== Board_initDMA ========
 */
void Board_initDMA(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if (!dmaInitialized) {
        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(hwiStruct), INT_UDMAERR, Board_errorDMAHwi,
                      &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't create DMA error hwi");
        }

        MAP_PRCMPeripheralClkEnable(PRCM_UDMA, PRCM_RUN_MODE_CLK);
        MAP_PRCMPeripheralReset(PRCM_UDMA);
        MAP_uDMAEnable();
        MAP_uDMAControlBaseSet(dmaControlTable);

        dmaInitialized = true;
    }
}

/*
 *  ======== Board_initGeneral ========
 */
void Board_initGeneral(void)
{
    /*  Reset DMA + other essential peripheral initialization
     *  ASSUMED by the simplelink and driverlib libraries
     */
    PRCMCC3200MCUInit();

    /* Configure pins as specified in the current configuration */

    /*
     * ======== Enable Peripheral Clocks ========
     * Enable all clocks (because wiring can use any pin for in any mode
     * at runtime)
     */
    MAP_PRCMPeripheralClkEnable(PRCM_CAMERA, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_I2S, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_SDHOST, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);


    /* ======== UART Pin Configuration ======== */

    /* Serial */

    /*
     * Configure LaunchPad P2.9 as a UART1: UART1 TX (via USB port)
     *     device pin: 55 (UART0_TX)
     *     Wiring id : 12
     */
    MAP_PinTypeUART(PIN_55, PIN_MODE_6);

    /*
     * Configure LaunchPad P3.3 as a UART1: UART1 RX (via USB port)
     *     device pin: 57 (UART0_RX)
     *     Wiring id : 23
     */
    MAP_PinTypeUART(PIN_57, PIN_MODE_6);

    /* Serial1 */

    /*
     * Configure LaunchPad P1.4 as a UART0: UART0 TX
     *     device pin: 3 (UART0_TX)
     *     Wiring id : 4
     */
    MAP_PinTypeUART(PIN_03, PIN_MODE_7);

    /*
     * Configure LaunchPad P1.3 as a UART0: UART0 RX
     *     device pin: 4 (UART0_RX)
     *     Wiring id : 3
     */
    MAP_PinTypeUART(PIN_04, PIN_MODE_7);


    /* ======== SPI Pin Configuration ======== */

    /*
     * Configure LaunchPad P1.7 as a SPI pin: SPI CLK
     *     device pin: 5 (GSPI_CLK)
     *     Wiring id : 7
     */
    MAP_PinTypeSPI(PIN_05, PIN_MODE_7);

    /*
     * Configure LaunchPad P2.6 as a SPI pin: SPI MOSI
     *     device pin: 7 (GSPI_MOSI)
     *     Wiring id : 15
     */
    MAP_PinTypeSPI(PIN_07, PIN_MODE_7);

    /*
     * Configure LaunchPad P2.7 as a SPI pin: SPI MISO
     *     device pin: 6 (GSPI_MISO)
     *     Wiring id : 14
     */
    MAP_PinTypeSPI(PIN_06, PIN_MODE_7);


    /* ======== I2C Pin Configuration ======== */

    /*
     * Configure LaunchPad P1.9 as a I2C pin: LaunchPad Sensor Data (via I2C)
     *     device pin: 1 (I2C_SCL)
     *     Wiring id : 9
     */
    MAP_PinTypeI2C(PIN_01, PIN_MODE_1);

    /*
     * Configure LaunchPad P1.10 as a I2C pin: LaunchPad Sensor Data (via I2C)
     *     device pin: 2 (I2C_SDA)
     *     Wiring id : 10
     */
    MAP_PinTypeI2C(PIN_02, PIN_MODE_1);
}

/*
 * ======== GPIO driver ========
 */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC3200.h>

/* GPIO configuration structure definitions */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOCC3200_config, ".const:GPIOCC3200_config")
#pragma DATA_SECTION(gpioPinConfigs, ".data:gpioPinConfigs")
#pragma DATA_SECTION(gpioCallbackFunctions, ".data:gpioCallbackFunctions")
#endif

GPIO_PinConfig gpioPinConfigs[] = {
    /* port_pin */
    GPIOCC3200_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  0  - dummy */
                          
    /* pins 1-10 */
    GPIOCC3200_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  1  - 3.3V */
    GPIOCC3200_GPIO_03 | GPIO_DO_NOT_CONFIG,    /*  2  - GPIO_03 */
    GPIOCC3200_GPIO_13 | GPIO_DO_NOT_CONFIG,    /*  3  - GPIO_13 */
    GPIOCC3200_GPIO_12 | GPIO_DO_NOT_CONFIG,    /*  4  - GPIO_12 */
    GPIOCC3200_GPIO_06 | GPIO_DO_NOT_CONFIG,    /*  5  - GPIO_06 */
    GPIOCC3200_GPIO_04 | GPIO_DO_NOT_CONFIG,    /*  6  - GPIO_04 */
    GPIOCC3200_GPIO_14 | GPIO_DO_NOT_CONFIG,    /*  7  - GPIO_14 */
    GPIOCC3200_GPIO_07 | GPIO_DO_NOT_CONFIG,    /*  8  - GPIO_07 */
    GPIOCC3200_GPIO_10 | GPIO_DO_NOT_CONFIG,    /*  9  - GPIO_10 */
    GPIOCC3200_GPIO_11 | GPIO_DO_NOT_CONFIG,    /*  10 - GPIO_11 */
                          
    /* pins 11-20 */
    GPIOCC3200_GPIO_22 | GPIO_DO_NOT_CONFIG,    /*  11 - GPIO_22 */
    GPIOCC3200_GPIO_01 | GPIO_DO_NOT_CONFIG,    /*  12 - GPIO_01 */
    GPIOCC3200_GPIO_25 | GPIO_DO_NOT_CONFIG,    /*  13 - GPIO_25 */
    GPIOCC3200_GPIO_15 | GPIO_DO_NOT_CONFIG,    /*  14 - GPIO_15 */
    GPIOCC3200_GPIO_16 | GPIO_DO_NOT_CONFIG,    /*  15 - GPIO_16 */
    GPIOCC3200_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  16 - RESET */
    GPIOCC3200_GPIO_31 | GPIO_DO_NOT_CONFIG,    /*  17 - GPIO_31 */
    GPIOCC3200_GPIO_17 | GPIO_DO_NOT_CONFIG,    /*  18 - GPIO_17 */
    GPIOCC3200_GPIO_28 | GPIO_DO_NOT_CONFIG,    /*  19 - GPIO_28 */
    GPIOCC3200_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  20 - GND */
                          
    /* pins 21-30 */
    GPIOCC3200_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  21 - 5V */
    GPIOCC3200_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  22 - GND */
    GPIOCC3200_GPIO_02 | GPIO_DO_NOT_CONFIG,    /*  23 - GPIO_02 */
    GPIOCC3200_GPIO_05 | GPIO_DO_NOT_CONFIG,    /*  24 - GPIO_05 */
    GPIOCC3200_GPIO_03 | GPIO_DO_NOT_CONFIG,    /*  25 - GPIO_03 */
    GPIOCC3200_GPIO_04 | GPIO_DO_NOT_CONFIG,    /*  26 - GPIO_04 */
    GPIOCC3200_GPIO_08 | GPIO_DO_NOT_CONFIG,    /*  27 - GPIO_08 */
    GPIOCC3200_GPIO_30 | GPIO_DO_NOT_CONFIG,    /*  28 - GPIO_30 */
    GPIOCC3200_GPIO_09 | GPIO_DO_NOT_CONFIG,    /*  29 - GPIO_09 */
    GPIOCC3200_GPIO_00 | GPIO_DO_NOT_CONFIG,    /*  30 - GPIO_00 */
                          
    /* pins 31-40 */
    GPIOCC3200_GPIO_24 | GPIO_DO_NOT_CONFIG,    /*  31 - GPIO_24 */
    GPIOCC3200_GPIO_23 | GPIO_DO_NOT_CONFIG,    /*  32 - GPIO_23 */
    GPIOCC3200_GPIO_05 | GPIO_DO_NOT_CONFIG,    /*  33 - GPIO_05 */
    GPIOCC3200_GPIO_07 | GPIO_DO_NOT_CONFIG,    /*  34 - GPIO_07 */
    GPIOCC3200_GPIO_28 | GPIO_DO_NOT_CONFIG,    /*  35 - GPIO_28 */
    GPIOCC3200_GPIO_25 | GPIO_DO_NOT_CONFIG,    /*  36 - GPIO_25 */
    GPIOCC3200_GPIO_09 | GPIO_DO_NOT_CONFIG,    /*  37 - GPIO_09 */
    GPIOCC3200_GPIO_24 | GPIO_DO_NOT_CONFIG,    /*  38 - GPIO_24 */
    GPIOCC3200_GPIO_10 | GPIO_DO_NOT_CONFIG,    /*  39 - GPIO_10 */
    GPIOCC3200_GPIO_11 | GPIO_DO_NOT_CONFIG,    /*  40 - GPIO_11 */
};

GPIO_CallbackFxn gpioCallbackFunctions[] = {
    /* port_pin */
    NULL,  /*  0  - dummy */

    /* pins 1-10 */
    NULL,  /*  1  - 3.3V */
    NULL,  /*  2  - GPIO_03 */
    NULL,  /*  3  - GPIO_13 */
    NULL,  /*  4  - GPIO_12 */
    NULL,  /*  5  - GPIO_06 */
    NULL,  /*  6  - GPIO_04 */
    NULL,  /*  7  - GPIO_14 */
    NULL,  /*  8  - GPIO_07 */
    NULL,  /*  9  - GPIO_10 */
    NULL,  /*  10 - GPIO_11 */
                    
    /* pins 11-20 */
    NULL,  /*  11 - GPIO_22 */
    NULL,  /*  12 - GPIO_01 */
    NULL,  /*  13 - GPIO_25 */
    NULL,  /*  14 - GPIO_15 */
    NULL,  /*  15 - GPIO_16 */
    NULL,  /*  16 - RESET */
    NULL,  /*  17 - GPIO_31 */
    NULL,  /*  18 - GPIO_17 */
    NULL,  /*  19 - GPIO_28 */
    NULL,  /*  20 - GND */
                    
    /* pins 21-30 */
    NULL,  /*  21 - 5V */
    NULL,  /*  22 - GND */
    NULL,  /*  23 - GPIO_02 */
    NULL,  /*  24 - GPIO_05 */
    NULL,  /*  25 - GPIO_03 */
    NULL,  /*  26 - GPIO_04 */
    NULL,  /*  27 - GPIO_08 */
    NULL,  /*  28 - GPIO_30 */
    NULL,  /*  29 - GPIO_09 */
    NULL,  /*  30 - GPIO_00 */
                    
    /* pins 31-40 */
    NULL,  /*  31 - GPIO_24 */
    NULL,  /*  32 - GPIO_23 */
    NULL,  /*  33 - GPIO_05 */
    NULL,  /*  34 - GPIO_07 */
    NULL,  /*  35 - GPIO_28 */
    NULL,  /*  36 - GPIO_25 */
    NULL,  /*  37 - GPIO_09 */
    NULL,  /*  38 - GPIO_24 */
    NULL,  /*  39 - GPIO_10 */
    NULL,  /*  40 - GPIO_11 */
};

/* User requested callback functions for the GPIO input signals */

/* The device-specific GPIO_config structure */
const GPIOCC3200_Config GPIOCC3200_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = ~0
};

/*
 *  ======== Board_initGPIO ========
 */
void Board_initGPIO(void)
{
    /* set up initial GPIO pin configurations */
    GPIO_init();
}

/*
 * ======== I2C ========
 */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC3200.h>
#include <driverlib/i2c.h>

I2CCC3200_Object i2cCC3200Objects[Board_I2CCOUNT];

/* I2C configuration structure */
const I2CCC3200_HWAttrs i2cCC3200HWAttrs[Board_I2CCOUNT] = {
    {
        .baseAddr = I2CA0_BASE, 
        .intNum = INT_I2CA0,
        .intPriority = ~(0), 
    }
};

const I2C_Config I2C_config[] = {
    {&I2CCC3200_fxnTable, &i2cCC3200Objects[0], &i2cCC3200HWAttrs[0]},
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_initI2C ========
 */
void Board_initI2C(void)
{
    I2C_init();
}

/*
 * ======== PWM driver ========
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC3200.h>
#include <driverlib/timer.h>

PWMTimerCC3200_Object pwmCC3200Objects[Board_PWMCOUNT];

const PWMTimerCC3200_HWAttrs pwmCC3200HWAttrs[Board_PWMCOUNT] = {
    {TIMERA0_BASE, TIMER_A},
    {TIMERA0_BASE, TIMER_B},
    {TIMERA1_BASE, TIMER_A},
    {TIMERA1_BASE, TIMER_B},
    {TIMERA2_BASE, TIMER_A},
    {TIMERA2_BASE, TIMER_B},
    {TIMERA3_BASE, TIMER_A},
    {TIMERA3_BASE, TIMER_B}
};

const PWM_Config PWM_config[] = {
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[0], &pwmCC3200HWAttrs[0]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[1], &pwmCC3200HWAttrs[1]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[2], &pwmCC3200HWAttrs[2]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[3], &pwmCC3200HWAttrs[3]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[4], &pwmCC3200HWAttrs[4]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[5], &pwmCC3200HWAttrs[5]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[6], &pwmCC3200HWAttrs[6]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[7], &pwmCC3200HWAttrs[7]},
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_initPWM ========
 */
void Board_initPWM(void)
{
    PWM_init();
}

/*
 * ======== SPI driver ========
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC3200DMA.h>
#include <driverlib/spi.h>

static SPICC3200DMA_Object SPICC3200DMAobjects[Board_SPICOUNT];

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiCC3200DMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
static uint32_t spiCC3200DMAscratchBuf[Board_SPICOUNT];

/* SPI configuration structure */
static const SPICC3200DMA_HWAttrs SPICC3200DMAHWAttrs[Board_SPICOUNT] = {
    {
        GSPI_BASE,
        INT_GSPI,
        0xC0,       /* make SPI interrupt one priority higher than default */
        PRCM_GSPI,
        SPI_HW_CTRL_CS,
        SPI_CS_ACTIVELOW,
        SPI_4PIN_MODE,
        SPI_TURBO_OFF,
        &spiCC3200DMAscratchBuf[0],
        0,
        UDMA_CH6_GSPI_RX,
        UDMA_CH7_GSPI_TX,
    },
};

const SPI_Config SPI_config[] = {
    {&SPICC3200DMA_fxnTable, &SPICC3200DMAobjects[0], &SPICC3200DMAHWAttrs[0]},
    {NULL, NULL, NULL},
};

/*
 *  ======== Board_initSPI ========
 */
void Board_initSPI(void)
{
    Board_initDMA();
    SPI_init();
}

/*
 * ======== UART driver ========
 */
#include <driverlib/uart.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC3200.h>

UARTCC3200_Object uartCC3200Objects[Board_UARTCOUNT];

unsigned char uartCC3200RingBuffer0[32];
unsigned char uartCC3200RingBuffer1[32];

/* UART configuration structure */
const UARTCC3200_HWAttrs uartCC3200HWAttrs[Board_UARTCOUNT] = {
    {
        .baseAddr = UARTA1_BASE, 
        .intNum = INT_UARTA1,
        .intPriority = ~(0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartCC3200RingBuffer0,
        .ringBufSize = sizeof(uartCC3200RingBuffer0),
    },
    {
        .baseAddr = UARTA0_BASE, 
        .intNum = INT_UARTA0,
        .intPriority = ~(0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartCC3200RingBuffer1,
        .ringBufSize = sizeof(uartCC3200RingBuffer1)
    }
};

const UART_Config UART_config[] = {
    {
        &UARTCC3200_fxnTable,
        &uartCC3200Objects[0],
        &uartCC3200HWAttrs[0]
    },
    {
        &UARTCC3200_fxnTable,
        &uartCC3200Objects[1],
        &uartCC3200HWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_initUART ========
 */
void Board_initUART(void)
{
    UART_init();
}

/* 
 *  =============================== Power ===============================
 */

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC3200.h>

/*
 *  ======== PowerCC3200_config ========
 *  In this configuration, Power management is disabled since runPolicy
 *  is set to 0.  Power management can be enabled from main() by calling
 *  Power_enablePolicy(), or by changing runPolicy to 1 in this structure.
 */
const PowerCC3200_Config PowerCC3200_config = {
    &PowerCC3200_initPolicy,   /* policyInitFxn */
    &PowerCC3200_sleepPolicy,  /* policyFxn */
    NULL,                      /* enterLPDSHookFxn */
    NULL,                      /* resumeLPDSHookFxn */
    0,                         /* enablePolicy */
    1,                         /* enableGPIOWakeupLPDS */
    0,                         /* enableGPIOWakeupShutdown */
    0,                         /* enableNetworkWakeupLPDS */
    PRCM_LPDS_GPIO13,          /* wakeupGPIOSourceLPDS */
    PRCM_LPDS_FALL_EDGE,       /* wakeupGPIOTypeLPDS */
    0,                         /* wakeupGPIOSourceShutdown */
    0,                         /* wakeupGPIOTypeShutdown */
    PRCM_SRAM_COL_1|PRCM_SRAM_COL_2|PRCM_SRAM_COL_3|PRCM_SRAM_COL_4
                               /* ramRetentionMaskLPDS */
};

/*
 *  ======== Board_initPower ========
 */
void Board_initPower(void)
{
    Power_init();
}

#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/interfaces/IHwi.h>

/*
 *  ======== Board_init ========
 *  Initialialize the ti.platforms.tink2 hardware
 */
void Board_init(void)
{
    /* driver-independent initialization */
    Board_initGeneral();

    /* driver-specific initialization */
    Board_initGPIO();
    Board_initPWM();
    Board_initPower();
}
