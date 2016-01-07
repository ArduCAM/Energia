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

#define ARDUINO_MAIN
#include "wiring_private.h"
#include <rom.h>
#include <rom_map.h>
#include <timer_a.h>
#include <adc14.h>
#include <ref_a.h>
#include <pmap.h>
#include <gpio.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>

#define NOT_IN_USE 0

/*
 * analogWrite() support
 */

extern PWM_Config PWM_config[];

/* Mappable PWM Timer capture pins */
const uint8_t mappable_pwms[] = {
    PM_TA0CCR1A,
    PM_TA0CCR2A,
    PM_TA0CCR3A,
    PM_TA0CCR4A,
    PM_TA1CCR1A,
    PM_TA1CCR2A,
    PM_TA1CCR3A,
    PM_TA1CCR4A,
};

/* port number to PXMAP translation */
const uint8_t pxmap[] = {
    0,
    P1MAP,
    P2MAP,
    P3MAP,
    P4MAP,
    P5MAP,
    P6MAP,
    P7MAP,
};

/* Current PWM timer GPIO mappings */
uint16_t used_pwm_port_pins[] = {
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
};

/*
 * For the MSP432, the timers used for PWM are clocked at 12MHz.
 * The period is set to 2.04ms in the PWM_open() call.
 * The PWM objects are configured for PWM_DUTY_COUNTS mode to minimize
 * the PWM_setDuty() processing overhead.
 * The 2.04ms period yields a period count of 24480.
 * The Arduino analogWrite() API takes a value of 0-255 for the duty cycle.
 * The PWM scale factor is then 24480 / 255 = 96
 */

#define PWM_SCALE_FACTOR 24480/255

void analogWrite(uint8_t pin, int val)
{
    uint16_t pwmIndex, pinId, pinNum;
    uint_fast8_t port;
    uint_fast16_t pinMask;
    uint32_t hwiKey;

    hwiKey = Hwi_disable();

    if (digital_pin_to_pin_function[pin] == PIN_FUNC_ANALOG_OUTPUT) {
        pwmIndex = digital_pin_to_pwm_index[pin];
    }
    else {
        /* re-configure pin if possible */
        PWM_Params params;

        /*
         * The pwmIndex fetched from the pin_to_pwm_index[] table
         * is either an actual index into the PWM instance table
         * if the pin has already been mapped to a PWM resource,
         * or a mappable port/pin ID, or NOT_MAPPABLE.
         */
        pinId = digital_pin_to_pwm_index[pin];

        if (pinId == NOT_MAPPABLE) {
            Hwi_restore(hwiKey);
            return; /* can't get there from here */
        }

        /* find an unused PWM resource and port map it */
        for (pwmIndex = 0; pwmIndex < 8; pwmIndex++) {
            if (used_pwm_port_pins[pwmIndex] == NOT_IN_USE) {
                /* remember which pinId is being used by this PWM resource */
                used_pwm_port_pins[pwmIndex] = pinId; /* save port/pin info */
                /* remember which PWM resource is being used by this pin */
                digital_pin_to_pwm_index[pin] = pwmIndex; /* save pwm index */
                break;
            }
        }

        if (pwmIndex > 7) {
            Hwi_restore(hwiKey);
            return; /* no unused PWM ports */
        }

        PWM_Params_init(&params);

        /* Open the PWM port */
        params.period = 2040; /* arduino period is 2.04ms (490Hz) */
        params.dutyMode = PWM_DUTY_COUNTS;
        PWM_open(pwmIndex, &params);

        port = pinId >> 8;
        pinMask = pinId & 0xff;

        /* derive pinNum from pinMask */
        pinNum = 0;
        while (((1 << pinNum) & pinMask) == 0) pinNum++;

        /* Get write-access to port mapping registers: */
        PMAP->rKEYID = PMAP_KEYID_VAL;

        /* Enable reconfiguration during runtime */
        PMAP->rCTL.r = (PMAP->rCTL.r & ~PMAPRECFG) | PMAP_ENABLE_RECONFIGURATION;

        /* Configure Port Mapping for this pin: */
        HWREG8(PMAP_BASE + pinNum + pxmap[port]) = mappable_pwms[pwmIndex];

        /* Disable write-access to port mapping registers: */
        PMAP->rKEYID = 0;

        /* Enable PWM output on GPIO pins */
        MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(port, pinMask,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);

        digital_pin_to_pin_function[pin] = PIN_FUNC_ANALOG_OUTPUT;
    }

    Hwi_restore(hwiKey);

    PWM_setDuty((PWM_Handle)&(PWM_config[pwmIndex]), (val * PWM_SCALE_FACTOR));
}

/*
 * This internal API is used to de-configure a pin that has been
 * put in analogWrite() mode.
 *
 * It will free up the pin's PWM resource after
 * it is no longer being used to support analogWrite() on a different
 * pin. It is called by pinMap() when a pin's function is being modified.
 */
void stopAnalogWrite(uint8_t pin)
{
    uint16_t pwmIndex = digital_pin_to_pwm_index[pin];
    uint_fast8_t port;
    uint_fast16_t pinMask;
    uint16_t pinNum;
    uint32_t hwiKey;

    /* undo dynamic port mapping plumbing */
    port = used_pwm_port_pins[pwmIndex] >> 8;
    pinMask = used_pwm_port_pins[pwmIndex] & 0xff;

    /* derive pinNum from pinMask */
    pinNum = 0;
    while (((1 << pinNum) & pinMask) == 0) pinNum++;
    /* the following code was extracted from PMAP_configurePort() */

    //Get write-access to port mapping registers:
    PMAP->rKEYID = PMAP_KEYID_VAL;

    //Enable reconfiguration during runtime
    PMAP->rCTL.r = (PMAP->rCTL.r & ~PMAPRECFG) | PMAP_ENABLE_RECONFIGURATION;

    //Undo Port Mapping for this pin:
    HWREG8(PMAP_BASE + pinNum + pxmap[port]) = PM_NONE;

    //Disable write-access to port mapping registers:
    PMAP->rKEYID = 0;

    /* Close PWM port */
    PWM_close((PWM_Handle)&(PWM_config[pwmIndex]));

    hwiKey = Hwi_disable();

    /* restore pin table entry with port/pin info */
    digital_pin_to_pwm_index[pin] = used_pwm_port_pins[pwmIndex];

    /* free up pwm resource */
    used_pwm_port_pins[pwmIndex] = NOT_IN_USE;

    Hwi_restore(hwiKey);
}

/*
 * analogRead() support
 */

static int8_t analogResolution = 10;
static bool adc_module_enabled = false;

static const uint16_t adc_to_port_pin[] = {
    GPIOMSP432_P5_5,  /* A0 */
    GPIOMSP432_P5_4,  /* A1 */
    GPIOMSP432_P5_3,  /* A2 */
    GPIOMSP432_P5_2,  /* A3 */
    GPIOMSP432_P5_1,  /* A4 */
    GPIOMSP432_P5_0,  /* A5 */
    GPIOMSP432_P4_7,  /* A6 */
    GPIOMSP432_P4_6,  /* A7 */

    GPIOMSP432_P4_5,  /* A8 */
    GPIOMSP432_P4_4,  /* A9 */
    GPIOMSP432_P4_3,  /* A10 */
    GPIOMSP432_P4_2,  /* A11 */
    GPIOMSP432_P4_1,  /* A12 */
    GPIOMSP432_P4_0,  /* A13 */
    GPIOMSP432_P6_1,  /* A14 */
    GPIOMSP432_P6_0,  /* A15 */

    GPIOMSP432_P9_1,  /* A16 */
    GPIOMSP432_P9_0,  /* A17 */
    GPIOMSP432_P8_7,  /* A18 */
    GPIOMSP432_P8_6,  /* A19 */
    GPIOMSP432_P8_5,  /* A20 */
    GPIOMSP432_P8_4,  /* A21 */
    GPIOMSP432_P8_3,  /* A22 */
    GPIOMSP432_P8_2,  /* A23 */
};

/*
 * \brief           Reads an analog value from the pin specified.
 * \param[in] pin   The pin number to read from.
 * \return          A 16-bit integer containing a 12-bit sample from the ADC.
 */
uint16_t analogRead(uint8_t pin)
{
    uint8_t adcIndex = digital_pin_to_adc_index[pin];
    uint32_t adcMem, adcInt;
    uint16_t sample = 0;
    uint64_t status;
    uint32_t hwiKey;

    if (adcIndex == NOT_ON_ADC) return (0);

    adcMem = 1 << adcIndex;
    adcInt = 1 << adcIndex;

    hwiKey = Hwi_disable();

    /* re-configure pin if necessary */
    if (digital_pin_to_pin_function[pin] != PIN_FUNC_ANALOG_INPUT) {
        uint_fast8_t port;
        uint_fast16_t pinMask;

        digital_pin_to_pin_function[pin] = PIN_FUNC_ANALOG_INPUT;

        /* initialize top level ADC module if it hasn't been already */
        if (adc_module_enabled == false) {
            adc_module_enabled = true;

            /* Initializing ADC (MCLK/1/1) */
            MAP_ADC14_enableModule();
            MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK,
                             ADC_PREDIVIDER_1,
                             ADC_DIVIDER_1,
                             0);
            /* Setting reference voltage to 2.5 */
            MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
            MAP_REF_A_enableReferenceVoltage();

            analogReadResolution(analogResolution);
        }

        port = adc_to_port_pin[adcIndex] >> 8;
        pinMask = adc_to_port_pin[adcIndex] & 0xff;

        /* Setting up GPIO pins as analog inputs (and references) */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(port, pinMask,
                                        GPIO_TERTIARY_MODULE_FUNCTION);
    }

    /* minimize latency by re-enabling ints temporarily */
    Hwi_restore(hwiKey);

    /* Sadly, I think the following code must all be done thread safely */
    hwiKey = Hwi_disable();

    /* stop all current conversions */
    MAP_ADC14_disableConversion();

    /* Configuring ADC Memory in single conversion mode
     * with use of internal VSS as references */
    MAP_ADC14_configureSingleSampleMode(adcMem, false);
    MAP_ADC14_configureConversionMemory(adcMem,
                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                    adcIndex, false);

    /* Enabling sample timer in auto iteration mode and interrupts */
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* clear out any stale conversions */
    status = MAP_ADC14_getInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* start new conversion */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    status = MAP_ADC14_getInterruptStatus();

    while ((adcInt & status) == 0) {
        status = MAP_ADC14_getInterruptStatus();
    }

    /* clear out last sample */
    MAP_ADC14_clearInterruptFlag(adcInt);
    sample = MAP_ADC14_getResult(adcMem);

    Hwi_restore(hwiKey);

    return (sample);
}

/*
 * This internal API is used to de-configure a pin that has been
 * put in analogRead() mode.
 *
 * It is called by pinMap() when a pin's function is
 * being modified.
 */
void stopAnalogRead(uint8_t pin)
{
    uint8_t adcIndex = digital_pin_to_adc_index[pin];
    uint_fast8_t port;
    uint_fast16_t pinMask;

    port = adc_to_port_pin[adcIndex] >> 8;
    pinMask = adc_to_port_pin[adcIndex] & 0xff;

    /* Place Pin in NORMAL GPIO mode */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(port, pinMask,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);
}

/*
 * \brief sets the number of bits to shift the value read by ADCFIFORead()
 */
void analogReadResolution(uint16_t bits)
{
    uint32_t adcBits;

    switch (bits) {
        case 8:
            adcBits = ADC_8BIT;
            break;
        case 10:
            adcBits = ADC_10BIT;
            break;
        case 12:
            adcBits = ADC_12BIT;
            break;
        case 14:
            adcBits = ADC_14BIT;
            break;
    }
    analogResolution = bits;

    MAP_ADC14_setResolution(adcBits);
}
