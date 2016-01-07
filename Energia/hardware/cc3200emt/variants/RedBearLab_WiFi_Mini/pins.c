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

#include <ti/runtime/wiring/cc3200/wiring_private.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>

#include <driverlib/pin.h>
#include <driverlib/timer.h>
#include "driverlib/adc.h"

uint8_t digital_pin_to_pin_function[] = {
    PIN_FUNC_UNUSED,	// D0 - IO02
    PIN_FUNC_UNUSED,	// D1 - IO01
    PIN_FUNC_UNUSED,	// D2 - IO06
    PIN_FUNC_UNUSED,	// D3 - IO07
    PIN_FUNC_UNUSED,	// D4 - IO00    
    PIN_FUNC_UNUSED,	// D5 - IO09
    PIN_FUNC_UNUSED,	// D6 - IO24
    PIN_FUNC_UNUSED,	// D7 - IO28
    PIN_FUNC_UNUSED,	// D8 - IO29
    PIN_FUNC_UNUSED,	// D9 - IO23
    PIN_FUNC_UNUSED,	// D10 - IO22
    PIN_FUNC_UNUSED,	// D11 - IO13   
    PIN_FUNC_UNUSED,	// D12 - IO17
    PIN_FUNC_UNUSED,	// D13 - IO30  
    PIN_FUNC_UNUSED,	// D14 - IO11
    PIN_FUNC_UNUSED,	// D15 - IO10
    PIN_FUNC_UNUSED,	// D16/A0 - IO02
    PIN_FUNC_UNUSED,	// D17/A1 - IO05
    PIN_FUNC_UNUSED,	// D18/A2 - IO04
    PIN_FUNC_UNUSED,	// D19/A3 - IO03
    PIN_FUNC_UNUSED,	// D20 - IO08
    PIN_FUNC_UNUSED,	// D21 - IO12
    PIN_FUNC_UNUSED,	// D22 - IO16
    PIN_FUNC_UNUSED,	// D23 - IO15
    PIN_FUNC_UNUSED,	// D24 - IO14 
};

const uint8_t digital_pin_to_timer[] = {
    NOT_ON_TIMER,     // D0 - IO02
    NOT_ON_TIMER,     // D1 - IO01
    NOT_ON_TIMER,     // D2 - IO06
    NOT_ON_TIMER,     // D3 - IO07
    NOT_ON_TIMER,     // D4 - IO00    
    TIMERA2B,         // D5 - IO09
    TIMERA0A,         // D6 - IO24
    NOT_ON_TIMER,     // D7 - IO28
    NOT_ON_TIMER,     // D8 - IO29
    NOT_ON_TIMER,     // D9 - IO23
    NOT_ON_TIMER,     // D10 - IO22
    NOT_ON_TIMER,     // D11 - IO13   
    NOT_ON_TIMER,     // D12 - IO17
    NOT_ON_TIMER,     // D13 - IO30  
    TIMERA3B,         // D14 - IO11
    TIMERA3A,         // D15 - IO10
    NOT_ON_TIMER,     // D16/A0 - IO02
    NOT_ON_TIMER,     // D17/A1 - IO05
    NOT_ON_TIMER,     // D18/A2 - IO04
    NOT_ON_TIMER,     // D19/A3 - IO03  
    NOT_ON_TIMER,     // D20 - IO08
    NOT_ON_TIMER,     // D21 - IO12
    NOT_ON_TIMER,     // D22 - IO16
    NOT_ON_TIMER,     // D23 - IO15
    NOT_ON_TIMER,     // D24 - IO14 
};

const uint16_t digital_pin_to_pin_num[] = {
    PIN_57,         // D0 - IO02
    PIN_55,         // D1 - IO01
    PIN_61,         // D2 - IO06
    PIN_62,         // D3 - IO07
    PIN_50,         // D4 - IO00
    PIN_64,         // D5 - IO09
    PIN_17,         // D6 - IO24
    PIN_18,         // D7 - IO28
    PIN_20,         // D8 - IO29
    PIN_16,         // D9 - IO23
    PIN_15,         // D10 - IO22
    PIN_04,         // D11 - IO13
    PIN_08,         // D12 - IO17
    PIN_53,         // D13 - IO30
    PIN_02,         // D14 - IO11
    PIN_01,         // D15 - IO10
    PIN_57,         // D16/A0 - IO02
    PIN_60,         // D17/A1 - IO05
    PIN_59,         // D18/A2 - IO04
    PIN_58,         // D19/A3 - IO03    
    PIN_63,         // D20 - IO08
    PIN_03,         // D21 - IO12
    PIN_07,         // D22 - IO16
    PIN_06,         // D23 - IO15
    PIN_05,         // D24 - IO14
};

