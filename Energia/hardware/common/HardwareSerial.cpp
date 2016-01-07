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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "wiring_private.h"
#include "HardwareSerial.h"

#define RX_BUFFER_EMPTY   (rxReadIndex == rxWriteIndex)
#define RX_BUFFER_FULL    (((rxWriteIndex + 1) % SERIAL_BUFFER_SIZE) == rxReadIndex)

void uartReadCallback(UART_Handle uart, void *buf, size_t count);
void uartReadCallback1(UART_Handle uart, void *buf, size_t count);

HardwareSerial::HardwareSerial(void)
{
    init(0);
}

HardwareSerial::HardwareSerial(unsigned long module)
{
    init(module);
}

/*
 * Private Methods
 */
void HardwareSerial::init(unsigned long module)
{
    rxWriteIndex = 0;
    rxReadIndex = 0;

    uartModule = module;
    begun = false;
}

void HardwareSerial::flushAll(void)
{
}

/*
 * Public Methods
 */
void HardwareSerial::begin(unsigned long baud)
{
    UART_Params uartParams;

    if (begun == TRUE) return;

    Semaphore_construct(&rxSemaphore, 0, NULL);

    Board_initUART();

    baudRate = baud;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = uartModule == 0 ? uartReadCallback : uartReadCallback1;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = baud;

    uart = UART_open(uartModule, &uartParams);

    if (uart != NULL) {
        GateMutex_construct(&gate, NULL);

        /* start the read process */
        UART_read(uart, &rxBuffer[rxWriteIndex], 1);

        begun = TRUE;
    }
}

void HardwareSerial::setModule(unsigned long module)
{
    /* Change which pins UART is on */
    /* need to implement */
}

void HardwareSerial::setPins(unsigned long pins)
{
    /* almost same functionality as above */
}

void HardwareSerial::seize(void)
{
    GateMutex_enter(GateMutex_handle(&gate));
}

void HardwareSerial::release(void)
{
    GateMutex_leave(GateMutex_handle(&gate), 0);
}

void HardwareSerial::end(void)
{
    begun = false;
    UART_close(uart);
}

int HardwareSerial::available(void)
{
    if (RX_BUFFER_EMPTY) {
        /* kick off another character read operation */
        UART_read(uart, &rxBuffer[rxWriteIndex], 1);
    }

    return (rxWriteIndex >= rxReadIndex) ?
        (rxWriteIndex - rxReadIndex) : SERIAL_BUFFER_SIZE - (rxReadIndex - rxWriteIndex);
}

int HardwareSerial::peek(void)
{
    unsigned char cChar = 0;

    if (available() == 0) {
        return (-1);
    }

    /* Read a character from the buffer. */
    cChar = rxBuffer[rxReadIndex];

    /* Return the character to the caller. */
    return (cChar);
}

int HardwareSerial::read(void)
{
    int iChar;
    unsigned int hwiKey;

    while ((iChar = peek()) < 0) {
        Semaphore_pend(Semaphore_handle(&rxSemaphore), BIOS_WAIT_FOREVER);
    }

    hwiKey = Hwi_disable();

    rxReadIndex = ((rxReadIndex) + 1) % SERIAL_BUFFER_SIZE;

    Hwi_restore(hwiKey);

    return (iChar);
}

void HardwareSerial::flush()
{
}

size_t HardwareSerial::write(uint8_t c)
{
    UART_write(uart,(char*)&c,1);
    return (1);
}

void HardwareSerial::readCallback(UART_Handle uart, void *buf, size_t count)
{
    uint8_t volatile full = RX_BUFFER_FULL;

    rxWriteIndex = ((rxWriteIndex) + 1) % SERIAL_BUFFER_SIZE;

    if (full) {
        rxReadIndex = ((rxReadIndex) + 1) % SERIAL_BUFFER_SIZE;
    }

    Semaphore_post(Semaphore_handle(&rxSemaphore));
}

/* C type function to call the thing above */
void uartReadCallback(UART_Handle uart, void *buf, size_t count)
{
    Serial.readCallback(uart, buf, count);
}

void uartReadCallback1(UART_Handle uart, void *buf, size_t count)
{
    Serial1.readCallback(uart, buf, count);
}

void serialEvent() __attribute__((weak));
void serialEvent() { }

void serialEventRun(void)
{
    if (Serial.available()) {
        serialEvent();
    }
}

void serialEventRun1(void)
{
    if (Serial1.available()) {
        serialEvent();
    }
}

HardwareSerial Serial(0);
HardwareSerial Serial1(1);
