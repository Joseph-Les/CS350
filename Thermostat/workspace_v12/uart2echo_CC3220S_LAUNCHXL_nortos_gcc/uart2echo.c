/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
 *  ======== uart2echo.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* State definitions for the state machine */
typedef enum {
    STATE_IDLE,
    STATE_O,
    STATE_ON,
    STATE_F,
    STATE_OFF
} State;

/* LED Control Functions */
void led_on() {
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
}

void led_off() {
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char input;
    const char echoPrompt[] = "Echoing characters:\r\n";
    UART2_Handle uart;
    UART2_Params uartParams;
    size_t bytesRead;
    size_t bytesWritten = 0;
    uint32_t status     = UART2_STATUS_SUCCESS;

    unsigned char currentState = STATE_IDLE;

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART where the default read and write mode is BLOCKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART2_write(uart, echoPrompt, sizeof(echoPrompt), &bytesWritten);

    /* Loop forever echoing and processing state machine */
    while (1)
    {
        bytesRead = 0;
        while (bytesRead == 0)
        {
            status = UART2_read(uart, &input, 1, &bytesRead);

            if (status != UART2_STATUS_SUCCESS)
            {
                /* UART2_read() failed */
                while (1) {}
            }
        }

        /* Echo the character back */
        bytesWritten = 0;
        while (bytesWritten == 0)
        {
            status = UART2_write(uart, &input, 1, &bytesWritten);

            if (status != UART2_STATUS_SUCCESS)
            {
                /* UART2_write() failed */
                while (1) {}
            }
        }

        /* State machine logic */
        switch (currentState)
        {
            case STATE_IDLE:
                // In the idle state, if the input is 'O', transition to STATE_O.
                if (input == 'O') {
                    currentState = STATE_O;
                }
                break;

            case STATE_O:
                // In the STATE_O state, if the input is 'N', transition to STATE_ON and turn the LED on.
                // If the input is 'F', transition to STATE_F.
                // For any other input, transition back to STATE_IDLE.
                if (input == 'N') {
                    currentState = STATE_ON;
                    led_on();
                } else if (input == 'F') {
                    currentState = STATE_F;
                } else {
                    currentState = STATE_IDLE;
                }
                break;

            case STATE_ON:
                // In the STATE_ON state, if the input is 'O', transition to STATE_O.
                if (input == 'O') {
                    currentState = STATE_O;
                }
                break;

            case STATE_F:
                // In the STATE_F state, if the input is 'F', transition to STATE_OFF and turn the LED off.
                // For any other input, transition back to STATE_IDLE.
                if (input == 'F') {
                    currentState = STATE_OFF;
                    led_off();
                } else {
                    currentState = STATE_IDLE;
                }
                break;

            case STATE_OFF:
                // In the STATE_OFF state, if the input is 'O', transition to STATE_O.
                if (input == 'O') {
                    currentState = STATE_O;
                }
                break;

            default:
                // If the current state is unrecognized, transition to STATE_IDLE.
                currentState = STATE_IDLE;
                break;
        }

    }
}
