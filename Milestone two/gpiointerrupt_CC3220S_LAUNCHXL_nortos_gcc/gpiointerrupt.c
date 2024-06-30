/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Morse code message states */
enum CURRENT_MESSAGES {SOS, OK_Signal} CURRENT_MESSAGE, BUTTON_STATE;

/* LED code states */
enum LED_STATES {RED, GREEN, OFF} LED_STATE;

/* Morse messages encoded as LED states */
/* SOS pattern in Morse code */
enum LED_STATES sosMessage[] = {
    /* S: dot-dot-dot */
    RED, OFF,
    RED, OFF,
    RED, OFF, OFF, OFF,
    /* O: dash-dash-dash */
    GREEN, GREEN, GREEN, OFF,
    GREEN, GREEN, GREEN, OFF,
    GREEN, GREEN, GREEN, OFF, OFF, OFF,
    /* S: dot-dot-dot */
    RED, OFF,
    RED, OFF,
    RED,
    /* Longer pause between repetitions of SOS */
    OFF, OFF, OFF, OFF, OFF, OFF, OFF
};

/* OK pattern in Morse code */
enum LED_STATES okMessage[] = {
    /* O: dash-dash-dash */
    GREEN, GREEN, GREEN, OFF,
    GREEN, GREEN, GREEN, OFF,
    GREEN, GREEN, GREEN, OFF, OFF, OFF,
    /* K: dash-dot-dash */
    GREEN, GREEN, GREEN, OFF,
    RED, OFF,
    GREEN, GREEN, GREEN,
    /* Longer pause between repetitions of OK */
    OFF, OFF, OFF, OFF, OFF, OFF, OFF
};

/* Counter to keep track of current LED state in Morse message */
unsigned int messageCounter = 0;

/*
 *  ======== setMorseLEDs ========
 *  Function to set the LEDs according to the current Morse code state.
 */
void setMorseLEDs() {
    switch(LED_STATE) {
        case RED:
            /* Turn on red LED, turn off green LED */
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_OFF);
            break;
        case GREEN:
            /* Turn off red LED, turn on green LED */
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            break;
        case OFF:
            /* Turn off both LEDs */
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_OFF);
            break;
        default:
            break;
    }
}

/*
 *  ======== timerCallback ========
 *  Callback function for timer interrupts to progress through Morse messages.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    switch(CURRENT_MESSAGE) {
        case SOS:
            /* Set LED state for current SOS message index */
            LED_STATE = sosMessage[messageCounter];
            setMorseLEDs();
            messageCounter++;

            /* Reset message counter and switch to next message if SOS message ends */
            if(messageCounter == (sizeof(sosMessage) / sizeof(sosMessage[0]))) {
                CURRENT_MESSAGE = BUTTON_STATE;
                messageCounter = 0;
            }
            break;
        case OK_Signal:
            /* Set LED state for current OK message index */
            LED_STATE = okMessage[messageCounter];
            setMorseLEDs();
            messageCounter++;

            /* Reset message counter and switch to next message if OK message ends */
            if(messageCounter == (sizeof(okMessage) / sizeof(okMessage[0]))) {
                CURRENT_MESSAGE = BUTTON_STATE;
                messageCounter = 0;
            }
            break;
        default:
            break;
    }
}

/*
 *  ======== initTimer ========
 *  Initialize timer interrupt to call timerCallback periodically.
 */
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000; // Timer period in microseconds
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if(timer0 == NULL) {
        /* Failed to initialize timer */
        while (1) {}
    }

    if(Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== gpioButtonCallback ========
 *  Callback function for GPIO interrupts triggered by button presses.
 */
void gpioButtonCallback(uint_least8_t index)
{
    switch(BUTTON_STATE) {
        case SOS:
            /* Switch to OK signal state on button press */
            BUTTON_STATE = OK_Signal;
            break;
        case OK_Signal:
            /* Switch to SOS signal state on button press */
            BUTTON_STATE = SOS;
            break;
        default:
            break;
    }
}

/*
 *  ======== mainThread ========
 *  Main function for initializing GPIO, timer, and handling button press interrupts.
 */
void *mainThread(void *arg0)
{
    /* Initialize GPIO and Timer drivers */
    GPIO_init();
    initTimer();

    /* Configure LED pins as outputs and button pin as input with pull-up and falling edge interrupt */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Ensure LEDs are off at start */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_OFF);

    /* Initialize button and message states */
    BUTTON_STATE = SOS;
    CURRENT_MESSAGE = BUTTON_STATE;

    /* Install callback for button press interrupts */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonCallback);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If a second button is available, configure it similarly.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure additional button pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install callback for additional button */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonCallback);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    return (NULL);
}
