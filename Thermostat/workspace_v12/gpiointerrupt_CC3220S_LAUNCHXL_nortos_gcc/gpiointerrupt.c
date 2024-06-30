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
 *  Main application file for handling GPIO interrupts, I2C communication,
 *  UART communication, and Timer functionalities in the thermostat project.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Definitions for task periods and pin assignments */
#define DISPLAY(length) UART2_write(uart2, output, length, NULL)
#define timerPeriod 200000 // 200 ms period for the main timer
#define checkButtonPeriod 200 // 200 ms for button checking task
#define checkTemperaturePeriod 500 // 500 ms for temperature reading task
#define updateHeatModeAndServerPeriod 1000 // 1000 ms for updating heat mode and server
#define BUTTON_INC_PIN CONFIG_GPIO_BUTTON_0 // Button for increasing temperature
#define BUTTON_DEC_PIN CONFIG_GPIO_BUTTON_1 // Button for decreasing temperature
#define LED_PIN CONFIG_GPIO_LED_0 // LED representing heater state
#define numTasks 3 // Number of tasks in the scheduler

/*
 *  ======== Task Type ========
 *  Defines structure for the task type in the scheduler.
 */
typedef struct task {
    int state;                    // Current state of the task
    unsigned long period;         // Rate at which the task should tick
    unsigned long elapsedTime;    // Time since task's previous tick
    int (*tickFunction)(int);     // Function to call for task's tick
} task;

/*
 *  ======== Driver Handles ========
 *  Handles for various drivers used in the project.
 */
I2C_Handle i2c;         // Handle for I2C driver
Timer_Handle timer0;    // Handle for Timer driver
UART2_Handle uart2;     // Handle for UART2 driver

/*
 *  ======== Global Variables ========
 *  Global variables for UART2, I2C, and application state.
 */
// UART2 global variables
char output[64]; // Buffer for UART output

// I2C global variables
static const struct {
    uint8_t address;   // I2C address of the sensor
    uint8_t resultReg; // Register to read from
    char *id;          // Sensor identifier
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1]; // Transmission buffer for I2C
uint8_t rxBuffer[2]; // Reception buffer for I2C
I2C_Transaction i2cTransaction; // I2C transaction structure

// Timer global variables
volatile unsigned char TimerFlag = 0; // Flag set by timer callback

// Thermostat global variables
enum BUTTON_STATES {INCREASE_TEMPERATURE, DECREASE_TEMPERATURE, BUTTON_INIT};
enum TEMPERATURE_SENSOR_STATES {READ_TEMPERATURE, TEMPERATURE_SENSOR_INIT};
enum HEATING_STATES {HEAT_OFF, HEAT_ON, HEAT_INIT};
enum BUTTON_STATES BUTTON_STATE; // Current state of the button
int16_t ambientTemperature = 0; // Current ambient temperature
int16_t setPoint = 28; // Desired set-point temperature
int seconds = 0; // Elapsed time in seconds

/*
 *  ======== Callback ========
 */
// GPIO button callback function to increase the thermostat set-point.
void gpioIncreaseTemperatureCallback(uint_least8_t index) {
    printf("Increase button pressed.\n");
    GPIO_toggle(LED_PIN); // Toggle LED for testing
    BUTTON_STATE = INCREASE_TEMPERATURE; // Set state to increase temperature
}

// GPIO button callback function to decrease the thermostat set-point.
void gpioDecreaseTemperatureCallback(uint_least8_t index) {
    printf("Decrease button pressed.\n");
    GPIO_toggle(LED_PIN); // Toggle LED for testing
    BUTTON_STATE = DECREASE_TEMPERATURE; // Set state to decrease temperature
}

// Timer callback function
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;  // Set flag to 1 to indicate timer is running.
}

/*
 *  ======== Initializations ========
 */
// Initialize UART2
void initUART(void) {
    UART2_Params uartParams;

    // Configure UART2 driver parameters
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_BLOCKING; // Blocking write
    uartParams.readMode = UART2_Mode_BLOCKING;  // Blocking read
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL; // Full read return
    uartParams.baudRate = 115200; // Baud rate for UART

    // Open the UART2 driver
    uart2 = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart2 == NULL) {
        /* UART2_open() failed */
        while (1);
    }
}

// Initialize I2C
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Initialize I2C driver
    I2C_init();

    // Configure I2C driver parameters
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz; // 400kHz bit rate

    // Open the I2C driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Scan through the possible sensor addresses to find the sensor
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i = 0; i < 3; ++i) {
        i2cTransaction.targetAddress = sensors[i].address; // Set target address
        txBuffer[0] = sensors[i].resultReg; // Set register address

        DISPLAY(snprintf(output, sizeof(output), "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, sizeof(output), "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, sizeof(output), "No\n\r"));
    }

    if (found) {
        DISPLAY(snprintf(output, sizeof(output), "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress)); // Detected sensor
    } else {
        DISPLAY(snprintf(output, sizeof(output), "Temperature sensor not found, contact professor\n\r"));
    }
}

// Initialize GPIO
void initGPIO(void) {
    GPIO_init(); // Initialize GPIO driver

    // Configure GPIO pins for buttons and LED
    GPIO_setConfig(BUTTON_INC_PIN, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING); // Pull-up with falling edge
    GPIO_setConfig(BUTTON_DEC_PIN, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING); // Pull-up with falling edge
    GPIO_setConfig(LED_PIN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); // LED output

    // Set callbacks for button presses
    GPIO_setCallback(BUTTON_INC_PIN, gpioIncreaseTemperatureCallback);
    GPIO_setCallback(BUTTON_DEC_PIN, gpioDecreaseTemperatureCallback);

    // Enable interrupts for buttons
    GPIO_enableInt(BUTTON_INC_PIN);
    GPIO_enableInt(BUTTON_DEC_PIN);

    BUTTON_STATE = BUTTON_INIT; // Initialize button state
}

// Initialize Timer
void initTimer(void) {
    Timer_Params params;

    // Initialize Timer driver
    Timer_init();
    Timer_Params_init(&params);
    params.period = timerPeriod; // Set period to 200 ms
    params.periodUnits = Timer_PERIOD_US; // Period specified in microseconds
    params.timerMode = Timer_CONTINUOUS_CALLBACK; // Continuous mode with callback
    params.timerCallback = timerCallback; // Timer callback function

    // Open the Timer driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        while (1) {}
    }
}

/*
 *  ======== adjustSetPointTemperature ========
 *  Adjusts the set-point temperature based on the button state.
 */
int adjustSetPointTemperature(int state) {
    switch (state) {
        case INCREASE_TEMPERATURE:
            if (setPoint < 99) { // Increase set-point temperature, max 99°C
                setPoint++;
                printf("Set-point increased to: %d\n", setPoint);
            }
            BUTTON_STATE = BUTTON_INIT; // Reset button state
            break;
        case DECREASE_TEMPERATURE:
            if (setPoint > 0) { // Decrease set-point temperature, min 0°C
                setPoint--;
                printf("Set-point decreased to: %d\n", setPoint);
            }
            BUTTON_STATE = BUTTON_INIT; // Reset button state
            break;
        default:
            printf("Invalid button state.\n");
            break;
    }

    state = BUTTON_STATE; // Return updated state
    return state;
}

/*
 *  ======== readTemp ========
 *  Reads the current temperature from the sensor and returns the reading.
 */
int16_t readTemp(void) {
    int16_t temperature = 0;
    i2cTransaction.readCount = 2; // Expect 2 bytes of data
    if (I2C_transfer(i2c, &i2cTransaction)) {
        // Combine two bytes into temperature value
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125; // Convert to degrees Celsius
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000; // Sign extend if negative
        }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Power cycle your board by unplugging USB.\n\r"));
    }
    return temperature;
}

/*
 *  ======== getAmbientTemperature ========
 *  Checks the current state to determine if the temperature should be read.
 */
int getAmbientTemperature(int state) {
    switch (state) {
        case TEMPERATURE_SENSOR_INIT:
            state = READ_TEMPERATURE; // Move to reading temperature state
            break;
        case READ_TEMPERATURE:
            ambientTemperature = readTemp(); // Read current ambient temperature
            printf("Ambient Temperature read: %d°C\n", ambientTemperature);
            break;
        default:
            printf("Invalid temperature sensor state.\n");
            break;
    }
    return state;
}

/*
 *  ======== setHeatMode ========
 *  Compares the ambient temperature to the set-point and controls the heat (LED).
 */
int setHeatMode(int state) {
    if (seconds != 0) {
        // Determine if the heat should be on or off
        if (ambientTemperature < setPoint) {
            GPIO_write(LED_PIN, CONFIG_GPIO_LED_ON); // Turn on LED (heat)
            state = HEAT_ON; // Update state to HEAT_ON
            printf("LED ON\n");
        } else {
            GPIO_write(LED_PIN, CONFIG_GPIO_LED_OFF); // Turn off LED (heat)
            state = HEAT_OFF; // Update state to HEAT_OFF
            printf("LED OFF\n");
        }
        // Format and send data via UART
        int length = snprintf(output, sizeof(output), "<%02d,%02d,%d,%04d>\n\r", ambientTemperature, setPoint, state == HEAT_ON ? 1 : 0, seconds);
        DISPLAY(length);
        printf("Ambient Temperature: %d°C, Set-point: %d°C, Heat: %d\n", ambientTemperature, setPoint, state == HEAT_ON ? 1 : 0);
    }
    seconds++; // Increment the elapsed time counter
    return state;
}

/*
 *  ======== mainThread ========
 *  Main loop for the task scheduler. Initializes all drivers and processes tasks.
 */
void *mainThread(void *arg0) {
    // Create task list with tasks
    task tasks[numTasks] = {
        { .state = BUTTON_INIT, .period = checkButtonPeriod, .elapsedTime = checkButtonPeriod, .tickFunction = &adjustSetPointTemperature },
        { .state = TEMPERATURE_SENSOR_INIT, .period = checkTemperaturePeriod, .elapsedTime = checkTemperaturePeriod, .tickFunction = &getAmbientTemperature },
        { .state = HEAT_INIT, .period = updateHeatModeAndServerPeriod, .elapsedTime = updateHeatModeAndServerPeriod, .tickFunction = &setHeatMode }
    };

    // Initialize drivers
    initUART();
    initI2C();
    initGPIO();
    initTimer();

    // Loop forever to process tasks
    while (1) {
        unsigned int i;
        for (i = 0; i < numTasks; ++i) {
            if (tasks[i].elapsedTime >= tasks[i].period) {
                tasks[i].state = tasks[i].tickFunction(tasks[i].state); // Execute task function
                tasks[i].elapsedTime = 0; // Reset elapsed time
            }
            tasks[i].elapsedTime += (timerPeriod / 1000); // Convert timer period to milliseconds
        }

        // Directly read GPIO state to detect button presses
        if (GPIO_read(BUTTON_INC_PIN) == 0) { // Increase button pressed
            printf("Increase button pressed directly detected.\n");
        }
        if (GPIO_read(BUTTON_DEC_PIN) == 0) { // Decrease button pressed
            printf("Decrease button pressed directly detected.\n");
        }

        while (!TimerFlag) {} // Wait for timer period
        TimerFlag = 0; // Clear timer flag
    }

    return (NULL);
}
