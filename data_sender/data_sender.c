/******************************************************************************

 @file  data_sender.c

 @brief main entry of the data sender

 Target Device: CC2650

 ******************************************************************************
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 *  ======== dataCollector.c ========
 */

/*********************************************************************
 * INCLUDES
 */

/* XDCtools Header files */
//#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* batmon */
#include <driverlib/aon_batmon.h>

//#include <stdio.h>
//#include <math.h>
//#include <limits.h>

/* Example/Board Header files */
#include "Board.h"


/*********************************************************************
 * CONSTANTS
 */

#define TASK_STACK_SIZE     768

#define BAUD_RATE           115200

#define MSG_LEN             8
#define TEMP_SIGN_IDX       4
#define TEMP_FIRST_IDX      5
#define TEMP_LAST_IDX       7
#define TEMP_POS_SIGN       '+'
#define TEMP_NEG_SIGN       '-'
#define ZERO_CHAR           '0'
#define SAMPLING_INTERVAL   10000000 // milliseconds (20000000 microseconds = 20 seconds)


/*********************************************************************
 * LOCAL VARIABLES
 */

Task_Struct         task0Struct;
Char                task0Stack[TASK_STACK_SIZE];
Semaphore_Struct    semStruct;
Semaphore_Handle    semHandle;
Clock_Struct        clk0Struct;
int32_t             temp;

/* Pin driver handle and state structure */
static PIN_Handle   ledPinHandle;
static PIN_State    ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 is on.
 *   - LEDs Board_LED1 is off.
 */
PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */


/*********************************************************************
 * @fn      clk0Fxn
 *
 * @brief   Periodic function, which samples a new temperature
 *          measurement and toggles the red LED
 *
 */
Void clk0Fxn(UArg arg0)
{
    // sample a new measurement
    temp = AONBatMonTemperatureGetDegC();

    // post semaphore to send the new measurement over UART
    Semaphore_post(semHandle);
    PIN_setOutputValue(ledPinHandle, Board_LED0, !PIN_getOutputValue(Board_LED0));
}


/*********************************************************************
 * @fn      uartWriteFxn
 *
 * @brief   Waiting for a new measurement, to send it over UART to
 *          the data logger
 *
 */
Void uartWriteFxn(UArg arg0, UArg arg1)
{
    UART_Handle uartHandle;
    uint32_t    msgCounter = 0;

    // Create a UART with data processing off
    UART_Params                 uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode    = UART_DATA_BINARY;
    uartParams.readDataMode     = UART_DATA_BINARY;
    uartParams.readReturnMode   = UART_RETURN_FULL;
    uartParams.readEcho         = UART_ECHO_OFF;
    uartParams.baudRate         = BAUD_RATE;

    while (1)
    {
        // wait for a new measurement
        Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);

        /*
         * get a handle to UART to send the new measurement as a message
         */
        uartHandle = UART_open(Board_UART0, &uartParams);
        if (uartHandle == NULL)
        {
            System_abort("Error opening the UART");
        }


        /*
         * construct the message
         */
        char        msg[MSG_LEN];
        const char  *msgPtr = msg;

        // update the message index - 4 bytes
        msgCounter++;
        memcpy(msg, &msgCounter, 4);

        // format the temperature sign - 1 byte
        if (temp < 0)
        {
            msg[TEMP_SIGN_IDX] = TEMP_NEG_SIGN;
            temp *= -1;
        }
        else
            msg[TEMP_SIGN_IDX] = TEMP_POS_SIGN;

        // format the temperature - 3 bytes
        int tempTmp = temp;
        int i;

        for (i = TEMP_LAST_IDX; i >= TEMP_FIRST_IDX; i--)
        {
            msg[i] = (tempTmp % 10) + ZERO_CHAR;
            tempTmp /= 10;
        }

        /*
         * send the message over UART and close the handle
         */
        UART_write(uartHandle, msgPtr, MSG_LEN);

        UART_close(uartHandle);
    }
}


/*******************************************************************************
 * @fn          Main
 *
 * @brief       Program Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initUART();

    // Construct a Semaphore object to be use as a resource lock, inital count 0
    Semaphore_Params    semParams;
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 0, &semParams);
    semHandle = Semaphore_handle(&semStruct);

    // Construct UART write task
    Task_Params             taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize    = TASK_STACK_SIZE;
    taskParams.stack        = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)uartWriteFxn, &taskParams, NULL);

    // Construct a periodic clock instance
    UInt32 sleepTickCount = SAMPLING_INTERVAL / Clock_tickPeriod;
    Clock_Params        clkParams;
    Clock_Params_init(&clkParams);
    clkParams.startFlag = TRUE;
    clkParams.period    = sleepTickCount;

    Clock_construct(&clk0Struct, (Clock_FuncPtr)clk0Fxn, sleepTickCount , &clkParams);

    // Open LED pins
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    // Enable the measurements
    AONBatMonEnable();

    // Start kernel
    BIOS_start();

    return (0);
}
