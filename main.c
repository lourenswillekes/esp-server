/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05 
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: 
*******************************************************************************/

#include "driverlib.h"

#include <stdint.h>
#include <stdbool.h>

#include "Timer32_driver.h"
#include "UART_driver.h"


#define BUFFER_LENGTH 1024

const char *AT_MODE2  = "AT+CWMODE_CUR=2\r\n";
const char *AT_CWSAP  = "AT+CWSAP_CUR=\"DRIZZY\",\"pass\",5,0\r\n";
const char *AT_CIFSR  = "AT+CIFSR\r\n";
const char *AT_CIPMUX = "AT+CIPMUX=1\r\n";
const char *AT_SERVER = "AT+CIPSERVER=1,80\r\n";

volatile char buffer[BUFFER_LENGTH];
volatile uint16_t idx = 0;
volatile uint8_t response_complete = 0;
volatile int err = 0;


int main(void)
{
    int j;
    int channel;
    char cipsend[32];
    char *req = NULL;
    char *res = NULL;

    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    // clk = 12 MHz
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // initialize timer32
    Timer32_init();

    // initialize uart
    UARTA0_init();
    UARTA2_init();

    // enable interrupts
    MAP_Interrupt_enableMaster();


    UART_transmitString(EUSCI_A2_BASE, "AT+RST\r\n");
    Timer32_waitms(500);

    // set mode to softap
    UART_transmitString(EUSCI_A2_BASE, AT_MODE2);
    Timer32_waitms(500);
    res = strstr(buffer, "OK");
    if (NULL != res)
    {
        UART_transmitString(EUSCI_A0_BASE, "01  OK'd\r\n");
    } else
    {
        err = 1;
        UART_transmitString(EUSCI_A0_BASE, "01  NOT OK'd\r\n");
    }


    if (!err) {
    // configure softap
    idx = 0;
    UART_transmitString(EUSCI_A2_BASE, AT_CWSAP);
    Timer32_waitms(500);
    res = strstr(buffer, "OK");
    if (NULL != res)
    {
        UART_transmitString(EUSCI_A0_BASE, "02  OK'd\r\n");
    } else
    {
        err = 2;
        UART_transmitString(EUSCI_A0_BASE, "02  NOT OK'd\r\n");
    }
    }


    if (!err) {
    // get local ip (display only)
    idx = 0;
    UART_transmitString(EUSCI_A2_BASE, AT_CIFSR);
    Timer32_waitms(500);
    res = strstr(buffer, "OK");
    if (NULL != res)
    {
        UART_transmitString(EUSCI_A0_BASE, "03  OK'd\r\n");
    } else
    {
        err = 3;
        UART_transmitString(EUSCI_A0_BASE, "03  NOT OK'd\r\n");
    }
    }


    if (!err) {
    // allow multiple connections
    idx = 0;
    UART_transmitString(EUSCI_A2_BASE, AT_CIPMUX);
    Timer32_waitms(500);
    res = strstr(buffer, "OK");
    if (NULL != res)
    {
        UART_transmitString(EUSCI_A0_BASE, "04  OK'd\r\n");
    } else
    {
        err = 4;
        UART_transmitString(EUSCI_A0_BASE, "04  NOT OK'd\r\n");
    }
    }


    if (!err) {
    // start the server
    idx = 0;
    UART_transmitString(EUSCI_A2_BASE, AT_SERVER);
    Timer32_waitms(2000);
    res = strstr(buffer, "OK");
    if (NULL != res)
    {
        UART_transmitString(EUSCI_A0_BASE, "05  OK'd\r\n");
    } else
    {
        err = 4;
        UART_transmitString(EUSCI_A0_BASE, "05  NOT OK'd\r\n");
    }
    }


    idx = 0;
    while (1)
    {

        req = strstr(buffer, "+IPD");
        if (NULL != req)
        {
            res = strstr(req, "HTTP");
            if (NULL != res)
            {
                sscanf(req, "+IPD,%d", &channel);
                sprintf(cipsend, "AT+CIPSEND=%d,4\r\n", channel);
                UART_transmitString(EUSCI_A2_BASE, cipsend);
                UART_transmitString(EUSCI_A2_BASE, "test\r\n");
            }

            idx = 0;
            buffer[0] = 0;

        }



    }




}


void UARTA0_ISR(void)
{
    uint8_t byte;

    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        byte = MAP_UART_receiveData(EUSCI_A0_BASE);
        MAP_UART_transmitData(EUSCI_A2_BASE, byte);
    }
}

void UARTA2_ISR(void)
{
    uint8_t byte;

    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        byte = MAP_UART_receiveData(EUSCI_A2_BASE);
        MAP_UART_transmitData(EUSCI_A0_BASE, byte);
        // add byte to the rx buffer
        buffer[idx++] = (char) byte;
        if (BUFFER_LENGTH == idx)
        {
            idx = 0;
        }
    }
}
