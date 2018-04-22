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

/*Includes*/
#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>
#include "environment_sensor.h"
#include "Timer32_driver.h"
#include "UART_driver.h"

/* Definitions */
#define BUFFER_LENGTH 1024
#define ON  1
#define OFF 0

//Frequently Used AT Commands
const char *AT_RESET  = "AT+RST\r\n";
const char *AT_MODE2  = "AT+CWMODE_CUR=2\r\n";
const char *AT_CWSAP  = "AT+CWSAP_CUR=\"DRIZZY\",\"pass\",5,0\r\n";
const char *AT_CIFSR  = "AT+CIFSR\r\n";
const char *AT_CIPMUX = "AT+CIPMUX=1\r\n";
const char *AT_SERVER = "AT+CIPSERVER=1,80\r\n";
const char *web_response_header = "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\n";

//UART comm variables
volatile char buffer[BUFFER_LENGTH];
volatile uint16_t idx = 0;
volatile uint8_t response_complete = 0;
volatile int err = 0;

//HTML
char *webpage_html_start = "<!DOCTYPE html><html> <body> <center>"
        "<h1>Set the color of LED light:<br></h1>"
        "<form action=\"\" method=\"get\">"
        "<p style=\"color:black;font-size:28px\"> LED color:<br> </p>"
        "<p style=\"color:red;font-size:24px\">"
        "<label for=\"red\">Red</label>"
        "<input type=\"checkbox\" name=\"red\"> </p>"
        "<p style=\"color:green;font-size:24px\">"
        "<label for=\"green\">Green</label>"
        "<input type=\"checkbox\" name=\"green\"> </p>"
        "<p style=\"color:blue;font-size:24px\">"
        "<label for=\"blue\">Blue</label>"
        "<input type=\"checkbox\" name=\"blue\"> </p>"
        "<p style=\"color:black;font-size:28px\">Turn Light:"
        "<input type=\"radio\" name=\"OnOff\" value=\"on\"> On"
        "<input type=\"radio\" name=\"OnOff\" value=\"off\" checked> Off<br></p>"
        "<fieldset>"
        "<legend style=\"color:black;font-size:28px\">Environmental variables: </legend>"
        "<p style=\"color:black;font-size:28px\">";

//Generate the first portions of the html code
void formatStart(char *text,int red, int blue, int green, int onoff){
    char rtext[10], btext[10], gtext[10], ontext[10], offtext[10];
    if(red){
        strcpy(rtext," checked");
    }else{
        strcpy(rtext," notchecked");
    }
    if(blue){
        strcpy(btext," checked");
    }else{
        strcpy(btext," notchecked");
    }
    if(green){
        strcpy(gtext," checked");
    }else{
        strcpy(gtext," notchecked");
    }if(onoff){
        strcpy(ontext," checked");
        strcpy(offtext," notchecked");
    }else{
        strcpy(offtext," checked");
        strcpy(ontext," notchecked");
    }
    sprintf(text,"<!DOCTYPE html><html> <body> <center>"
        "<h1>Set the color of LED light:<br></h1>"
        "<form action=\"\" method=\"get\">"
        "<p style=\"color:black;font-size:28px\"> LED color:<br> </p>"
        "<p style=\"color:red;font-size:24px\">"
        "<label for=\"red\">Red</label>"
        "<input type=\"checkbox\" name=\"red\"%s> </p>"
        "<p style=\"color:green;font-size:24px\">"
        "<label for=\"green\">Green</label>"
        "<input type=\"checkbox\" name=\"green\"%s> </p>"
        "<p style=\"color:blue;font-size:24px\">"
        "<label for=\"blue\">Blue</label>"
        "<input type=\"checkbox\" name=\"blue\"%s> </p>"
        "<p style=\"color:black;font-size:28px\">Turn Light:"
        "<input type=\"radio\" name=\"OnOff\" value=\"on\"%s> On"
        "<input type=\"radio\" name=\"OnOff\" value=\"off\"%s> Off<br></p>"
        "<fieldset>"
        "<legend style=\"color:black;font-size:28px\">Environmental variables: </legend>"
        "<p style=\"color:black;font-size:28px\">",rtext,gtext,btext,ontext,offtext);
}

//seperated from webpage_html_start by data section that needs to be formated
char *webpage_html_end = "</fieldset> <br> <input type=\"submit\" value=\"Submit\"> </form> </center> </body> </html>";

//Generate HTML webpage
void formatHTMLPage(char *msg, float temperature, float humidity, float pressure, int red, int blue, int green, int onoff){
    char temp[100];
    char temp2[1000];
    msg[0] = 0;
    sprintf(temp,"Temperature: %02f<br>Humidity: %02f%%<br>Pressure: %3.1f mmHg<br></p>",temperature,humidity,pressure);
    //strcat(msg, webpage_html_start);
    formatStart(temp2,red,blue,green, onoff);
    strcat(msg, temp2);
    strcat(msg, temp);
    strcat(msg, webpage_html_end);
}

//Global status indicators
int red_LED = OFF, blue_LED = OFF, green_LED = OFF, onOff = OFF;

//Function prototype
void updateLEDs(void);

int main(void)
{
    int channel;
    char cipsend[32];
    char close[32];
    char *req = NULL;
    char *res = NULL;
    char webpage[1536];

    // data used by the bme
    struct bme280_dev dev;
    struct bme280_data compensated_data;
    float normal_humidity = 0;
    float normal_pressure = 0;
    float normal_temperature = 0;

    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    //LED Config
    P2DIR |= BIT0|BIT1|BIT2;
    P2OUT &= ~(BIT0|BIT1|BIT2);

    // (s)mclk = 12 MHz
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //Timer 32 initialization
    Timer32_init();

    // initialize bme
    BME280_Init(&dev);    // initializes timer32

    // initialize uart
    UARTA0_init();
    UARTA2_init();

    // enable interrupts
    MAP_Interrupt_enableMaster();

    // reset the esp
    UART_transmitString(EUSCI_A2_BASE, AT_RESET);
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
    Timer32_waitms(1000);
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
        //Check to see if a browser sent a response
        req = strstr(buffer, "+IPD");
        Timer32_waitms(100);
        if (NULL != req)
        {
            int i = 0;
            char temp[3];
            char get_request[60];
            //check if update
            res = strstr(req, "?");
            if(res != NULL)
            {
                sscanf(res,"?%60s",get_request);
                //check for red
                res = strstr(get_request, "red=");
                if(res != NULL){
                    sscanf(res,"red=%2c",temp);

                    if(temp[1]=='n'){
                        red_LED = ON;
                    }else{
                        red_LED = OFF;
                    }
                }else{
                    red_LED = OFF;
                }
                temp[1]='x';
                //check for blue
                res = strstr(get_request, "blue=");
                if(res != NULL){
                    sscanf(res,"blue=%2c",temp);
                    if(temp[1]=='n'){
                        blue_LED = ON;
                    }else{
                        blue_LED = OFF;
                    }
                }else{
                    blue_LED = OFF;
                }
                temp[1]='x';
                //check for green
                res = strstr(get_request, "green=");
                if(res != NULL){
                    sscanf(res,"green=%2c",temp);
                    if(temp[1]=='n'){
                        green_LED = ON;
                    }else{
                        green_LED = OFF;
                    }
                }else{
                    green_LED = OFF;
                }
                temp[1]='x';
                //check for on/off
                res = strstr(get_request, "OnOff");
                if(res != NULL){
                    sscanf(res,"OnOff=%2c",temp);
                    if(temp[1]=='n'){
                        onOff = ON;
                    }else{
                        onOff = OFF;
                    }
                }
                temp[1]='x';

                //Update LEDs
                updateLEDs();
            }

            // search buffer for http request
            res = strstr(req, "HTTP");
            if (NULL != res)
            {
                // read bme
                BME280_Read(&dev, &compensated_data);

                // format the sensor data properly
                normal_humidity = compensated_data.humidity / 1000;
                normal_pressure = (compensated_data.pressure / 13332.237) + 17;
                normal_temperature = compensated_data.temperature * 0.018 + 32;

                // put data into webpage
                formatHTMLPage(webpage, normal_temperature, normal_humidity, normal_pressure,red_LED,blue_LED,green_LED,onOff);

                // get channel number from http request
                sscanf(req, "+IPD,%d", &channel);

                //header response
                sprintf(cipsend, "AT+CIPSEND=%d,%d\r\n", channel, strlen(web_response_header));
                UART_transmitString(EUSCI_A2_BASE, cipsend);
                while(strstr(buffer,">")==NULL){
                    ;//wait for wrap
                }
                UART_transmitString(EUSCI_A2_BASE, web_response_header);
                Timer32_waitms(100);

                // format send command to channel for page length bytes
                sprintf(cipsend, "AT+CIPSEND=%d,%d\r\n", channel, strlen(webpage));

                // send the send command
                UART_transmitString(EUSCI_A2_BASE, cipsend);
                while(strstr(buffer,">")==NULL){
                    ;//wait for wrap
                }
                // send the webpage
                UART_transmitString(EUSCI_A2_BASE, webpage);

                Timer32_waitms(500);

                sprintf(close, "AT+CIPCLOSE=%d\r\n", channel);
                UART_transmitString(EUSCI_A2_BASE, close);

            }

            //clear buffer. If not cleared, the web page will be sent indefinitely
            idx = 0;
            for(i = 0; i < BUFFER_LENGTH; i++)
            {
                buffer[i] = 0;
            }

        }

    }

}

void updateLEDs(void){
    if(onOff == ON){
        if(red_LED == ON){
            P2OUT |= BIT0;
        }else{
            P2OUT &= ~BIT0;
        }
        if(blue_LED == ON){
            P2OUT |= BIT2;
        }else{
            P2OUT &= ~BIT2;
        }
        if(green_LED == ON){
            P2OUT |= BIT1;
        }else{
            P2OUT &= ~BIT1;
        }
    }else{
        P2OUT &= ~(BIT0|BIT1|BIT2);
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
