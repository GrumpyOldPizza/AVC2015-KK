/*
 * Copyright (c) 2015 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "kitty.h"
#include "gps.h"

typedef struct _tm4c123_i2c_device_t {
    const uint8_t          *wdata;
    volatile uint32_t      wcount;
} tm4c123_uart_device_t;

static tm4c123_uart_device_t tm4c123_uart_device;

void UART2_IRQHandler(void)
{
    uint8_t c;

    ARMV7M_PROFILE_TAG_PUSH(UART);

    UART2->ICR = UART_ICR_RXIC | UART_ICR_TXIC | UART_ICR_RTIC;
    
    while (!(UART2->FR & UART_FR_RXFE))
    {
	c = UART2->DR & UART_DR_DATA_M;

	gps_receive(c);
    }

    if (UART2->IM & UART_IM_TXIM)
    {
	while ((tm4c123_uart_device.wcount != 0) && !(UART2->FR & UART_FR_TXFF))
	{
	    UART2->DR = *tm4c123_uart_device.wdata;

	    tm4c123_uart_device.wdata++;
	    tm4c123_uart_device.wcount--;
	}

	if (tm4c123_uart_device.wcount == 0)
	{
	    UART2->IM &= ~UART_IM_TXIM;
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}

void tm4c123_uart_send(const uint8_t *data, uint32_t count)
{
    tm4c123_uart_device.wdata = data;
    tm4c123_uart_device.wcount = count;

    UART2->IM |= UART_IM_TXIM;

    NVIC_SetPendingIRQ(UART2_IRQn);
}

void tm4c123_uart_wait(void)
{
    while ((tm4c123_uart_device.wcount != 0) || (UART2->FR & UART_FR_BUSY))
    {
        continue;
    }
}

void tm4c123_uart_configure(uint32_t speed)
{
    unsigned int rate;

    tm4c123_uart_wait();

    /*
     * BRD = BRDI + BRDF = UARTSysClk / (16 * BaudRate)
     *
     * Scale by 64 to compute the fraction:
     *
     * BRD * 64 = (UARTSysClk * 64) / (16 * BaudRate)
     * BRD * 64 = (UARTSysClk * 4) / (BaudRate)
     * BRDI = (BRD * 64) / 64;
     * BRDF = (BRD * 64) % 64;
     *
     * Round by 0.5:
     *
     * BRD * 64 = (2 * ((UARTSysClk * 4) / (BaudRate)) + 1) / 2
     * BRD * 64 = (((UARTSysClk * 8) / (BaudRate)) + 1) / 2
     *
     * UARTSysClk = SystemCoreClock
     * BaudRate = 115200
     */

    rate = ((SystemCoreClock * 8) / speed + 1) >> 1;

    UART2->CTL          = 0;
    UART2->ECR_UART_ALT = ~0u;
    UART2->IBRD         = rate >> 6;
    UART2->FBRD         = rate & 63;
    UART2->LCRH         = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    UART2->IM           = UART_IM_RXIM | UART_IM_RTIM;
    UART2->ICR          = ~0u;
    UART2->CTL          = (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN);
}

void tm4c123_uart_initialize(void)
{
    /* Enable Peripheral Clocks 
     */

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    /* Enable pin PD6 for UART2 U2RX
     */
    ROM_GPIOPinConfigure(GPIO_PD6_U2RX);
    ROM_GPIOPinTypeUART(GPIOD_BASE, GPIO_PIN_6);

    /* Enable pin PD7 for UART2 U2TX
     * First open the lock and select the bits we want to modify in the GPIO commit register.
     */

    (&GPIOD->LOCK)[0] = GPIO_LOCK_KEY; /* GPIOD->LOCK */
    (&GPIOD->LOCK)[1] = 0x80;          /* GPIOD->CR   */

    /* Now modify the configuration of the pins that we unlocked.
     */
    ROM_GPIOPinConfigure(GPIO_PD7_U2TX);
    ROM_GPIOPinTypeUART(GPIOD_BASE, GPIO_PIN_7);

    NVIC_SetPriority(UART2_IRQn, 4);
    NVIC_EnableIRQ(UART2_IRQn);
}
