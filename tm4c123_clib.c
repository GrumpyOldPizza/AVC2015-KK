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

#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#undef errno
extern int errno;

static int tm4c123_clib_initialized = 0;

static void tm4c123_clib_initialize(void)
{
    unsigned int rate;

    /* Enable Peripheral Clocks 
     */

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    /* Enable pin PA0 for UART0 U2RX
     */
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0);

    /* Enable pin PA1 for UART0 U2TX
     */

    /* Now modify the configuration of the pins that we unlocked.
     */
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_1);

    
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

    rate = ((SystemCoreClock * 8) / TM4C123_CLIB_UART_SPEED + 1) >> 1;

    UART0->CTL          = 0;
    UART0->ECR_UART_ALT = ~0u;
    UART0->IBRD         = rate >> 6;
    UART0->FBRD         = rate & 63;
    UART0->LCRH         = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    UART0->IM           = 0;
    UART0->ICR          = ~0u;
    UART0->CTL          = (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN);

    tm4c123_clib_initialized = 1;
}

extern uint32_t __HeapBase[];
extern uint32_t __StackLimit[];

void * _sbrk (int nbytes)
{
    void *p;

    static void *__HeapCurrent = (void*)(&__HeapBase[0]);

    if (((uint8_t*)__HeapCurrent + nbytes) <= (uint8_t*)(&__StackLimit[0]))
    {
	p = __HeapCurrent;
	
	__HeapCurrent = (void*)((uint8_t*)__HeapCurrent + nbytes);

	return p;
    }
    else
    {
	errno = ENOMEM;

	return  (void *) -1;
    }
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    errno = EINVAL;

    return -1;
}

int _close(int file) {
    return -1;
}

int _isatty(int file) 
{
    switch (file) {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;

    default:
	errno = EBADF;
	return 0;
    }
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;

    return 0;
}

int _lseek(int file, int offset, int whence)
{
    return 0;
}

int _read(int file, char *buf, int nbytes)
{
    int n;

    switch (file) {
    case STDIN_FILENO:
	if (!tm4c123_clib_initialized)
	{
	    tm4c123_clib_initialize();
	}

        for (n = 0; n < nbytes; n++)
	{
	    if (UART0->FR & UART_FR_RXFE)
	    {
		break;
	    }

	    buf[n] = UART0->DR & UART_DR_DATA_M;
        }
	return n;

    default:
        errno = EBADF;
        return -1;
    }
}

int _write(int file, char *buf, int nbytes)
{
    int n;

    switch (file) {
    case STDOUT_FILENO:
    case STDERR_FILENO:
	if (!tm4c123_clib_initialized)
	{
	    tm4c123_clib_initialize();
	}

        for (n = 0; n < nbytes; n++)
	{
	    while (UART0->FR & UART_FR_TXFF)
	    {
		continue;
	    }
	    
	    UART0->DR = buf[n];
        }
	return n;

    default:
        errno = EBADF;
        return -1;
    }
}
