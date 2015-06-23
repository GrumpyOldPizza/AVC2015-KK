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

void tm4c123_spi_initialize(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    /* Enable pin PB4 for SSI2 SSI2CLK
     */
    ROM_GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    ROM_GPIOPinTypeSSI(GPIOB_BASE, GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIOB_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);


    /* Enable pin PB6 for SSI2 SSI2RX
     */
    ROM_GPIOPinConfigure(GPIO_PB6_SSI2RX);
    ROM_GPIOPinTypeSSI(GPIOB_BASE, GPIO_PIN_6);
    ROM_GPIOPadConfigSet(GPIOB_BASE, GPIO_PIN_6, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    /* Enable pin PB7 for SSI2 SSI2TX
     */
    ROM_GPIOPinConfigure(GPIO_PB7_SSI2TX);
    ROM_GPIOPinTypeSSI(GPIOB_BASE, GPIO_PIN_7);
    ROM_GPIOPadConfigSet(GPIOB_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

#if (TM4C123_SPI_CONFIG_LEGACY == 0)

    /* Enable pin PA3 for GPIOInput (CS_SDCARD)
     */
    ROM_GPIOPinTypeGPIOInput(GPIOA_BASE, GPIO_PIN_3);
    ROM_GPIOPadConfigSet(GPIOA_BASE, GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    /* Enable pin PA4 for GPIOOutput (CS_TFT)
     */
    ROM_GPIOPinTypeGPIOOutput(GPIOA_BASE, GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIOA_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    /* Enable pin PE4 for GPIOOutput
     */
    ROM_GPIOPinTypeGPIOOutput(GPIOE_BASE, GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIOE_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    /* Set defaults for CS/DC to H
     */
    GPIOA->DATA |= (GPIO_PIN_3 | GPIO_PIN_4);
    GPIOE->DATA |= (GPIO_PIN_4);
    
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */

    /* Enable pin PA3 for GPIOOutput (CS_EEPROM)
     */
    ROM_GPIOPinTypeGPIOOutput(GPIOA_BASE, GPIO_PIN_3);
    ROM_GPIOPadConfigSet(GPIOA_BASE, GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    /* Enable pin PA4 for GPIOInput (CS_SDCARD)
     */
    ROM_GPIOPinTypeGPIOInput(GPIOA_BASE, GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIOA_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    /* Enable pin PB5 for GPIOOutput (CS_TFT)
     */
    ROM_GPIOPinTypeGPIOOutput(GPIOB_BASE, GPIO_PIN_5);
    ROM_GPIOPadConfigSet(GPIOB_BASE, GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    /* Enable pin PE5 for GPIOOutput (DS_TFT)
     */
    ROM_GPIOPinTypeGPIOOutput(GPIOE_BASE, GPIO_PIN_5);
    ROM_GPIOPadConfigSet(GPIOE_BASE, GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    /* Set defaults for CS/DC to H
     */
    GPIOA->DATA |= (GPIO_PIN_3 | GPIO_PIN_4);
    GPIOB->DATA |= (GPIO_PIN_5);
    GPIOE->DATA |= (GPIO_PIN_5);

#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */

    /* Initialize slave interfaces.
     */

    tm4c123_tft_initialize();
    tm4c123_disk_initialize();
}
