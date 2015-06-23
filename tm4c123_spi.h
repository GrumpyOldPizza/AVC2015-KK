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

#if !defined(_TM4C123_SPI_H)
#define _TM4C123_SPI_H

/*
 * TFT Boosterpack with Default/Legacy configutation:
 *
 * Def  Legacy
 * PB4  PB4   SCLK        SSI2CLK
 * PB6  PB6   MISO        SSI2RX
 * PB7  PB7   MOSI        SSI2TX
 *
 * -    PA3   CS_EEPROM   GPIO
 * PA3  PA4   CS_SDCARD   GPIO
 * PA4  PB5   CS_TFT      GPIO
 * PE4  PE5   DC_TFT      GPIO
 */

/*
 *
 *
 * TM4C123_SPI_CONFIG_LEGACY
 *
 * 0    1
 * PB4  PB4   SCLK        SSI2CLK
 * PB6  PB6   MISO        SSI2RX
 * PB7  PB7   MOSI        SSI2TX
 * -    PA3   CS_EEPROM   GPIO
 * PA3  PA4   CS_SDCARD   GPIO
 * PA4  PB5   CS_TFT      GPIO
 * PE4  PE5   DC_TFT      GPIO
 */

#define TM4C123_SPI_CONFIG_LEGACY   0

extern void tm4c123_spi_initialize(void);

#endif /* _TM4C123_SPI_H */
