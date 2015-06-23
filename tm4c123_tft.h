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

#if !defined(_TM4C123_TFT_H)
#define _TM4C123_TFT_H

#include "tm4c123_spi.h"

#define TM4C123_TFT_SSI_SPEED     10000000
#define TM4C123_TFT_SSI_FIFO_SIZE 8

extern void tm4c123_tft_select(void);
extern void tm4c123_tft_deselect(void);
extern void tm4c123_tft_command(void);
extern void tm4c123_tft_data(void);
extern void tm4c123_tft_send(uint8_t data);
extern uint8_t tm4c123_tft_receive(void);
extern void tm4c123_tft_setup_data16(void);
extern void tm4c123_tft_finish_data16(void);
extern void tm4c123_tft_send_data16(uint16_t data);
extern void tm4c123_tft_send_data16_multiple(uint16_t data, uint32_t count);
extern void tm4c123_tft_initialize(void);

#define TFT_PORT_SPI_SELECT()                            tm4c123_tft_select()
#define TFT_PORT_SPI_DESELECT()                          tm4c123_tft_deselect()
#define TFT_PORT_SPI_COMMAND()                           tm4c123_tft_command()
#define TFT_PORT_SPI_DATA()                              tm4c123_tft_data()
#define TFT_PORT_SPI_SEND(_data)                         tm4c123_tft_send((_data))
#define TFT_PORT_SPI_RECEIVE()                           tm4c123_tft_receive()
#define TFT_PORT_SPI_SETUP_DATA16()                      tm4c123_tft_setup_data16()
#define TFT_PORT_SPI_FINISH_DATA16()                     tm4c123_tft_finish_data16()
#define TFT_PORT_SPI_SEND_DATA16(_data)                  tm4c123_tft_send_data16((_data))
#define TFT_PORT_SPI_SEND_DATA16_MULTIPLE(_data, _count) tm4c123_tft_send_data16_multiple((_data),(_count))

#endif /* _TM4C123_TFT_H */
