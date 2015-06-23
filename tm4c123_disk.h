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

#if !defined(_TM4C123_DISK_H)
#define _TM4C123_DISK_H

#include "tm4c123_spi.h"

#define TM4C123_DISK_CONFIG_DMA      0

#define TM4C123_DISK_SSI_SPEED       20000000
#define TM4C123_DISK_SSI_FIFO_COUNT  8

extern void tm4c123_disk_time_start(void);
extern bool tm4c123_disk_time_elapsed(uint32_t time);
extern int tm4c123_disk_lock(void);
extern void tm4c123_disk_unlock(void);
extern bool tm4c123_disk_present(void);
extern uint32_t tm4c123_disk_mode(int mode);
extern void tm4c123_disk_select(void);
extern void tm4c123_disk_deselect(void);
extern void tm4c123_disk_send(uint8_t data);
extern uint8_t tm4c123_disk_receive(void);
extern void tm4c123_disk_send_block(const uint8_t *data);
extern uint32_t tm4c123_disk_receive_block(uint8_t *data);
extern void tm4c123_disk_initialize(void);

#define RFAT_PORT_DISK_TIME_START()              tm4c123_disk_time_start()
#define RFAT_PORT_DISK_TIME_ELAPSED(_time)       tm4c123_disk_time_elapsed((_time))
#define RFAT_PORT_DISK_LOCK()                    tm4c123_disk_lock()
#define RFAT_PORT_DISK_UNLOCK()                  tm4c123_disk_unlock()
#define RFAT_PORT_DISK_SPI_PRESENT()             tm4c123_disk_present()
#define RFAT_PORT_DISK_SPI_MODE(_mode)           tm4c123_disk_mode((_mode))
#define RFAT_PORT_DISK_SPI_SELECT()              tm4c123_disk_select()
#define RFAT_PORT_DISK_SPI_DESELECT()            tm4c123_disk_deselect()
#define RFAT_PORT_DISK_SPI_SEND(_data)           tm4c123_disk_send((_data))
#define RFAT_PORT_DISK_SPI_RECEIVE()             tm4c123_disk_receive()
#define RFAT_PORT_DISK_SPI_SEND_BLOCK(_data)     tm4c123_disk_send_block((_data))
#define RFAT_PORT_DISK_SPI_RECEIVE_BLOCK(_data)  tm4c123_disk_receive_block((_data))

#endif /* _TM4C123_DISK_H */
