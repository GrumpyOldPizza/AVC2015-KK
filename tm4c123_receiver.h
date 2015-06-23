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

#if !defined(_TM4C123_RECEIVER_H)
#define _TM4C123_RECEIVER_H

#include "fifo.h"

typedef struct _tm4c123_receiver_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    uint32_t          pulse;
} tm4c123_receiver_entry_t;

typedef struct _tm4c123_receiver_entry_payload_t {
    uint32_t          pulse;
} tm4c123_receiver_entry_payload_t;

#define TM4C123_RECEIVER_EVENT_CH1IN    1
#define TM4C123_RECEIVER_EVENT_CH2IN    2
#define TM4C123_RECEIVER_EVENT_CH3IN    3

typedef struct _tm4c123_switch_fifo_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
} tm4c123_switch_fifo_entry_t;

#define TM4C123_SWITCH_EVENT_NONE       0
#define TM4C123_SWITCH_EVENT_SWITCH_ON  1
#define TM4C123_SWITCH_EVENT_SWITCH_OFF 2

#define TM4C123_RECEIVER_CH1IN GPIO_PIN_1
#define TM4C123_RECEIVER_CH2IN GPIO_PIN_2
#define TM4C123_RECEIVER_CH3IN GPIO_PIN_3

extern void GPIOE_IRQHandler(void);
extern int tm4c123_switch_event(void);
extern int tm4c123_receiver_heartbeat(void);
extern void tm4c123_receiver_reset(void);
extern void tm4c123_receiver_initialize(void);

#endif /* _TM4C123_PRECEIVER_H */
