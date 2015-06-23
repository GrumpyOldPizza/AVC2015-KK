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

#if !defined(_TM4C123_BUTTON_H)
#define _TM4C123_BUTTON_H

#include "fifo.h"

typedef struct _tm4c123_button_fifo_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
} tm4c123_button_fifo_entry_t;

#define TM4C123_BUTTON_EVENT_NONE             0
#define TM4C123_BUTTON_EVENT_BUTTON_1_PRESS   1
#define TM4C123_BUTTON_EVENT_BUTTON_1_TIMEOUT 2
#define TM4C123_BUTTON_EVENT_BUTTON_1_RELEASE 3
#define TM4C123_BUTTON_EVENT_BUTTON_2_PRESS   4
#define TM4C123_BUTTON_EVENT_BUTTON_2_TIMEOUT 5
#define TM4C123_BUTTON_EVENT_BUTTON_2_RELEASE 6

#define TM4C123_BUTTON_2_ALT    GPIO_PIN_0 /* PF0 */ 
#define TM4C123_BUTTON_1        GPIO_PIN_4 /* PF4 */
#define TM4C123_BUTTON_2        GPIO_PIN_3 /* PB3 */ 

extern void GPIOB_IRQHandler(void);
extern void GPIOF_IRQHandler(void);

extern int tm4c123_button_event(void);
extern void tm4c123_button_initialize(void);

#endif /* _TM4C123_BUTTON_H */
