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

#if !defined(_ARMV7M_SYSTICK_H)
#define _ARMV7M_SYSTICK_H

#define TIMEOUT_SLOT_I2C              0
#define TIMEOUT_SLOT_MPU6050          1
#define TIMEOUT_SLOT_BUTTON1_DEBOUNCE 2
#define TIMEOUT_SLOT_BUTTON1_RELEASE  3
#define TIMEOUT_SLOT_BUTTON2_DEBOUNCE 4
#define TIMEOUT_SLOT_BUTTON2_RELEASE  5
#define TIMEOUT_SLOT_RECEIVER         6
#define TIMEOUT_SLOT_CONTROL          7
#define TIMEOUT_SLOT_NUM              8

typedef void (*armv7m_systick_callback_t)(void);

extern void SysTick_Handler(void);

extern uint64_t armv7m_systick_clock(void);
extern void armv7m_systick_mdelay(uint32_t msec);
extern void armv7m_systick_timeout(unsigned int slot, uint32_t msec, armv7m_systick_callback_t callback);
extern void armv7m_systick_initialize(void);

#endif /* _ARMV7M_SYSTICK_H */
