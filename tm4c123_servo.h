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

#if !defined(_TM4C123_SERVO_H)
#define _TM4C123_SERVO_H

#define TM4C123_SERVO_CH1OUT GPIO_PIN_4
#define TM4C123_SERVO_CH2OUT GPIO_PIN_5

#define TM4C123_SERVO_MODE_AUTONOMOUS 0
#define TM4C123_SERVO_MODE_PASSTHROU  1
#define TM4C123_SERVO_MODE_FAILSAFE   2

static void inline tm4c123_servo_passthrou_ch1out(uint32_t data)
{
    armv7m_bitband_peripheral_write(&GPIOC->DATA, 4, data);
}

static void inline tm4c123_servo_passthrou_ch2out(uint32_t data)
{
    armv7m_bitband_peripheral_write(&GPIOC->DATA, 5, data);
}

extern void WTIMER0B_IRQHandler(void);

extern void tm4c123_servo_fault(void);
extern void tm4c123_servo_configure(uint32_t mode);
extern void tm4c123_servo_initialize(void);

#endif /* _TM4C123_SERVO_H */
