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

static uint32_t armv7m_udelay_scale = 0;

void armv7m_udelay(uint32_t usec)
{
    uint32_t count;

    if (armv7m_udelay_scale == 0)
    {
	armv7m_udelay_scale = SystemCoreClock / 1000000;
    }

    count = (usec * armv7m_udelay_scale) / 3;

#if defined(__ARM_ARCH_6M__)
    /* GCC uses pre-UAL for ARMV6M ... */
    __asm__ volatile ("1: sub  %0, %0, #1 \n"
		      "   bne  1b         \n"
		      : "+r" (count)
		      :
		      :);
#else  /* _ARM_ARCH_6M__ */
    __asm__ volatile ("1: subs  %0, #1 \n"
		      "   bne.n 1b     \n"
		      : "+r" (count)
		      :
		      :);
#endif /* _ARM_ARCH_6M__ */
}
