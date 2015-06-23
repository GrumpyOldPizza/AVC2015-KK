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

uint32_t *__StackLevel;

void armv7m_stack_check(void)
{
    uint32_t *__StackCheck = __StackLevel;

    while (__StackCheck > &__StackLimit[8])
    {
	__StackLevel = __StackCheck;
	__StackCheck -= 8;

	if ((__StackCheck[0] == 0xdeadbeef) &&
	    (__StackCheck[1] == 0xdeadbeef) &&
	    (__StackCheck[2] == 0xdeadbeef) &&
	    (__StackCheck[3] == 0xdeadbeef) &&
	    (__StackCheck[4] == 0xdeadbeef) &&
	    (__StackCheck[5] == 0xdeadbeef) &&
	    (__StackCheck[6] == 0xdeadbeef) &&
	    (__StackCheck[7] == 0xdeadbeef))
	{
	    break;
	}
    }
}

static void armv7m_stack_fill(void)
{
    __asm__("   ldr   r0, =0xdeadbeef  ;\n"
	    "   ldr   r1, =__StackLimit;\n"
	    "   mov   r2, r13          ;\n"
	    "   adds  r1, #32          ;\n"
	    "   subs  r2, r1           ;\n"
	    "1: subs  r2, #4           ;\n"
	    "   str   r0, [r1, r2]     ;\n"
	    "   bne.n 1b               ;\n"
	);
}

void armv7m_stack_initialize(void)
{
    __StackLevel = &__StackTop[0];

    /* Define a 32 byte no-access region at the bottom and top of the
     * system stack to for an exception on stack over and underflow.
     */

    MPU->CTRL    = 0x00000005;
    MPU->RBAR    = ((uint32_t)&__StackLimit[0]) | 0x10;
    MPU->RASR    = 0x10050009;
    MPU->RBAR_A1 = ((uint32_t)&__StackTop[0]) | 0x11;
    MPU->RASR_A1 = 0x10050009;

    armv7m_stack_fill();
}
