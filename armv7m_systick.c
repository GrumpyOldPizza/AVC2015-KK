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

typedef struct _armv7m_systick_device_t {
    volatile uint32_t      ltime;
    volatile uint32_t      utime;
    volatile uint32_t      mask;
    struct {
        volatile uint32_t                  wait_count;
        volatile armv7m_systick_callback_t callback;
    }                                      timeout[TIMEOUT_SLOT_NUM];
    volatile uint32_t                      thread_wait_count;
} armv7m_systick_device_t;

static armv7m_systick_device_t armv7m_systick_device;

void SysTick_Handler(void)
{
    unsigned int slot;
    uint32_t mask, wait_count;
    armv7m_systick_callback_t callback;

    ARMV7M_PROFILE_TAG_PUSH(SYSTICK);

    armv7m_systick_device.ltime += 1;

    if (armv7m_systick_device.ltime == 0) 
    {
	armv7m_systick_device.utime += 1;
    }


    mask = armv7m_systick_device.mask;

    while (mask)
    {
	slot = 31 - armv7m_clz(mask);
	mask &= ~(1 << slot);

	wait_count = armv7m_atomic_sub(&armv7m_systick_device.timeout[slot].wait_count, 1);

	if (wait_count == 0)
	{
	    armv7m_bitband_sram_write(&armv7m_systick_device.mask, slot, 0);

	    callback = armv7m_systick_device.timeout[slot].callback;

	    if (callback)
	    {
		(*callback)();
	    }
	}
    }

    wait_count = armv7m_systick_device.thread_wait_count;

    if (wait_count)
    {
	wait_count--;

	armv7m_systick_device.thread_wait_count = wait_count;
    }

    ARMV7M_PROFILE_TAG_POP();
}

void armv7m_systick_timeout(unsigned int slot, uint32_t msec, armv7m_systick_callback_t callback)
{
    armv7m_bitband_sram_write(&armv7m_systick_device.mask, slot, 0);

    armv7m_systick_device.timeout[slot].wait_count = 0;
    armv7m_systick_device.timeout[slot].callback   = callback;

    if (callback)
    {
	armv7m_systick_device.timeout[slot].wait_count = msec +1;

	armv7m_bitband_sram_write(&armv7m_systick_device.mask, slot, 1);
    }
}

uint64_t armv7m_systick_clock(void)
{
    uint32_t ltime, utime;

    do
    {
	ltime = armv7m_systick_device.ltime;
	utime = armv7m_systick_device.utime;
    }
    while (ltime != armv7m_systick_device.ltime);

    return ((uint64_t)utime << 32) | (uint64_t)ltime;
}


void armv7m_systick_mdelay(uint32_t msec)
{
    armv7m_systick_device.thread_wait_count = msec +1;

    while (armv7m_systick_device.thread_wait_count)
    {
	ARMV7M_PROFILE_TAG_PUSH(IDLE);

	__WFI();

	ARMV7M_PROFILE_TAG_POP();
    }
}

void armv7m_systick_initialize(void)
{
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->LOAD = ((SystemCoreClock / 1000) - 1);
    SysTick->VAL  = 0;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);

    NVIC_SetPriority(SysTick_IRQn, 6);
}
