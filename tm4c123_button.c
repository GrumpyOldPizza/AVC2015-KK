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

typedef struct _tm4c123_button_device_t {
    uint8_t  button1_state;
    uint8_t  button2_state;
    uint64_t button1_tick;
    uint64_t button2_tick;
} tm4c123_button_device_t;

static tm4c123_button_device_t tm4c123_button_device;

FIFO_CREATE(sizeof(tm4c123_button_fifo_entry_t), 8, tm4c123_button_fifo);

int tm4c123_button_event(void)
{
    tm4c123_button_fifo_entry_t *entry;
    int event = TM4C123_BUTTON_EVENT_NONE;

    if ((entry = (tm4c123_button_fifo_entry_t*)fifo_receive(&tm4c123_button_fifo)))
    {
	event = entry->flags;

	record_enter(entry, sizeof(tm4c123_button_fifo_entry_t));
	
	fifo_release(&tm4c123_button_fifo);
    }

    return event;
}

static void tm4c123_button_fifo_enqueue(uint8_t event, uint64_t tick)
{
    tm4c123_button_fifo_entry_t *entry;

    entry = (tm4c123_button_fifo_entry_t*)fifo_allocate(&tm4c123_button_fifo);

    if (entry)
    {
	entry->type  = RECORD_TYPE_BUTTON;
	entry->flags = event;
	entry->utime = tick >> 32;
	entry->ltime = tick & 0xffffffff;

	fifo_send(&tm4c123_button_fifo);
    }
}

static void tm4c123_button_release_button1_callback(void)
{
    tm4c123_button_fifo_enqueue(TM4C123_BUTTON_EVENT_BUTTON_1_TIMEOUT, tm4c123_capture_clock());
}

static void tm4c123_button_release_button2_callback(void)
{
    tm4c123_button_fifo_enqueue(TM4C123_BUTTON_EVENT_BUTTON_2_TIMEOUT, tm4c123_capture_clock());
}

static void tm4c123_button_debounce_button1_callback(void)
{
    tm4c123_button_device.button1_state = !tm4c123_button_device.button1_state;

    if (tm4c123_button_device.button1_state)
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON1_RELEASE, 2000, tm4c123_button_release_button1_callback);
    }
    else
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON1_RELEASE, 0, NULL);
    }

    tm4c123_button_fifo_enqueue((tm4c123_button_device.button1_state ? TM4C123_BUTTON_EVENT_BUTTON_1_PRESS : TM4C123_BUTTON_EVENT_BUTTON_1_RELEASE), tm4c123_button_device.button1_tick);
}

static void tm4c123_button_debounce_button2_callback(void)
{
    tm4c123_button_device.button2_state = !tm4c123_button_device.button2_state;

    if (tm4c123_button_device.button2_state)
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_RELEASE, 2000, tm4c123_button_release_button2_callback);
    }
    else
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_RELEASE, 0, NULL);
    }

    tm4c123_button_fifo_enqueue((tm4c123_button_device.button2_state ? TM4C123_BUTTON_EVENT_BUTTON_2_PRESS : TM4C123_BUTTON_EVENT_BUTTON_2_RELEASE), tm4c123_button_device.button2_tick);
}

void GPIOB_IRQHandler(void)
{
    uint8_t data;
    uint64_t tick;

    ARMV7M_PROFILE_TAG_PUSH(BUTTON);

    tick = tm4c123_capture_clock();

    data = GPIOB->DATA;
    
    if (GPIOB->MIS & TM4C123_BUTTON_2)
    {
	GPIOB->ICR = TM4C123_BUTTON_2;

	if (!(data & TM4C123_BUTTON_2))
	{
	    /* PRESS */

	    if (tm4c123_button_device.button2_state == 0)
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 20, tm4c123_button_debounce_button2_callback);

		tm4c123_button_device.button2_tick = tick;
	    }
	    else
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 0, NULL);
	    }
	}
	else
	{
	    /* RELEASE */

	    if (tm4c123_button_device.button2_state != 0)
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 20, tm4c123_button_debounce_button2_callback);

		tm4c123_button_device.button2_tick = tick;
	    }
	    else
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 0, NULL);
	    }
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}

void GPIOF_IRQHandler(void)
{
    uint8_t data;
    uint64_t tick;

    ARMV7M_PROFILE_TAG_PUSH(BUTTON);

    tick = tm4c123_capture_clock();

    data = GPIOF->DATA;
    
    if (GPIOF->MIS & TM4C123_BUTTON_1)
    {
        GPIOF->ICR = TM4C123_BUTTON_1;

	if (!(data & TM4C123_BUTTON_1))
	{
	    /* PRESS */

	    if (tm4c123_button_device.button1_state == 0)
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON1_DEBOUNCE, 20, tm4c123_button_debounce_button1_callback);

		tm4c123_button_device.button1_tick = tick;
	    }
	    else
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON1_DEBOUNCE, 0, NULL);
	    }
	}
	else
	{
	    /* RELEASE */

	    if (tm4c123_button_device.button1_state != 0)
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON1_DEBOUNCE, 20, tm4c123_button_debounce_button1_callback);

		tm4c123_button_device.button1_tick = tick;
	    }
	    else
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON1_DEBOUNCE, 0, NULL);
	    }
	}
    }

#if (KITTY_CONFIG_ALTERNATIVE == 1)
    if (GPIOF->MIS & TM4C123_BUTTON_2_ALT)
    {
        GPIOF->ICR = TM4C123_BUTTON_2_ALT;

	if (!(data & TM4C123_BUTTON_2_ALT))
	{
	    /* PRESS */

	    if (tm4c123_button_device.button2_state == 0)
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 20, tm4c123_button_debounce_button2_callback);

		tm4c123_button_device.button2_tick = tick;
	    }
	    else
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 0, NULL);
	    }
	}
	else
	{
	    /* RELEASE */

	    if (tm4c123_button_device.button2_state != 0)
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 20, tm4c123_button_debounce_button2_callback);

		tm4c123_button_device.button2_tick = tick;
	    }
	    else
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_BUTTON2_DEBOUNCE, 0, NULL);
	    }
	}
    }
#endif /* KITTY_CONFIG_ALTERNATIVE == 1 */

    ARMV7M_PROFILE_TAG_POP();
}

void tm4c123_button_initialize(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /* Enable pin PB3 for GPIOInput with weak pullup
     */
    ROM_GPIOPinTypeGPIOInput(GPIOB_BASE, GPIO_PIN_3);
    ROM_GPIOPadConfigSet(GPIOB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* Enable IRQs on both edges.
     */
    GPIOB->IS  &= ~GPIO_PIN_3;
    GPIOB->IBE |= GPIO_PIN_3;
    GPIOB->IM  |= GPIO_PIN_3;

    NVIC_SetPriority(GPIOB_IRQn, 6);
    NVIC_EnableIRQ(GPIOB_IRQn);

#if (KITTY_CONFIG_ALTERNATIVE == 1)
    (&GPIOF->LOCK)[0] = GPIO_LOCK_KEY; /* GPIOD->LOCK */
    (&GPIOF->LOCK)[1] = 0x01;          /* GPIOD->CR   */


    /* Enable pin PF0 for GPIOInput with weak pullup
     */
    ROM_GPIOPinTypeGPIOInput(GPIOF_BASE, GPIO_PIN_0);
    ROM_GPIOPadConfigSet(GPIOF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* Enable IRQs on both edges.
     */
    GPIOF->IS  &= ~GPIO_PIN_0;
    GPIOF->IBE |= GPIO_PIN_0;
    GPIOF->IM  |= GPIO_PIN_0;
#endif /* KITTY_CONFIG_ALTERNATIVE == 1 */

    /* Enable pin PF4 for GPIOInput with weak pullup
     */
    ROM_GPIOPinTypeGPIOInput(GPIOF_BASE, GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIOF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* Enable IRQs on both edges.
     */
    GPIOF->IS  &= ~GPIO_PIN_4;
    GPIOF->IBE |= GPIO_PIN_4;
    GPIOF->IM  |= GPIO_PIN_4;

    NVIC_SetPriority(GPIOF_IRQn, 6);
    NVIC_EnableIRQ(GPIOF_IRQn);
}
