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

#define TM4C123_RECEIVER_STATE_NONE        0
#define TM4C123_RECEIVER_STATE_READY       1
#define TM4C123_RECEIVER_STATE_FAULT       2
#define TM4C123_RECEIVER_STATE_ACTIVE      3
#define TM4C123_RECEIVER_STATE_PASSTHROU   4
#define TM4C123_RECEIVER_STATE_FAILSAFE    5

ARMV7M_SAFE_CREATE(uint8_t, tm4c123_receiver_state);
ARMV7M_SAFE_CREATE(uint8_t, tm4c123_receiver_ch1in_count);
ARMV7M_SAFE_CREATE(uint8_t, tm4c123_receiver_ch2in_count);
ARMV7M_SAFE_CREATE(uint8_t, tm4c123_receiver_ch3in_switch);
ARMV7M_SAFE_CREATE(uint32_t, tm4c123_receiver_scale);
ARMV7M_SAFE_CREATE(uint64_t, tm4c123_receiver_ch1in_tick);
ARMV7M_SAFE_CREATE(uint64_t, tm4c123_receiver_ch2in_tick);
ARMV7M_SAFE_CREATE(uint64_t, tm4c123_receiver_ch3in_tick);

#define TM4C123_RECEIVER_CHANNEL_THRESHOLD 8
// #define TM4C123_RECEIVER_FRAME_THRESHOLD   (20 * 20000)
#define TM4C123_RECEIVER_FRAME_THRESHOLD   (2000000)

#define TM4C123_RECEIVER_PULSE_LEFT         950
#define TM4C123_RECEIVER_PULSE_SHORT       1250
#define TM4C123_RECEIVER_PULSE_CENTER      1500
#define TM4C123_RECEIVER_PULSE_LONG        1750
#define TM4C123_RECEIVER_PULSE_RIGHT       2050

static void tm4c123_receiver_ch1in_callback(uint8_t state, uint8_t data, uint64_t tick);
static void tm4c123_receiver_ch2in_callback(uint8_t state, uint8_t data, uint64_t tick);
static void tm4c123_receiver_ch3in_callback(uint8_t state, uint8_t data, uint64_t tick);

FIFO_CREATE(sizeof(tm4c123_switch_fifo_entry_t), 4, tm4c123_switch_fifo);

int tm4c123_switch_event(void)
{
    tm4c123_switch_fifo_entry_t *entry;
    int event = TM4C123_SWITCH_EVENT_NONE;

    if ((entry = (tm4c123_switch_fifo_entry_t*)fifo_receive(&tm4c123_switch_fifo)))
    {
	event = entry->flags;

	record_enter(entry, sizeof(tm4c123_switch_fifo_entry_t));
	
	fifo_release(&tm4c123_switch_fifo);
    }

    return event;
}

void GPIOE_IRQHandler(void)
{
    uint8_t state, data;
    uint32_t mask;
    uint64_t tick;

    ARMV7M_PROFILE_TAG_PUSH(RECEIVER);

    tick = tm4c123_capture_clock();

    /* Trick code below. The FlySky GT2B is using a staggered
     * pluse scheme. That means we may see the first IRQ before
     * the next pulse for the next channel is pending. Hence
     * we have to check and clear only the ones that had been
     * present at the top of the handler.
     */
    data = GPIOE->DATA;
    mask = GPIOE->MIS;
    
    if (mask & (TM4C123_RECEIVER_CH1IN | TM4C123_RECEIVER_CH2IN | TM4C123_RECEIVER_CH3IN))
    {
        GPIOE->ICR = mask;

	ARMV7M_SAFE_READ(uint8_t, tm4c123_receiver_state, state);

	/* Use reverse scan order for CH1IN, CH2IN, so that state transision are
	 * as short as possible (NONE -> INACTIVE -> ACTIVE).
	 */

	if (mask & TM4C123_RECEIVER_CH2IN)
	{
	    tm4c123_receiver_ch2in_callback(state, data, tick);
	}

	if (mask & TM4C123_RECEIVER_CH1IN)
	{
	    tm4c123_receiver_ch1in_callback(state, data, tick);
	}

	if (mask & TM4C123_RECEIVER_CH3IN)
	{
	    tm4c123_receiver_ch3in_callback(state, data, tick);
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}

static void tm4c123_receiver_ch1in_callback(uint8_t state, uint8_t data, uint64_t tick)
{
    uint32_t pulse, count, scale;
    uint64_t tick_ch1in;
    tm4c123_receiver_entry_payload_t payload;

    if (state == TM4C123_RECEIVER_STATE_PASSTHROU)
    {
	tm4c123_servo_passthrou_ch1out(!!(data & TM4C123_RECEIVER_CH1IN));
    }

    ARMV7M_SAFE_READ(uint32_t, tm4c123_receiver_scale, scale);

    if (data & TM4C123_RECEIVER_CH1IN)
    {
	ARMV7M_SAFE_WRITE(uint64_t, tm4c123_receiver_ch1in_tick, tick);
    }
    else
    {
	ARMV7M_SAFE_READ(uint64_t, tm4c123_receiver_ch1in_tick, tick_ch1in);
		    
	pulse = (uint32_t)(tick - tick_ch1in) / scale;

	payload.pulse = pulse; 

	record_enter_extended(RECORD_TYPE_RECEIVER, TM4C123_RECEIVER_EVENT_CH1IN, tick_ch1in, &payload, sizeof(payload));

	/* If the pulse on CH1IN is within the legal limit, wait for
	 * TM4C123_RECEIVER_CHANNEL_THRESHOLD back to back pulses before
	 * switching from INACTIVE to ACTIVE mode.
	 */
	if ((pulse >= TM4C123_RECEIVER_PULSE_LEFT) && (pulse <= TM4C123_RECEIVER_PULSE_RIGHT))
	{
	    ARMV7M_SAFE_READ(uint8_t, tm4c123_receiver_ch1in_count, count);
	    
	    if (count >= TM4C123_RECEIVER_CHANNEL_THRESHOLD)
	    {
		if (state == TM4C123_RECEIVER_STATE_READY)
		{
		    state = TM4C123_RECEIVER_STATE_ACTIVE;
				
		    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_state, state);

		    control_rcv_active();
		}
	    }
	    else
	    {
		count++;
		
		ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_ch1in_count, count);
	    }
	}
	else
	{
	    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_ch1in_count, 0);
	}

	/* ### log here CH1IN, "state", "tick", "pulse" */
    }
}

static void tm4c123_receiver_ch2in_callback(uint8_t state, uint8_t data, uint64_t tick)
{
    uint32_t pulse, count, scale;
    uint64_t tick_ch1in, tick_ch2in;
    tm4c123_receiver_entry_payload_t payload;

    if (state == TM4C123_RECEIVER_STATE_PASSTHROU)
    {
	tm4c123_servo_passthrou_ch2out(!!(data & TM4C123_RECEIVER_CH2IN));
    }

    ARMV7M_SAFE_READ(uint32_t, tm4c123_receiver_scale, scale);

    if (data & TM4C123_RECEIVER_CH2IN)
    {
	ARMV7M_SAFE_WRITE(uint64_t, tm4c123_receiver_ch2in_tick, tick);

	/* In ACTIVE mode check if more than a time of TM4C123_RECEIVER_FRAME_THRESHOLD 
	 * has elapsed on CH1IN. If so switch to FAILSAFE mode, as the signal on CH1IN
	 * is turned off when the receiver loses the signal.
	 */
	if (state == TM4C123_RECEIVER_STATE_ACTIVE)
	{
	    ARMV7M_SAFE_READ(uint64_t, tm4c123_receiver_ch1in_tick, tick_ch1in);

	    if (((uint32_t)(tick - tick_ch1in) / scale) >= TM4C123_RECEIVER_FRAME_THRESHOLD)
	    {
		state = TM4C123_RECEIVER_STATE_FAILSAFE;
		
		ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_state, state);
		
		tm4c123_servo_configure(TM4C123_SERVO_MODE_FAILSAFE);
	    }
	}
    }
    else
    {
	ARMV7M_SAFE_READ(uint64_t, tm4c123_receiver_ch2in_tick, tick_ch2in);
		    
	pulse = (uint32_t)(tick - tick_ch2in) / scale;

	payload.pulse = pulse; 

	record_enter_extended(RECORD_TYPE_RECEIVER, TM4C123_RECEIVER_EVENT_CH2IN, tick_ch2in, &payload, sizeof(payload));

	/* If the pulse on CH2IN is within the legal limit, wait for
	 * TM4C123_RECEIVER_CHANNEL_THRESHOLD back to back pulses before
	 * switching from NONE to INACTIVE mode.
	 */
	if ((pulse >= TM4C123_RECEIVER_PULSE_LEFT) && (pulse <= TM4C123_RECEIVER_PULSE_RIGHT))
	{
	    /* In ACTIVE state if CH2IN is outside the center band, enter PASSTHROU mode.
	     */
	    if ((state == TM4C123_RECEIVER_STATE_ACTIVE) && ((pulse <= TM4C123_RECEIVER_PULSE_SHORT) || (pulse >= TM4C123_RECEIVER_PULSE_LONG)))
	    {
		state = TM4C123_RECEIVER_STATE_PASSTHROU;
		
		ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_state, state);
		
		tm4c123_servo_configure(TM4C123_SERVO_MODE_PASSTHROU);
		
		tm4c123_servo_passthrou_ch2out(!!(data & TM4C123_RECEIVER_CH2IN));
	    }
	    else
	    {
		ARMV7M_SAFE_READ(uint8_t, tm4c123_receiver_ch2in_count, count);
		
		if (count >= TM4C123_RECEIVER_CHANNEL_THRESHOLD)
		{
		    if (state == TM4C123_RECEIVER_STATE_NONE)
		    {
		        armv7m_systick_timeout(TIMEOUT_SLOT_RECEIVER, 0, NULL);

			state = TM4C123_RECEIVER_STATE_READY;
			
			ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_state, state);

			control_rcv_initialize(1);
		    }
		}
		else
		{
		    count++;
		    
		    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_ch2in_count, count);
		}
	    }
	}
	else
	{
	    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_ch2in_count, 0);
	}

	/* ### log here CH2IN, "state", "tick", "pulse" */
    }
}

static void tm4c123_receiver_ch3in_callback(uint8_t state, uint8_t data, uint64_t tick)
{
    uint8_t ch3in_switch;
    uint32_t pulse, scale;
    uint64_t tick_ch3in;
    tm4c123_switch_fifo_entry_t *entry;
    tm4c123_receiver_entry_payload_t payload;

    ARMV7M_SAFE_READ(uint32_t, tm4c123_receiver_scale, scale);

    if (data & TM4C123_RECEIVER_CH3IN)
    {
	ARMV7M_SAFE_WRITE(uint64_t, tm4c123_receiver_ch3in_tick, tick);
    }
    else
    {
	ARMV7M_SAFE_READ(uint64_t, tm4c123_receiver_ch3in_tick, tick_ch3in);
		    
	pulse = (uint32_t)(tick - tick_ch3in) / scale;

	payload.pulse = pulse; 

	record_enter_extended(RECORD_TYPE_RECEIVER, TM4C123_RECEIVER_EVENT_CH3IN, tick_ch3in, &payload, sizeof(payload));

	/* If the pulse on CH3IN is within the legal limit, update the switch logic.
	 */
	if ((pulse >= TM4C123_RECEIVER_PULSE_LEFT) && (pulse <= TM4C123_RECEIVER_PULSE_RIGHT))
	{
	    ARMV7M_SAFE_READ(uint8_t, tm4c123_receiver_ch3in_switch, ch3in_switch);

	    if (pulse <= TM4C123_RECEIVER_PULSE_SHORT)
	    {
		if (ch3in_switch)
		{
		    ch3in_switch = !ch3in_switch;

		    if ((state == TM4C123_RECEIVER_STATE_ACTIVE) || (state == TM4C123_RECEIVER_STATE_PASSTHROU))
		    {
			entry = (tm4c123_switch_fifo_entry_t*)fifo_allocate(&tm4c123_switch_fifo);
		      
			if (entry)
			{
			    entry->type  = RECORD_TYPE_SWITCH;
			    entry->flags = TM4C123_SWITCH_EVENT_SWITCH_OFF;
			    entry->utime = tick >> 32;
			    entry->ltime = tick & 0xffffffff;
			    
			    fifo_send(&tm4c123_switch_fifo);
			}
		    }
		}
	    }
	    else if (pulse >= TM4C123_RECEIVER_PULSE_LONG)
	    {
		if (!ch3in_switch)
		{
		    ch3in_switch = !ch3in_switch;

		    if ((state == TM4C123_RECEIVER_STATE_ACTIVE) || (state == TM4C123_RECEIVER_STATE_PASSTHROU))
		    {
			entry = (tm4c123_switch_fifo_entry_t*)fifo_allocate(&tm4c123_switch_fifo);
		      
			if (entry)
			{
			    entry->type  = RECORD_TYPE_SWITCH;
			    entry->flags = TM4C123_SWITCH_EVENT_SWITCH_ON;
			    entry->utime = tick >> 32;
			    entry->ltime = tick & 0xffffffff;
			    
			    fifo_send(&tm4c123_switch_fifo);
			}
		    }
		}
	    }

	    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_ch3in_switch, ch3in_switch);
	}
    }
}

int tm4c123_receiver_heartbeat(void)
{
    int success;
    uint8_t state;
    uint32_t scale;
    uint64_t tick, tick_ch2in;

    ARMV7M_PROFILE_TAG_PUSH(RECEIVER);

    success = 1;

    ARMV7M_SAFE_READ(uint32_t, tm4c123_receiver_scale, scale);
    ARMV7M_SAFE_READ(uint64_t, tm4c123_receiver_ch2in_tick, tick_ch2in);

    tick = tm4c123_capture_clock();

    if (((uint32_t)(tick - tick_ch2in) / scale) >= TM4C123_RECEIVER_FRAME_THRESHOLD)
    {
	ARMV7M_SAFE_READ(uint8_t, tm4c123_receiver_state, state);

	if (!((state == TM4C123_RECEIVER_STATE_NONE) || (state == TM4C123_RECEIVER_STATE_FAULT)))
	{
	    state = TM4C123_RECEIVER_STATE_FAILSAFE;
	    
	    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_state, state);
	    
	    tm4c123_servo_configure(TM4C123_SERVO_MODE_FAILSAFE);

	    success = 0;
	}
    }

    ARMV7M_PROFILE_TAG_POP();

    return success;
}

static void tm4c123_receiver_timeout(void)
{
    uint8_t state;

    __disable_irq();

    state = TM4C123_RECEIVER_STATE_FAULT;

    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_state, state);
    
    tm4c123_servo_configure(TM4C123_SERVO_MODE_PASSTHROU);

    __enable_irq();

    control_rcv_initialize(0);
}

void tm4c123_receiver_reset(void)
{
    uint8_t state;
  
    __disable_irq();

    state = TM4C123_RECEIVER_STATE_NONE;

    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_receiver_state, state);
    
    tm4c123_servo_configure(TM4C123_SERVO_MODE_AUTONOMOUS);

    __enable_irq();

    armv7m_systick_timeout(TIMEOUT_SLOT_RECEIVER, 10000, tm4c123_receiver_timeout);
}

void tm4c123_receiver_initialize(void)
{
    uint32_t scale;

    scale = SystemCoreClock / 1000000;

    ARMV7M_SAFE_WRITE(uint32_t, tm4c123_receiver_scale, scale);


    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    /* Setup PE1/PE2/PE3 as inputs for the receiver.
     */
    ROM_GPIOPinTypeGPIOInput(GPIOE_BASE, GPIO_PIN_1);
    ROM_GPIOPinTypeGPIOInput(GPIOE_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeGPIOInput(GPIOE_BASE, GPIO_PIN_3);

    GPIOE->IS  &= ~(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOE->IBE |= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOE->IM  |= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    NVIC_SetPriority(GPIOE_IRQn, 1);
    NVIC_EnableIRQ(GPIOE_IRQn);

    armv7m_systick_timeout(TIMEOUT_SLOT_RECEIVER, 10000, tm4c123_receiver_timeout);
}
