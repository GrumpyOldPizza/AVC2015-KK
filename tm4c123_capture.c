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
#include "gps.h"
#include "mpu6050.h"

typedef struct _tm4c123_capture_device_t {
    uint32_t   clock_ltime;
    uint32_t   clock_utime;
    uint32_t   rpm_ltime;
    uint32_t   rpm_utime;
    uint32_t   pps_ltime;
    uint32_t   pps_utime;
    uint32_t   pps_count;
    uint32_t   pps_period;
    uint32_t   int1_ltime;
    uint32_t   int1_utime;
    uint32_t   int2_ltime;
    uint32_t   int2_utime;
} tm4c123_capture_device_t;

static tm4c123_capture_device_t tm4c123_capture_device;

FIFO_CREATE(sizeof(tm4c123_rpm_fifo_entry_t), 50, tm4c123_rpm_fifo);

FIFO_CREATE(sizeof(tm4c123_pps_fifo_entry_t), 1, tm4c123_pps_fifo);

void WTIMER1A_IRQHandler(void)
{
    uint32_t utime, ltime;
    tm4c123_rpm_fifo_entry_t *entry;

    ARMV7M_PROFILE_TAG_PUSH(CAPTURE);

    if (WTIMER1->MIS & TIMER_MIS_CAEMIS)
    {
        WTIMER1->ICR = TIMER_ICR_CAECINT;

	utime = WTIMER1->TAPS;
        ltime = WTIMER1->TAR;

	tm4c123_capture_device.rpm_utime = utime;
	tm4c123_capture_device.rpm_ltime = ltime;

	entry = (tm4c123_rpm_fifo_entry_t*)fifo_allocate(&tm4c123_rpm_fifo);

	if (entry)
	{
	    entry->type  = RECORD_TYPE_RPM;
	    entry->flags = 0;
	    entry->utime = tm4c123_capture_device.rpm_utime;
	    entry->ltime = tm4c123_capture_device.rpm_ltime;

	    fifo_send(&tm4c123_rpm_fifo);

	    armv7m_pendsv_pending(PENDSV_SLOT_RPM);
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}

void WTIMER1B_IRQHandler(void)
{
    uint32_t utime, ltime;
    uint64_t delta;
    tm4c123_pps_fifo_entry_t *entry;

    // ARMV7M_PROFILE_TAG_PUSH(CAPTURE);

    if (WTIMER1->MIS & TIMER_MIS_CBEMIS)
    {
        WTIMER1->ICR = TIMER_ICR_CBECINT;

	utime = WTIMER1->TBPS;
        ltime = WTIMER1->TBR;

	if ((tm4c123_capture_device.pps_utime != 0) || (tm4c123_capture_device.pps_ltime != 0))
	{
	    delta = (((((uint64_t)utime) << 32) | ((uint64_t)ltime)) - ((((uint64_t)tm4c123_capture_device.pps_utime) << 32) | ((uint64_t)tm4c123_capture_device.pps_ltime)));

	    if ((delta >= ((9 * SystemCoreClock) / 10)) && (delta <= ((11 * SystemCoreClock) / 10)))
	    {
		if (tm4c123_capture_device.pps_count <= 7)
		{
		    tm4c123_capture_device.pps_period = ((uint32_t)delta + (tm4c123_capture_device.pps_count * tm4c123_capture_device.pps_period)) / (tm4c123_capture_device.pps_count +1);
		    tm4c123_capture_device.pps_count++;
		}
		else
		{
		    tm4c123_capture_device.pps_period = ((uint32_t)delta + 7 * tm4c123_capture_device.pps_period) / 8;
		}
	    }
	}

	tm4c123_capture_device.pps_utime = utime;
	tm4c123_capture_device.pps_ltime = ltime;

	entry = (tm4c123_pps_fifo_entry_t*)fifo_allocate(&tm4c123_pps_fifo);

	if (entry)
	{
	    entry->type  = RECORD_TYPE_PPS;
	    entry->flags = 0;
	    entry->utime = tm4c123_capture_device.pps_utime;
	    entry->ltime = tm4c123_capture_device.pps_ltime;
	    entry->period = tm4c123_capture_device.pps_period;

	    fifo_send(&tm4c123_pps_fifo);

	    armv7m_pendsv_pending(PENDSV_SLOT_PPS);
	}

	gps_pps_callback(((((uint64_t)tm4c123_capture_device.pps_utime) << 32) | ((uint64_t)tm4c123_capture_device.pps_ltime)), tm4c123_capture_device.pps_period);
    }

    // ARMV7M_PROFILE_TAG_POP();
}

void WTIMER3A_IRQHandler(void)
{
    uint32_t utime, ltime;

    ARMV7M_PROFILE_TAG_PUSH(CAPTURE);

    if (WTIMER3->MIS & TIMER_MIS_CAEMIS)
    {
        WTIMER3->ICR = TIMER_ICR_CAECINT;

	utime = WTIMER3->TAPS;
        ltime = WTIMER3->TAR;

	tm4c123_capture_device.int1_utime = utime;
	tm4c123_capture_device.int1_ltime = ltime;

	mpu6050_int_callback((((uint64_t)tm4c123_capture_device.int1_utime) << 32) | ((uint64_t)tm4c123_capture_device.int1_ltime));
    }

    ARMV7M_PROFILE_TAG_POP();
}

void WTIMER3B_IRQHandler(void)
{
    uint32_t utime, ltime;

    ARMV7M_PROFILE_TAG_PUSH(CAPTURE);

    if (WTIMER3->MIS & TIMER_MIS_CBEMIS)
    {
        WTIMER3->ICR = TIMER_ICR_CBECINT;

	utime = WTIMER3->TBPS;
        ltime = WTIMER3->TBR;

	tm4c123_capture_device.int2_utime = utime;
	tm4c123_capture_device.int2_ltime = ltime;
    }

    ARMV7M_PROFILE_TAG_POP();
}

static void tm4c123_capture_rpm_pendsv_callback(void)
{
    tm4c123_rpm_fifo_entry_t *entry;
    uint64_t tick;

    while ((entry = (tm4c123_rpm_fifo_entry_t*)fifo_receive(&tm4c123_rpm_fifo)))
    {
	record_enter(entry, sizeof(tm4c123_rpm_fifo_entry_t));

	tick = ((uint64_t)entry->utime << 32) | (uint64_t)entry->ltime;

	navigation_rpm_notify(tick);

	fifo_release(&tm4c123_rpm_fifo);
    }
}

static void tm4c123_capture_pps_pendsv_callback(void)
{
    tm4c123_pps_fifo_entry_t *entry;
    uint64_t tick;

    while ((entry = (tm4c123_pps_fifo_entry_t*)fifo_receive(&tm4c123_pps_fifo)))
    {
	record_enter(entry, sizeof(tm4c123_pps_fifo_entry_t));

	tick = ((uint64_t)entry->utime << 32) | (uint64_t)entry->ltime;

	navigation_pps_notify(tick, entry->period);

	fifo_release(&tm4c123_pps_fifo);
    }
}

uint64_t tm4c123_capture_clock(void)
{
    uint32_t utime, ltime;

    do
    {
	utime = WTIMER1->TAPV;
	ltime = WTIMER1->TAV;
    }
    while (utime != WTIMER1->TAPV);

    tm4c123_capture_device.clock_utime = utime;
    tm4c123_capture_device.clock_ltime = ltime;

    return (((uint64_t)tm4c123_capture_device.clock_utime << 32) | (uint64_t)tm4c123_capture_device.clock_ltime);
}

void tm4c123_capture_initialize(void)
{
    /* WTIMER1 is used for RPM/PPS.
     */

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    /* Enable pin PC6 for WTIMER1 WT1CCP0
     */
    ROM_GPIOPinConfigure(GPIO_PC6_WT1CCP0);
    ROM_GPIOPinTypeTimer(GPIOC_BASE, GPIO_PIN_6);

    /* Enable pin PC7 for WTIMER1 WT1CCP1
     */
    ROM_GPIOPinConfigure(GPIO_PC7_WT1CCP1);
    ROM_GPIOPinTypeTimer(GPIOC_BASE, GPIO_PIN_7);

    /* Setup WTIMER1 to generate an EVENT on every positive edge for A and
     * on every positive edge for B.
     */
    WTIMER1->CTL      = 0;
    WTIMER1->CFG      = TIMER_CFG_16_BIT;  /* really 32bit for WTIMER */
    WTIMER1->TAMR     = TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP;
    WTIMER1->TBMR     = TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP;    
    WTIMER1->IMR      = TIMER_IMR_CBEIM | TIMER_IMR_CAEIM;
    WTIMER1->ICR      = TIMER_ICR_CBECINT | TIMER_ICR_CAECINT;
    WTIMER1->TAILR    = 0xffffffff;
    WTIMER1->TBILR    = 0xffffffff;
    WTIMER1->TAPR     = 0x00000000;
    WTIMER1->TBPR     = 0x00000000;
    WTIMER1->CTL      = TIMER_CTL_TBEVENT_POS | TIMER_CTL_TBEN | TIMER_CTL_TAEVENT_POS | TIMER_CTL_TAEN;

    NVIC_SetPriority(WTIMER1A_IRQn, 2);
    NVIC_EnableIRQ(WTIMER1A_IRQn);

    NVIC_SetPriority(WTIMER1B_IRQn, 2);
    NVIC_EnableIRQ(WTIMER1B_IRQn);


    /* WTIMER3 is used for INT/DRDY.
     */

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    /* Enable pin PD2 for WTIMER3 WT3CCP0
     */
    ROM_GPIOPinConfigure(GPIO_PD2_WT3CCP0);
    ROM_GPIOPinTypeTimer(GPIOD_BASE, GPIO_PIN_2);


    /* Setup WTIMER3A/B to generate an EVENT on every negative edge.
     */
    WTIMER3->CTL      = 0;
    WTIMER3->CFG      = TIMER_CFG_16_BIT;  /* really 32bit for WTIMER */
    WTIMER3->TAMR     = TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP;
    WTIMER3->TBMR     = TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP;
    WTIMER3->IMR      = TIMER_IMR_CBEIM | TIMER_IMR_CAEIM;
    WTIMER3->ICR      = TIMER_ICR_CBECINT | TIMER_ICR_CAECINT;
    WTIMER3->TAILR    = 0xffffffff;
    WTIMER3->TBILR    = 0xffffffff;
    WTIMER3->TAPR     = 0x00000000;
    WTIMER3->TBPR     = 0x00000000;
    WTIMER3->CTL      = TIMER_CTL_TBEVENT_NEG | TIMER_CTL_TBEN | TIMER_CTL_TAEVENT_NEG | TIMER_CTL_TAEN;

    NVIC_SetPriority(WTIMER3A_IRQn, 3);
    NVIC_EnableIRQ(WTIMER3A_IRQn);

    NVIC_SetPriority(WTIMER3B_IRQn, 3);
    NVIC_EnableIRQ(WTIMER3B_IRQn);

    /* At last sync all the timers, so they reference the same time base.
     */

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0); /* needed for time sync */

    WTIMER0->SYNC = TIMER_SYNC_SYNCWT1_TA | TIMER_SYNC_SYNCWT1_TB | TIMER_SYNC_SYNCWT3_TA | TIMER_SYNC_SYNCWT3_TB;
    WTIMER0->SYNC = 0;

    tm4c123_capture_device.pps_period = SystemCoreClock;
    tm4c123_capture_device.pps_count = 0;

    armv7m_pendsv_callback(PENDSV_SLOT_RPM, tm4c123_capture_rpm_pendsv_callback);
    armv7m_pendsv_callback(PENDSV_SLOT_PPS, tm4c123_capture_pps_pendsv_callback);
}
