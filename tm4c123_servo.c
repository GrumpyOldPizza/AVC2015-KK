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

#define TM4C123_SERVO_FRAME_TIME     16000
#define TM4C123_SERVO_PULSE_LEFT     1000
#define TM4C123_SERVO_PULSE_CENTER   1500
#define TM4C123_SERVO_PULSE_RIGHT    2000

/* This is kind of hacky. If we need to enter failsafe mode
 * anything can be corrupted. Hence a hardcoded value for
 * the SystemCoreClock is needed ... However it might well
 * be that the MCU core clock had been changed ... which means
 * at the end of the day, a FAILSAFE situation needs to reset
 * and then enter FAILSAFE mode ...
 */
#define TM4C123_SERVO_FAILSAFE_SCALE (80000000 / 1000000)

ARMV7M_SAFE_CREATE(uint8_t, tm4c123_servo_mode);
ARMV7M_SAFE_CREATE(uint32_t, tm4c123_servo_ch1out_offset);
ARMV7M_SAFE_CREATE(uint32_t, tm4c123_servo_ch2out_offset);
ARMV7M_SAFE_CREATE(uint32_t, tm4c123_servo_scale);

void WTIMER0B_IRQHandler(void)
{
    uint32_t scale, ch1out_offset, ch2out_offset;
    uint16_t steering, throttle;
    uint8_t mode;

    ARMV7M_PROFILE_TAG_PUSH(SERVO);

    if (WTIMER0->MIS & TIMER_MIS_CBEMIS)
    {
	WTIMER0->ICR = TIMER_ICR_CBECINT;

	ARMV7M_SAFE_READ(uint32_t, tm4c123_servo_scale, scale); 
	ARMV7M_SAFE_READ(uint32_t, tm4c123_servo_ch2out_offset, ch2out_offset);

	// if (GPIOC->DATA & GPIO_PIN_5)
	if (WTIMER0->TBR > ((scale * TM4C123_SERVO_FRAME_TIME) - ch2out_offset))
	{
	    /* On a rising edge compute steering and throttle, and
	     * update WTIMER0B with the pulse..
	     */

	    ARMV7M_SAFE_READ(uint8_t, tm4c123_servo_mode, mode); 

	    control_routine(mode, &steering, &throttle);

	    ch1out_offset = scale * steering;
	    ch2out_offset = scale * throttle;

	    ARMV7M_SAFE_WRITE(uint32_t, tm4c123_servo_ch1out_offset, ch1out_offset);
	    ARMV7M_SAFE_WRITE(uint32_t, tm4c123_servo_ch2out_offset, ch2out_offset);

	    WTIMER0->TBMATCHR = (scale * TM4C123_SERVO_FRAME_TIME) - ch2out_offset;
	}
	else
	{
	    /* On a falling edge load WTIMER0A as a slave pulse staggered right
	     * after the CH2OUT pulse.
	     */
	    ARMV7M_SAFE_READ(uint32_t, tm4c123_servo_ch1out_offset, ch1out_offset);

	    WTIMER0->TAILR    = 0xffffffff;
	    WTIMER0->TAMATCHR = 0xffffffff - ch1out_offset;

	    if (ch1out_offset)
	    {
		WTIMER0->CTL = TIMER_CTL_TBEVENT_BOTH | TIMER_CTL_TBEN | TIMER_CTL_TAEN;
	    }
	    else
	    {
		WTIMER0->CTL = TIMER_CTL_TBEVENT_BOTH | TIMER_CTL_TBEN;
	    }
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}


void tm4c123_servo_fault(void)
{
    uint32_t scale;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    /* Enable pin PC4 for GPIO Output
     */
    ROM_GPIOPinTypeGPIOOutput(GPIOC_BASE, TM4C123_SERVO_CH1OUT);
    
    armv7m_bitband_peripheral_write(&GPIOC->DATA, 4, 0);
    
    /* Enable pin PC5 for WTIMER0 WT0CCP1
     */
    ROM_GPIOPinConfigure(GPIO_PC5_WT0CCP1);
    ROM_GPIOPinTypeTimer(GPIOC_BASE, GPIO_PIN_5);
    
    scale = TM4C123_SERVO_FAILSAFE_SCALE;
    
    WTIMER0->CTL      = 0;
    WTIMER0->CFG      = TIMER_CFG_16_BIT;
    WTIMER0->TBMR     = TIMER_TBMR_TBPWMIE | TIMER_TBMR_TBAMS | TIMER_TBMR_TBMR_PERIOD;    
    WTIMER0->IMR      = 0;
    WTIMER0->ICR      = 0;
    WTIMER0->TBILR    = (scale * TM4C123_SERVO_FRAME_TIME);
    WTIMER0->TBMATCHR = (scale * (TM4C123_SERVO_FRAME_TIME - TM4C123_SERVO_PULSE_CENTER));
    WTIMER0->TBPR     = 0;
    WTIMER0->TBPMR    = 0;
    WTIMER0->CTL      = TIMER_CTL_TBEN;

    tm4c123_led(TM4C123_LED_RED);
}

void tm4c123_servo_configure(uint32_t mode)
{
    if (mode == TM4C123_SERVO_MODE_PASSTHROU)
    {
	/* AUTONOMOUS/FAILSAFE -> PASSTHROU
	 */
	ROM_GPIOPinTypeGPIOOutput(GPIOC_BASE, TM4C123_SERVO_CH1OUT);
	ROM_GPIOPinTypeGPIOOutput(GPIOC_BASE, TM4C123_SERVO_CH2OUT);

	armv7m_bitband_peripheral_write(&GPIOC->DATA, 4, 0);
    }
    else
    {
	/* PASSTHROU -> AUTONOMOUS/FAILSAFE
	 */
	ROM_GPIOPinConfigure(GPIO_PC4_WT0CCP0);
	ROM_GPIOPinTypeTimer(GPIOC_BASE, TM4C123_SERVO_CH1OUT);

	/* Enable pin PC5 for WTIMER0 WT0CCP1
	 */
	ROM_GPIOPinConfigure(GPIO_PC5_WT0CCP1);
	ROM_GPIOPinTypeTimer(GPIOC_BASE, TM4C123_SERVO_CH2OUT);
    }

    ARMV7M_SAFE_WRITE(uint8_t, tm4c123_servo_mode, mode); 
}

void tm4c123_servo_initialize(void)
{
    uint32_t scale;   

    scale = SystemCoreClock / 1000000;

    ARMV7M_SAFE_WRITE(uint32_t, tm4c123_servo_scale, scale); 

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    /* Enable pin PC4 for WTIMER0 WT0CCP0
     */
    ROM_GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    ROM_GPIOPinTypeTimer(GPIOC_BASE, GPIO_PIN_4);

    /* Enable pin PC5 for WTIMER0 WT0CCP1
     */
    ROM_GPIOPinConfigure(GPIO_PC5_WT0CCP1);
    ROM_GPIOPinTypeTimer(GPIOC_BASE, GPIO_PIN_5);

    /* Setup WTIMER0 to generate an EVENT on both edges for B.
     * In PWM mode WTIMER0 cannot generate a timout interrupt, so
     * we have to use the positive edge of the PWM signal.
     */
    WTIMER0->CTL      = 0;
    WTIMER0->CFG      = TIMER_CFG_16_BIT;
    WTIMER0->TAMR     = TIMER_TAMR_TAAMS;    
    WTIMER0->TBMR     = TIMER_TBMR_TBPWMIE | TIMER_TBMR_TBAMS | TIMER_TBMR_TBMR_PERIOD;    
    WTIMER0->IMR      = TIMER_IMR_CBEIM;
    WTIMER0->ICR      = TIMER_ICR_CBECINT;
    WTIMER0->TAILR    = 0xffffffff;
    WTIMER0->TAMATCHR = 0xffffffff;
    WTIMER0->TBILR    = (scale * TM4C123_SERVO_FRAME_TIME);
    WTIMER0->TBMATCHR = (scale * (TM4C123_SERVO_FRAME_TIME - TM4C123_SERVO_PULSE_CENTER));
    WTIMER0->TAPR     = 0;
    WTIMER0->TBPR     = 0;
    WTIMER0->TAPMR    = 0;
    WTIMER0->TBPMR    = 0;
    WTIMER0->CTL      = TIMER_CTL_TBEVENT_BOTH | TIMER_CTL_TBEN;

    ARMV7M_SAFE_WRITE(uint32_t, tm4c123_servo_ch2out_offset, (scale * TM4C123_SERVO_PULSE_CENTER));

    NVIC_SetPriority(WTIMER0B_IRQn, 5);
    NVIC_EnableIRQ(WTIMER0B_IRQn);
}
