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

void _start(void)
{
    __disable_irq();

    armv7m_pendsv_initialize();
    armv7m_profile_initialize();
    armv7m_stack_initialize();
    armv7m_systick_initialize();

    tm4c123_button_initialize();
    tm4c123_capture_initialize();
    tm4c123_led_initialize();
    tm4c123_i2c_initialize();
    tm4c123_receiver_initialize();
    tm4c123_servo_initialize();
    tm4c123_spi_initialize();
    tm4c123_uart_initialize();
    tm4c123_udma_initialize();

    __enable_irq();

    guidance_initialize();
    navigation_initialize();

    tft_initialize();

    tm4c123_led(TM4C123_LED_GREEN);

    mpu6050_initialize();

    gps_initialize(GPS_RESET_NONE, &control_gps_ini_callback, &gps_location_callback, &gps_satellites_callback);

    control_initialize();
    record_initialize();
    display_initialize();

    ARMV7M_PROFILE_TAG_PUSH(IDLE);

    for (;;)
    {
	if (record_flush())
	{
	    continue;
	}

	display_update();

	armv7m_stack_check();
    }

    ARMV7M_PROFILE_TAG_POP();
}
