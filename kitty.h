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

#if !defined(_KITTY_H)
#define _KITTY_H

#define KITTY_CONFIG_ALTERNATIVE 0

#include "TM4C123.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TARGET_IS_BLIZZARD_RA3
#define PART_TM4C123GH6PM

#include "BSP/TI/TM4C123/inc/hw_types.h"
//#include "BSP/TI/TM4C123/inc/hw_memmap.h"
#include "BSP/TI/TM4C123/inc/hw_gpio.h"
#include "BSP/TI/TM4C123/inc/hw_udma.h"
#include "BSP/TI/TM4C123/inc/tm4c123gh6pm.h"
#include "BSP/TI/TM4C123/driverlib/rom.h"
#include "BSP/TI/TM4C123/driverlib/sysctl.h"
#include "BSP/TI/TM4C123/driverlib/pin_map.h"
#include "BSP/TI/TM4C123/driverlib/gpio.h"
#include "BSP/TI/TM4C123/driverlib/udma.h"

#include "armv7m_atomic.h"
#include "armv7m_core.h"
#include "armv7m_bitband.h"
#include "armv7m_exception.h"
#include "armv7m_pendsv.h"
#include "armv7m_profile.h"
#include "armv7m_safe.h"
#include "armv7m_stack.h"
#include "armv7m_systick.h"
#include "armv7m_udelay.h"

#include "tm4c123_button.h"
#include "tm4c123_capture.h"
#include "tm4c123_clib.h"
#include "tm4c123_disk.h"
#include "tm4c123_i2c.h"
#include "tm4c123_led.h"
#include "tm4c123_receiver.h"
#include "tm4c123_servo.h"
#include "tm4c123_spi.h"
#include "tm4c123_tft.h"
#include "tm4c123_uart.h"
#include "tm4c123_udma.h"

#include "ak8975.h"
#include "calibration.h"
#include "constants.h"
#include "control.h"
#include "display.h"
#include "ekf_core.h"
#include "ekf_math.h"
#include "fifo.h"
#include "gps.h"
#include "guidance.h"
#include "hmc5883.h"
#include "mission.h"
#include "mpu6050.h"
#include "navigation.h"
#include "record.h"
#include "rfat.h"
#include "tft.h"

extern void _start(void);

#endif /* _KITTY_H */
