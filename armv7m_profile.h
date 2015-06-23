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

#if !defined(_ARMV7M_PROFILE_H)
#define _ARMV7M_PROFILE_H

#define ARMV7M_PROFILE_TAG_OTHER      0
#define ARMV7M_PROFILE_TAG_RECEIVER   1
#define ARMV7M_PROFILE_TAG_CAPTURE    2
#define ARMV7M_PROFILE_TAG_I2C        3
#define ARMV7M_PROFILE_TAG_UART       4
#define ARMV7M_PROFILE_TAG_SERVO      5
#define ARMV7M_PROFILE_TAG_BUTTON     6
#define ARMV7M_PROFILE_TAG_SYSTICK    7
#define ARMV7M_PROFILE_TAG_CONTROL    8
#define ARMV7M_PROFILE_TAG_NAVIGATION 9
#define ARMV7M_PROFILE_TAG_RECORD     10
#define ARMV7M_PROFILE_TAG_DISPLAY    11
#define ARMV7M_PROFILE_TAG_IDLE       15
#define ARMV7M_PROFILE_TAG_COUNT      16

#define ARMV7M_PROFILE_STACK_SIZE    32

typedef struct _armv7m_profile_device_t {
    uint8_t          tag;
    uint8_t          index;
    uint8_t          stack[ARMV7M_PROFILE_STACK_SIZE];
    uint32_t         timestamp;
    uint64_t         cycles[ARMV7M_PROFILE_TAG_COUNT];
} armv7m_profile_device_t;

extern armv7m_profile_device_t armv7m_profile_device;

#define ARMV7M_PROFILE_TAG_PUSH(_tag)                                                     \
{                                                                                         \
    uint32_t __timestamp, __cycles;                                                       \
    __disable_irq();				                                          \
    __timestamp = DWT->CYCCNT;		                                                  \
    __cycles = __timestamp - armv7m_profile_device.timestamp;                             \
    armv7m_profile_device.cycles[armv7m_profile_device.tag] += __cycles;                  \
    armv7m_profile_device.stack[armv7m_profile_device.index] = armv7m_profile_device.tag; \
    armv7m_profile_device.index++;		                                          \
    armv7m_profile_device.tag = ARMV7M_PROFILE_TAG_##_tag;                                \
    armv7m_profile_device.timestamp = __timestamp;                                        \
    __enable_irq();				                                          \
}

#define ARMV7M_PROFILE_TAG_POP() {                                                        \
    uint32_t __timestamp, __cycles;                                                       \
    __disable_irq();				                                          \
    __timestamp = DWT->CYCCNT;		                                                  \
    __cycles = __timestamp - armv7m_profile_device.timestamp;                             \
    armv7m_profile_device.cycles[armv7m_profile_device.tag] += __cycles;                  \
    armv7m_profile_device.index--;		                                          \
    armv7m_profile_device.tag = armv7m_profile_device.stack[armv7m_profile_device.index]; \
    armv7m_profile_device.timestamp = __timestamp;                                        \
    __enable_irq();				                                          \
}

extern void armv7m_profile_cycles(uint64_t *cycles);
extern void armv7m_profile_initialize(void);

#endif /* _ARMV7M_PROFILE_H */
