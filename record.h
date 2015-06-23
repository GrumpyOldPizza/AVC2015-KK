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

#if !defined(_RECORD_H)
#define _RECORD_H

#define RECORD_STATE_NONE          0
#define RECORD_STATE_OPEN          1
#define RECORD_STATE_CLOSED        2
#define RECORD_STATE_FAULT         3

#define RECORD_SEQUENCE_ACTIVE       0x80000000

#define RECORD_DATA_SIZE 16384

#define RECORD_TYPE_NONE             0
#define RECORD_TYPE_MPU6050          1
#define RECORD_TYPE_AK8975           2
#define RECORD_TYPE_HMC5883          3
#define RECORD_TYPE_RPM              4
#define RECORD_TYPE_PPS              5
#define RECORD_TYPE_GPS_LOCATION     6
#define RECORD_TYPE_GPS_SATELLITES   7
#define RECORD_TYPE_BUTTON           8
#define RECORD_TYPE_SWITCH           9
#define RECORD_TYPE_RECEIVER         10
#define RECORD_TYPE_RESERVED_11      11
#define RECORD_TYPE_PROFILE          12
#define RECORD_TYPE_EKF              13
#define RECORD_TYPE_NAVIGATION       14
#define RECORD_TYPE_GUIDANCE         15
#define RECORD_TYPE_CONTROL          16
#define RECORD_TYPE_NUM              17

typedef struct _record_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
} record_entry_t;

extern void record_start(void);
extern void record_stop(void);
extern void record_enter(const void *data, unsigned int size);
extern void record_enter_extended(uint8_t type, uint8_t flags, uint64_t tick, const void *data, unsigned int size);
extern uint32_t record_sequence(void);
extern int record_flush(void);
extern void record_initialize(void);

#endif /* _RECORD_H */

