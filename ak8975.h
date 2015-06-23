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

#if !defined(_AK8975_H)
#define _AK8975_H

#include "mpu6050.h"

#define AK8975_CONFIG_ORIENTATION       (MPU6050_CONFIG_ORIENTATION)    /* MPU-9150 has the same orientation */

#define AK8975_CONFIG_TRIGGER_INTERVAL  10
#define AK8975_CONFIG_CONVERT_OFFSET    10
#define AK8975_CONFIG_SAMPLE_COUNT      5

#define AK8975_FLAG_I2C_ERROR           0x20
#define AK8975_FLAG_FIFO_OVERFLOW       0x40
#define AK8975_FLAG_MAG_INVALID         0x80

typedef struct _ak8975_fifo_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    int16_t           mx;
    int16_t           my;
    int16_t           mz;
    uint16_t          reserved;
} ak8975_fifo_entry_t;

#define AK8975_I2C_ADDRESS              0x0c

#define AK8975_RA_WIA                   0x00
#define AK8975_RA_INFO                  0x01
#define AK8975_RA_ST1                   0x02
#define AK8975_RA_HXL                   0x03
#define AK8975_RA_HXH                   0x04
#define AK8975_RA_HYL                   0x05
#define AK8975_RA_HYH                   0x06
#define AK8975_RA_HZL                   0x07
#define AK8975_RA_HZH                   0x08
#define AK8975_RA_ST2                   0x09
#define AK8975_RA_CNTL                  0x0a
#define AK8975_RA_ASTC                  0x0c
#define AK8975_RA_I2CDIS                0x0f
#define AK8975_RA_ASAX                  0x10
#define AK8975_RA_ASAY                  0x11
#define AK8975_RA_ASAZ                  0x12

#define AK8975_ST1_DRDY                 0x01
#define AK8975_ST2_DERR                 0x08
#define AK8975_ST2_HOFL                 0x10
#define AK8975_CNTL_MODE_PWR_DOWN       0x00
#define AK8975_CNTL_MODE_SINGLE_MEAS    0x01
#define AK8975_CNTL_MODE_SELF_TEST      0x08
#define AK8975_CNTL_MODE_FUSE_ROM       0x0f
#define AK8975_ASTC_SELF                0x40
#define AK8975_I2CDIS_I2CDIS            0x01

extern int ak8975_int_callback(void);
extern void ak8975_initialize(void);

#endif /* _AK8975_H */

