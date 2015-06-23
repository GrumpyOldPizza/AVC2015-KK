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

#if !defined(_HMC5883_H)
#define _HMC5883_H

#define HMC5883_CONFIG_ORIENTATION      0

#define HMC5883_CONFIG_TRIGGER_INTERVAL 10
#define HMC5883_CONFIG_CONVERT_OFFSET   6
#define HMC5883_CONFIG_SAMPLE_COUNT     4

typedef struct _hmc5883_fifo_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    int16_t           mx;
    int16_t           my;
    int16_t           mz;
    uint16_t          reserved;
} hmc5883_fifo_entry_t;

#define HMC5883_I2C_ADDRESS             0x1e
//#define HMC5883_I2C_ADDRESS             0x1f

#define HMC5883_RA_CONFIG_A             0x00
#define HMC5883_RA_CONFIG_B             0x01
#define HMC5883_RA_MODE                 0x02
#define HMC5883_RA_MAG_XOUT_H           0x03
#define HMC5883_RA_MAG_XOUT_L           0x04
#define HMC5883_RA_MAG_ZOUT_H           0x05
#define HMC5883_RA_MAG_ZOUT_L           0x06
#define HMC5883_RA_MAG_YOUT_H           0x07
#define HMC5883_RA_MAG_YOUT_L           0x08
#define HMC5883_RA_STATUS               0x09
#define HMC5883_RA_ID_A                 0x0a
#define HMC5883_RA_ID_B                 0x0b
#define HMC5883_RA_ID_C                 0x0c
#define HMC5983_RA_TEMP_H               0x31 /* HMC5983 */
#define HMC5983_RA_TEMP_L               0x32 /* HMC5983 */

#define HMC5983_CONFIG_A_TS             0x80 /* HMC5983 */
#define HMC5883_CONFIG_A_SAMPLES_1      0x00
#define HMC5883_CONFIG_A_SAMPLES_2      0x20
#define HMC5883_CONFIG_A_SAMPLES_4      0x40
#define HMC5883_CONFIG_A_SAMPLES_8      0x60
#define HMC5883_CONFIG_A_ODR_0_75       0x00
#define HMC5883_CONFIG_A_ODR_1_5        0x04
#define HMC5883_CONFIG_A_ODR_3          0x08
#define HMC5883_CONFIG_A_ODR_7_5        0x0c
#define HMC5883_CONFIG_A_ODR_15         0x10
#define HMC5883_CONFIG_A_ODR_30         0x14
#define HMC5883_CONFIG_A_ODR_75         0x18
#define HMC5983_CONFIG_A_ODR_220        0x1c /* HMC5983 */
#define HMC5883_CONFIG_A_BIAS_NORMAL    0x00
#define HMC5883_CONFIG_A_BIAS_POSITIVE  0x01
#define HMC5883_CONFIG_A_BIAS_NEGATIVE  0x02

#define HMC5883_CONFIG_B_GAIN_0_88      0x00
#define HMC5883_CONFIG_B_GAIN_1_3       0x20
#define HMC5883_CONFIG_B_GAIN_1_9       0x40
#define HMC5883_CONFIG_B_GAIN_2_5       0x60
#define HMC5883_CONFIG_B_GAIN_4_0       0x80
#define HMC5883_CONFIG_B_GAIN_4_7       0xa0
#define HMC5883_CONFIG_B_GAIN_5_6       0xc0
#define HMC5883_CONFIG_B_GAIN_8_1       0xe0

#define HMC5883_MODE_CONTINUOUS_MEAS    0x00
#define HMC5883_MODE_SINGLE_MEAS        0x01
#define HMC5883_MODE_IDLE_0             0x02
#define HMC5883_MODE_IDLE_1             0x03

#define HMC5883_STATUS_RDY              0x01
#define HMC5883_STATUS_LOCK             0x02

extern int hmc5883_int_callback(void);
extern void hmc5883_initialize(void);

#endif /* _HMC5883_H */
