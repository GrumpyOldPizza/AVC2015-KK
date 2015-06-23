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

#if !defined(_TM4C123_I2C_H)
#define _TM4C123_I2C_H

#define I2C_STATUS_MASTER_DONE      0
#define I2C_STATUS_ARBITRATION_LOST 1
#define I2C_STATUS_ADDRESS_NACK     2
#define I2C_STATUS_DATA_NACK        3
#define I2C_STATUS_BUS_ERROR        4

#define I2C_SEQUENCE_END            0x8000
#define I2C_SEQUENCE_READ           0x4000
#define I2C_SEQUENCE_OR             0x2000
#define I2C_SEQUENCE_AND            0x1000
#define I2C_SEQUENCE_WAIT           0x0800
#define I2C_SEQUENCE_WAIT_NOT       0x0c00
#define I2C_SEQUENCE_WRITE          0x0000
#define I2C_SEQUENCE_COMMAND        0x0400
#define I2C_SEQUENCE_DELAY          0x03ff

#define I2C_SEQUENCE_DATA_SIZE      64

typedef struct _tm4c123_i2c_sequence_t {
    uint16_t         mode;
    uint8_t          data[2];           /* index/data for write, index/offset for read, index for wait/cancel */
} tm4c123_i2c_sequence_t;

typedef void (*tm4c123_i2c_callback_t)(uint32_t status, uint8_t *rdata, uint32_t rcount);

extern void I2C3_IRQHandler(void);
extern void tm4c123_i2c_initialize(void);
extern void tm4c123_i2c_master_transfer(uint8_t address, const uint8_t *wdata, uint32_t wcount, uint8_t *rdata, uint32_t rcount, tm4c123_i2c_callback_t callback);
extern void tm4c123_i2c_master_sequence(uint8_t address, uint32_t delay, const tm4c123_i2c_sequence_t *sequence, tm4c123_i2c_callback_t callback);

#endif /* _TM4C123_I2C_H */
