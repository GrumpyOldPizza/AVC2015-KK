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

#define AK8975_STATE_NONE       0
#define AK8975_STATE_READY      1
#define AK8975_STATE_CONVERTING 2
#define AK8975_STATE_BUSY       3
#define AK8975_STATE_FAULT      4

typedef struct _ak8975_device_t {
    uint8_t         state;
    uint8_t         flags;
    uint8_t         wait;
    uint8_t         count;
    uint8_t         index;
    uint8_t         sens_data[8];
    uint64_t        sens_tick;
#if (AK8975_CONFIG_SAMPLE_COUNT > 1)
    int32_t         sens_average[3];
#endif /* (AK8975_CONFIG_SAMPLE_COUNT > 1) */
} ak8975_device_t;

static ak8975_device_t ak8975_device;

FIFO_CREATE(sizeof(ak8975_fifo_entry_t), 5, ak8975_fifo);

static const tm4c123_i2c_sequence_t ak8975_ini_sequence[] = {
    { I2C_SEQUENCE_WRITE | I2C_SEQUENCE_END | 1, { AK8975_RA_CNTL, AK8975_CNTL_MODE_PWR_DOWN } },
};

static void ak8975_pendsv_callback(void)
{
    float mxf, myf, mzf;
    uint64_t tick;
    ak8975_fifo_entry_t *entry;

    while ((entry = (ak8975_fifo_entry_t*)fifo_receive(&ak8975_fifo)))
    {
	record_enter(entry, sizeof(ak8975_fifo_entry_t));

	if (!(entry->flags & AK8975_FLAG_MAG_INVALID))
	{
	    tick = ((uint64_t)entry->utime << 32) | (uint64_t)entry->ltime;

#if (AK8975_CONFIG_SAMPLE_COUNT > 1)
	    if (ak8975_device.index == 0)
	    {
		ak8975_device.sens_average[0] = entry->mx;
		ak8975_device.sens_average[1] = entry->my;
		ak8975_device.sens_average[2] = entry->mz;
	    }
	    else
	    {
		ak8975_device.sens_average[0] += entry->mx;
		ak8975_device.sens_average[1] += entry->my;
		ak8975_device.sens_average[2] += entry->mz;
	    }

	    if (++ak8975_device.index == AK8975_CONFIG_SAMPLE_COUNT)
	    {
		ak8975_device.index = 0;

		mxf = ((float)ak8975_device.sens_average[0] / (float)AK8975_CONFIG_SAMPLE_COUNT) - calibration.mag_bias[0];
		myf = ((float)ak8975_device.sens_average[1] / (float)AK8975_CONFIG_SAMPLE_COUNT) - calibration.mag_bias[1];
		mzf = ((float)ak8975_device.sens_average[2] / (float)AK8975_CONFIG_SAMPLE_COUNT) - calibration.mag_bias[2];

		mxf = mxf * calibration.mag_matrix[0][0] + myf * calibration.mag_matrix[0][1] + mzf * calibration.mag_matrix[0][2];
		myf = mxf * calibration.mag_matrix[1][0] + myf * calibration.mag_matrix[1][1] + mzf * calibration.mag_matrix[1][2];
		mzf = mxf * calibration.mag_matrix[2][0] + myf * calibration.mag_matrix[2][1] + mzf * calibration.mag_matrix[2][2];
		
		navigation_mag_notify(tick, mxf, myf, mzf);
	    }
#else /* AK8975_CONFIG_SAMPLE_COUNT > 1 */

	    mxf = (float)entry->mx - calibration.mag_bias[0];
	    myf = (float)entry->my - calibration.mag_bias[1];
	    mzf = (float)entry->mz - calibration.mag_bias[2];

	    mxf = mxf * calibration.mag_matrix[0][0] + myf * calibration.mag_matrix[0][1] + mzf * calibration.mag_matrix[0][2];
	    myf = mxf * calibration.mag_matrix[1][0] + myf * calibration.mag_matrix[1][1] + mzf * calibration.mag_matrix[1][2];
	    mzf = mxf * calibration.mag_matrix[2][0] + myf * calibration.mag_matrix[2][1] + mzf * calibration.mag_matrix[2][2];

	    navigation_mag_notify(tick, mxf, myf, mzf);
#endif /* AK8975_CONFIG_SAMPLE_COUNT > 1 */
	}

	fifo_release(&ak8975_fifo);
    }
}

static void ak8975_start_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    ak8975_device.state = AK8975_STATE_CONVERTING;
    ak8975_device.wait = AK8975_CONFIG_CONVERT_OFFSET;
    ak8975_device.sens_tick = tm4c123_capture_clock();

    mpu6050_done();
}

static void ak8975_data_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    int16_t mx, my, mz;
    ak8975_fifo_entry_t *entry;

    if (status == I2C_STATUS_MASTER_DONE)
    {
	entry = (ak8975_fifo_entry_t*)fifo_allocate(&ak8975_fifo);
	  
	if (entry)
	{
	    mx = ((uint16_t)ak8975_device.sens_data[1] | ((uint16_t)ak8975_device.sens_data[2] << 8));
	    my = ((uint16_t)ak8975_device.sens_data[3] | ((uint16_t)ak8975_device.sens_data[4] << 8));
	    mz = ((uint16_t)ak8975_device.sens_data[5] | ((uint16_t)ak8975_device.sens_data[6] << 8));

	    entry->type  = RECORD_TYPE_AK8975;
	    entry->flags = ak8975_device.flags;

	    if (!(ak8975_device.sens_data[0] & AK8975_ST1_DRDY) || (ak8975_device.sens_data[7] & (AK8975_ST2_DERR | AK8975_ST2_HOFL)))
	    {
		entry->flags |= AK8975_FLAG_MAG_INVALID;
	    }

	    entry->utime = ak8975_device.sens_tick >> 32;
	    entry->ltime = ak8975_device.sens_tick & 0xffffffff;

	    ak8975_device.flags = 0;

	    /* AK8975 is NED. Simply apply ORIENTATION.
	     */
#if (AK8975_CONFIG_ORIENTATION == 0)
	    entry->mx =  mx;  entry->my =  my;  entry->mz =  mz;
#elif (AK8975_CONFIG_ORIENTATION == 1)
	    entry->mx = -my;  entry->my =  mx;  entry->mz =  mz;
#elif (AK8975_CONFIG_ORIENTATION == 2)
	    entry->mx = -mx;  entry->my = -my;  entry->mz =  mz;
#elif (AK8975_CONFIG_ORIENTATION == 3)
	    entry->mx =  my;  entry->my = -mx;  entry->mz =  mz;
#elif (AK8975_CONFIG_ORIENTATION == 4)
	    entry->mx =  mx;  entry->my = -my;  entry->mz = -mz;
#elif (AK8975_CONFIG_ORIENTATION == 5)
	    entry->mx =  my;  entry->my =  mx;  entry->mz = -mz;
#elif (AK8975_CONFIG_ORIENTATION == 6)
	    entry->mx = -mx;  entry->my =  my;  entry->mz = -mz;
#else
	    entry->mx = -my;  entry->my = -mx;  entry->mz = -mz;
#endif
	    
	    fifo_send(&ak8975_fifo);
	    
	    armv7m_pendsv_pending(PENDSV_SLOT_AK8975);
	}
	else
	{
	    ak8975_device.flags |= AK8975_FLAG_FIFO_OVERFLOW;
	}
    }
    else
    {
	ak8975_device.flags |= AK8975_FLAG_I2C_ERROR;
    }

#if (AK8975_CONFIG_TRIGGER_INTERVAL == AK8975_CONFIG_CONVERT_OFFSET)
    {
	static const uint8_t ak8975_wdata[] = { AK8975_RA_CNTL, AK8975_CNTL_MODE_SINGLE_MEAS };

	tm4c123_i2c_master_transfer(AK8975_I2C_ADDRESS, &ak8975_wdata[0], 2, NULL, 0, ak8975_start_callback);

	ak8975_device.state = AK8975_STATE_BUSY;
    }

#else /*AK8975_CONFIG_TRIGGER_INTERVAL == AK8975_CONFIG_CONVERT_OFFSET */

    ak8975_device.state = AK8975_STATE_READY;
    ak8975_device.wait = AK8975_CONFIG_TRIGGER_INTERVAL - AK8975_CONFIG_CONVERT_OFFSET;

    mpu6050_done();

#endif /*AK8975_CONFIG_TRIGGER_INTERVAL == AK8975_CONFIG_CONVERT_OFFSET */
}

int ak8975_int_callback(void)
{
    int busy = 0;
    
    if (ak8975_device.wait)
    {
	--ak8975_device.wait;
    }

    if (!ak8975_device.wait)
    {
	if (ak8975_device.state == AK8975_STATE_CONVERTING)
	{
	    static const uint8_t ak8975_wdata[] = { AK8975_RA_ST1 };
		
	    tm4c123_i2c_master_transfer(AK8975_I2C_ADDRESS, &ak8975_wdata[0], 1, &ak8975_device.sens_data[0], 8, ak8975_data_callback);

	    ak8975_device.state = AK8975_STATE_BUSY;
	    
	    busy = 2;
	}

	if (ak8975_device.state == AK8975_STATE_READY)
	{
	    static const uint8_t ak8975_wdata[] = { AK8975_RA_CNTL, AK8975_CNTL_MODE_SINGLE_MEAS };
		
	    tm4c123_i2c_master_transfer(AK8975_I2C_ADDRESS, &ak8975_wdata[0], 2, NULL, 0, ak8975_start_callback);

	    ak8975_device.state = AK8975_STATE_BUSY;
		
	    busy = 3;
	}
    }

    return busy;
}

static void ak8975_ini_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    if (status == I2C_STATUS_MASTER_DONE)
    {
	ak8975_device.state = AK8975_STATE_READY;
    }
    else
    {
	ak8975_device.state = AK8975_STATE_FAULT;
    }

    mpu6050_enable((ak8975_device.state == AK8975_STATE_READY));
}

void ak8975_initialize(void)
{
    armv7m_pendsv_callback(PENDSV_SLOT_AK8975, ak8975_pendsv_callback);

    tm4c123_i2c_master_sequence(AK8975_I2C_ADDRESS, 0, &ak8975_ini_sequence[0], ak8975_ini_callback);
}
