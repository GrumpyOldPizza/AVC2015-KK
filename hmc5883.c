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

#define HMC5883_STATE_NONE       0
#define HMC5883_STATE_READY      1
#define HMC5883_STATE_CONVERTING 2
#define HMC5883_STATE_BUSY       3
#define HMC5883_STATE_FAULT      4

#define HMC5883_FLAG_INVALID     0x80
#define HMC5883_FLAG_ORIENTATION 0x07

typedef struct _hmc5883_device_t {
    uint32_t        status;
    uint8_t         state;
    uint8_t         wait;
    uint8_t         count;
    uint8_t         index;
    uint8_t         sens_data[6];
    uint64_t        sens_tick;
#if (HMC5883_CONFIG_SAMPLE_COUNT > 1)
    int16_t         mag_samples[HMC5883_CONFIG_SAMPLE_COUNT][3];
#endif /* (HMC5883_CONFIG_SAMPLE_COUNT > 1) */
} hmc5883_device_t;

static hmc5883_device_t hmc5883_device;

FIFO_CREATE(sizeof(hmc5883_fifo_entry_t), 5, hmc5883_fifo);

static const tm4c123_i2c_sequence_t hmc5883_ini_sequence[] = {
    { I2C_SEQUENCE_WRITE | 1,                    { HMC5883_RA_CONFIG_A, HMC5883_CONFIG_A_SAMPLES_1 | HMC5883_CONFIG_A_ODR_15 | HMC5883_CONFIG_A_BIAS_NORMAL } },
    { I2C_SEQUENCE_WRITE | 1,                    { HMC5883_RA_CONFIG_B, HMC5883_CONFIG_B_GAIN_1_9                                                           } },
    { I2C_SEQUENCE_WRITE | I2C_SEQUENCE_END | 1, { HMC5883_RA_MODE,     HMC5883_MODE_IDLE_0                                                                 } },
};

static void hmc5883_pendsv_callback(void)
{
    float mxf, myf, mzf;
    uint64_t tick;
    hmc5883_fifo_entry_t *entry;
#if (HMC5883_CONFIG_SAMPLE_COUNT > 1)
    int mxi, myi, mzi;
    unsigned int index;
#endif /* HMC5883_CONFIG_SAMPLE_COUNT > 1*/

    while ((entry = (hmc5883_fifo_entry_t*)fifo_receive(&hmc5883_fifo)))
    {
	record_enter(entry, sizeof(hmc5883_fifo_entry_t));

	if (!(entry->flags & HMC5883_FLAG_INVALID))
	{
	    tick = ((uint64_t)entry->utime << 32) | (uint64_t)entry->ltime;

#if (HMC5883_CONFIG_SAMPLE_COUNT > 1)
	    hmc5883_device.mag_samples[hmc5883_device.index][0] = entry->mx;
	    hmc5883_device.mag_samples[hmc5883_device.index][1] = entry->my;
	    hmc5883_device.mag_samples[hmc5883_device.index][2] = entry->mz;

	    if (++hmc5883_device.index == HMC5883_CONFIG_SAMPLE_COUNT)
	    {
		hmc5883_device.index = 0;
	    }

	    if (hmc5883_device.count == HMC5883_CONFIG_SAMPLE_COUNT)
	    {
		for (index = 0, mxi = 0, myi = 0, mzi = 0; index < HMC5883_CONFIG_SAMPLE_COUNT; index++)
		{
		    mxi += hmc5883_device.mag_samples[index][0];
		    myi += hmc5883_device.mag_samples[index][1];
		    mzi += hmc5883_device.mag_samples[index][2];
		}

		mxf = ((float)mxi / (float)HMC5883_CONFIG_SAMPLE_COUNT) - calibration.mag_bias[0];
		myf = ((float)myi / (float)HMC5883_CONFIG_SAMPLE_COUNT) - calibration.mag_bias[1];
		mzf = ((float)mzi / (float)HMC5883_CONFIG_SAMPLE_COUNT) - calibration.mag_bias[2];
	    }
	    else
	    {
		hmc5883_device.count++;

		mxf = (float)entry->mx - calibration.mag_bias[0];
		myf = (float)entry->my - calibration.mag_bias[1];
		mzf = (float)entry->mz - calibration.mag_bias[2];
	    }
#else /* HMC5883_CONFIG_SAMPLE_COUNT > 1 */

	    mxf = (float)entry->mx - calibration.mag_bias[0];
	    myf = (float)entry->my - calibration.mag_bias[1];
	    mzf = (float)entry->mz - calibration.mag_bias[2];

#endif /* HMC5883_CONFIG_SAMPLE_COUNT > 1 */

	    mxf = mxf * calibration.mag_matrix[0][0] + myf * calibration.mag_matrix[0][1] + mzf * calibration.mag_matrix[0][2];
	    myf = mxf * calibration.mag_matrix[1][0] + myf * calibration.mag_matrix[1][1] + mzf * calibration.mag_matrix[1][2];
	    mzf = mxf * calibration.mag_matrix[2][0] + myf * calibration.mag_matrix[2][1] + mzf * calibration.mag_matrix[2][2];

	    navigation_mag_notify(tick, mxf, myf, mzf);
	}

	fifo_release(&hmc5883_fifo);
    }
}

static void hmc5883_start_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    hmc5883_device.state = HMC5883_STATE_CONVERTING;
    hmc5883_device.wait = HMC5883_CONFIG_CONVERT_OFFSET;
    hmc5883_device.sens_tick = tm4c123_capture_clock();

    mpu6050_done();
}

static void hmc5883_data_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    int16_t mx, my, mz;
    hmc5883_fifo_entry_t *entry;

    if (status == I2C_STATUS_MASTER_DONE)
    {
	entry = (hmc5883_fifo_entry_t*)fifo_allocate(&hmc5883_fifo);
	  
	if (entry)
	{
	    mx = ((uint16_t)hmc5883_device.sens_data[1] | ((uint16_t)hmc5883_device.sens_data[0] << 8));
	    my = ((uint16_t)hmc5883_device.sens_data[5] | ((uint16_t)hmc5883_device.sens_data[4] << 8));
	    mz = ((uint16_t)hmc5883_device.sens_data[3] | ((uint16_t)hmc5883_device.sens_data[2] << 8));
	
	    entry->type  = RECORD_TYPE_HMC5883;
	    entry->flags = 0;

	    if ((mx == -4096) || (my == -4096) || (mz != -4096))
	    {
		entry->flags |= HMC5883_FLAG_INVALID;
	    }

	    entry->utime = hmc5883_device.sens_tick >> 32;
	    entry->ltime = hmc5883_device.sens_tick & 0xffffffff;

	    /* HMC5883 is ENU. Convert to NED and apply ORIENTATION.
	     */
#if (HMC5883_CONFIM_ORIENTATION == 0)
	    entry->mx =  my;  entry->my =  mx;  entry->mz = -mz;
#elif (HMC5883_CONFIM_ORIENTATION == 1)
	    entry->mx = -mx;  entry->my =  my;  entry->mz = -mz;
#elif (HMC5883_CONFIM_ORIENTATION == 2)
	    entry->mx = -my;  entry->my = -mx;  entry->mz = -mz;
#elif (HMC5883_CONFIM_ORIENTATION == 3)
	    entry->mx =  mx;  entry->my = -my;  entry->mz = -mz;
#elif (HMC5883_CONFIM_ORIENTATION == 4)
	    entry->mx =  my;  entry->my = -mx;  entry->mz =  mz;
#elif (HMC5883_CONFIM_ORIENTATION == 5)
	    entry->mx =  mx;  entry->my =  my;  entry->mz =  mz;
#elif (HMC5883_CONFIM_ORIENTATION == 6)
	    entry->mx = -my;  entry->my =  mx;  entry->mz =  mz;
#else
	    entry->mx = -mx;  entry->my = -my;  entry->mz =  mz;
#endif
	    
	    fifo_send(&hmc5883_fifo);
	    
	    armv7m_pendsv_pending(PENDSV_SLOT_HMC5883);
	}
    }


#if (HMC5883_CONFIG_TRIGGER_INTERVAL == HMC5883_CONFIG_CONVERT_OFFSET)
    {
	static const uint8_t hmc5883_wdata[2] = { HMC5883_RA_MODE, HMC5883_MODE_SINGLE_MEAS };

	tm4c123_i2c_master_transfer(HMC5883_I2C_ADDRESS, &hmc5883_wdata[0], 2, NULL, 0, hmc5883_start_callback);

	hmc5883_device.state = HMC5883_STATE_BUSY;
    }

#else /*HMC5883_CONFIG_TRIGGER_INTERVAL == HMC5883_CONFIG_CONVERT_OFFSET */

    hmc5883_device.state = HMC5883_STATE_READY;
    hmc5883_device.wait = HMC5883_CONFIG_TRIGGER_INTERVAL - HMC5883_CONFIG_CONVERT_OFFSET;

    mpu6050_done();

#endif /*HMC5883_CONFIG_TRIGGER_INTERVAL == HMC5883_CONFIG_CONVERT_OFFSET */
}

int hmc5883_int_callback(void)
{
    int busy = 0;

    if (hmc5883_device.wait)
    {
	--hmc5883_device.wait;
    }

    if (!hmc5883_device.wait)
    {
	if (hmc5883_device.state == HMC5883_STATE_CONVERTING)
	{
	    static const uint8_t hmc5883_wdata[1] = { HMC5883_RA_MAG_XOUT_H };
		
	    tm4c123_i2c_master_transfer(HMC5883_I2C_ADDRESS, &hmc5883_wdata[0], 1, &hmc5883_device.sens_data[0], 6, hmc5883_data_callback);

	    hmc5883_device.state = HMC5883_STATE_BUSY;
	    
	    busy = 2;
	}

	if (hmc5883_device.state == HMC5883_STATE_READY)
	{
	    static const uint8_t hmc5883_wdata[2] = { HMC5883_RA_MODE, HMC5883_MODE_SINGLE_MEAS };
		
	    tm4c123_i2c_master_transfer(HMC5883_I2C_ADDRESS, &hmc5883_wdata[0], 2, NULL, 0, hmc5883_start_callback);

	    hmc5883_device.state = HMC5883_STATE_BUSY;
		
	    busy = 3;
	}
    }

    return busy;
}

static void hmc5883_ini_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
  hmc5883_device.status = status;

    if (status == I2C_STATUS_MASTER_DONE)
    {
	hmc5883_device.state = HMC5883_STATE_READY;
    }
    else
    {
	hmc5883_device.state = HMC5883_STATE_FAULT;
    }

    mpu6050_enable((hmc5883_device.state == HMC5883_STATE_READY));
}

void hmc5883_initialize(void)
{
    armv7m_pendsv_callback(PENDSV_SLOT_HMC5883, hmc5883_pendsv_callback);

    tm4c123_i2c_master_sequence(HMC5883_I2C_ADDRESS, 5, &hmc5883_ini_sequence[0], hmc5883_ini_callback);
}
