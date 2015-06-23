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

#define MPU6050_STATE_NONE    0
#define MPU6050_STATE_READY   1
#define MPU6050_STATE_FAULT   2

typedef struct _mpu6050_device_t {
    uint32_t          status;
    uint32_t          recovery;
    volatile uint32_t busy;
    uint8_t           state;
    uint8_t           flags;
    uint8_t           sens_data[14];
    uint64_t          sens_tick;
    volatile uint8_t  int_pending;
    uint64_t          int_tick;
    uint32_t          accel_delay;
    uint32_t          gyro_delay;
} mpu6050_device_t;

static mpu6050_device_t mpu6050_device;

FIFO_CREATE(sizeof(mpu6050_fifo_entry_t), 50, mpu6050_fifo);

static const tm4c123_i2c_sequence_t mpu6050_ini_sequence[] = {
    { I2C_SEQUENCE_WRITE | 100,              { MPU6050_RA_PWR_MGMT_1,   MPU6050_DEVICE_RESET                                                } },
    { I2C_SEQUENCE_WRITE | 50,               { MPU6050_RA_USER_CTRL,    MPU6050_FIFO_RESET | MPU6050_I2C_MST_RESET | MPU6050_SIG_COND_RESET } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_PWR_MGMT_1,   MPU6050_CLKSEL_PLL_ZGYRO                                            } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_PWR_MGMT_2,   0                                                                   } },

    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_I2C_MST_CTRL,       0 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_I2C_SLV0_CTRL,      0 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_I2C_SLV1_CTRL,      0 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_I2C_SLV2_CTRL,      0 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_I2C_SLV3_CTRL,      0 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_I2C_SLV4_CTRL,      0 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_I2C_MST_DELAY_CTRL, 0 } },

    /* Setup for 1000Hz sample rate, and a 20Hz low pass filter. This results in a 8.5ms delay for accel,
     * and 8.3ms delay for gyro.
     */

    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_CONFIG,       MPU6050_EXT_SYNC_DISABLED | MPU6050_DLPF_BW_42                      } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_SMPLRT_DIV,   0                                                                   } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_GYRO_CONFIG,  MPU6050_GYRO_FS_250                                                 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_4                                                  } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_INT_PIN_CFG,  MPU6050_INT_LEVEL | MPU6050_INT_RD_CLEAR | MPU6050_I2C_BYPASS_EN    } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_INT_ENABLE,   MPU6050_DATA_RDY_EN                                                 } },
    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_FIFO_EN,      0                                                                   } },

#if (MPU6050_CONFIG_AK8975 == 1)
    { I2C_SEQUENCE_OR,                       { MPU6050_RA_YG_OFFS_TC,   MPU6050_I2C_MST_VDDIO                                               } },
#endif /* MPU6050_CONFIG_AK8975 == 1 */

    { I2C_SEQUENCE_WRITE,                    { MPU6050_RA_FIFO_EN,      0                                                                   } },

    /* Restart a possibnly stuck INT by reading out the GYRO data */
    { I2C_SEQUENCE_READ  | I2C_SEQUENCE_END, { MPU6050_RA_GYRO_XOUT_H,  6                                                                   } },
};

static void mpu6050_recovery_timeout(void);

static void mpu6050_pendsv_callback(void)
{
    float axf, ayf, azf, gxf, gyf, gzf;
    uint64_t tick;
    mpu6050_fifo_entry_t *entry;

    while ((entry = (mpu6050_fifo_entry_t*)fifo_receive(&mpu6050_fifo)))
    {
	record_enter(entry, sizeof(mpu6050_fifo_entry_t));

	tick = ((uint64_t)entry->utime << 32) | (uint64_t)entry->ltime;

	axf = (float)entry->ax * MPU6050_ACCEL_SCALE;
	ayf = (float)entry->ay * MPU6050_ACCEL_SCALE;
	azf = (float)entry->az * MPU6050_ACCEL_SCALE;

	axf = calibration.accel_scale[0] * (axf - calibration.accel_bias[0]);
	ayf = calibration.accel_scale[0] * (ayf - calibration.accel_bias[0]);
	azf = calibration.accel_scale[0] * (azf - calibration.accel_bias[0]);

	navigation_accel_notify(tick - mpu6050_device.accel_delay, axf, ayf, azf);

	gxf = (float)entry->gx * MPU6050_GYRO_SCALE;
	gyf = (float)entry->gy * MPU6050_GYRO_SCALE;
	gzf = (float)entry->gz * MPU6050_GYRO_SCALE;

	navigation_gyro_notify(tick - mpu6050_device.gyro_delay, gxf, gyf, gzf);

	fifo_release(&mpu6050_fifo);
    }
}

static void mpu6050_data_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    int16_t ax, ay, az, gx, gy, gz;
    int busy = 0;
    mpu6050_fifo_entry_t *entry;

    if (status == I2C_STATUS_MASTER_DONE)
    {
	entry = (mpu6050_fifo_entry_t*)fifo_allocate(&mpu6050_fifo);

	if (entry)
	{
	    ax = (int16_t)(((uint16_t)mpu6050_device.sens_data[0]  << 8) | (uint16_t)mpu6050_device.sens_data[1] );
	    ay = (int16_t)(((uint16_t)mpu6050_device.sens_data[2]  << 8) | (uint16_t)mpu6050_device.sens_data[3] );
	    az = (int16_t)(((uint16_t)mpu6050_device.sens_data[4]  << 8) | (uint16_t)mpu6050_device.sens_data[5] );
	    gx = (int16_t)(((uint16_t)mpu6050_device.sens_data[8]  << 8) | (uint16_t)mpu6050_device.sens_data[9] );
	    gy = (int16_t)(((uint16_t)mpu6050_device.sens_data[10] << 8) | (uint16_t)mpu6050_device.sens_data[11]);
	    gz = (int16_t)(((uint16_t)mpu6050_device.sens_data[12] << 8) | (uint16_t)mpu6050_device.sens_data[13]);

	    entry->type  = RECORD_TYPE_MPU6050;
	    entry->flags = mpu6050_device.flags;
	    entry->utime = mpu6050_device.sens_tick >> 32;
	    entry->ltime = mpu6050_device.sens_tick & 0xffffffff;

	    mpu6050_device.flags = 0;

	    /* MPU6050 is ENU. Convert to NED and apply ORIENTATION.
	     */

#if (MPU6050_CONFIG_ORIENTATION == 0)
	    entry->ax =  ay;  entry->ay =  ax;  entry->az = -az;
	    entry->gx =  gy;  entry->gy =  gx;  entry->gz = -gz;
#elif (MPU6050_CONFIG_ORIENTATION == 1)
	    entry->ax = -ax;  entry->ay =  ay;  entry->az = -az;
	    entry->gx = -gx;  entry->gy =  gy;  entry->gz = -gz;
#elif (MPU6050_CONFIG_ORIENTATION == 2)
	    entry->ax = -ay;  entry->ay = -ax;  entry->az = -az;
	    entry->gx = -gy;  entry->gy = -gx;  entry->gz = -gz;
#elif (MPU6050_CONFIG_ORIENTATION == 3)
	    entry->ax =  ax;  entry->ay = -ay;  entry->az = -az;
	    entry->gx =  gx;  entry->gy = -gy;  entry->gz = -gz;
#elif (MPU6050_CONFIG_ORIENTATION == 4)
	    entry->ax =  ay;  entry->ay = -ax;  entry->az =  az;
	    entry->gx =  gy;  entry->gy = -gx;  entry->gz =  gz;
#elif (MPU6050_CONFIG_ORIENTATION == 5)
	    entry->ax =  ax;  entry->ay =  ay;  entry->az =  az;
	    entry->gx =  gx;  entry->gy =  gy;  entry->gz =  gz;
#elif (MPU6050_CONFIG_ORIENTATION == 6)
	    entry->ax = -ay;  entry->ay =  ax;  entry->az =  az;
	    entry->gx = -gy;  entry->gy =  gx;  entry->gz =  gz;
#else
	    entry->ax = -ax;  entry->ay = -ay;  entry->az =  az;
	    entry->gx = -gx;  entry->gy = -gy;  entry->gz =  gz;
#endif

	    fifo_send(&mpu6050_fifo);

	    armv7m_pendsv_pending(PENDSV_SLOT_MPU6050);
	}
	else
	{
	    mpu6050_device.flags |= MPU6050_FLAG_FIFO_OVERFLOW;
	}

	armv7m_systick_timeout(TIMEOUT_SLOT_MPU6050, 10, mpu6050_recovery_timeout);
    }
    else
    {
	mpu6050_device.flags |= MPU6050_FLAG_I2C_ERROR;
    }

#if (MPU6050_CONFIG_AK8975 == 1)
    busy = ak8975_int_callback();
#else /* MPU6050_CONFIG_AK8975 == 1 */
#if (MPU6050_CONFIG_HMC5883 == 1)
    busy = hmc5883_int_callback();
#endif /* MPU6050_CONFIG_HMC5883 == 1 */
#endif /* MPU6050_CONFIG_AK8975 == 1 */

    /* The idea below is to allow other sensors on the I2C bus
     * to get a time-slot if "busy" is not set to true.
     */

    mpu6050_device.busy = busy;
}

void mpu6050_int_callback(uint64_t tick)
{
    if (mpu6050_device.state == MPU6050_STATE_READY) 
    {
	if (!mpu6050_device.busy)
	{
	    static const uint8_t mpu6050_ra_index = MPU6050_RA_ACCEL_XOUT_H;

	    tm4c123_i2c_master_transfer(MPU6050_I2C_ADDRESS, &mpu6050_ra_index, 1, &mpu6050_device.sens_data[0], 14, mpu6050_data_callback);

	    mpu6050_device.busy = 1;
	    mpu6050_device.sens_tick = tick;
	}
	else
	{
	    mpu6050_device.int_pending = 1;
	    mpu6050_device.int_tick = tick;
	}
    }
}

/* Every now and then the MPU6050 INT line gets stuck. To get it going again,
 * the GYRO data registers are read back with a 10ms timeout. So there can be
 * up to 10 missing samples. If 20 are missing, the IMU goes into FAULT mode.
 */

static void mpu6050_recovery_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    if (status == I2C_STATUS_MASTER_DONE)
    {
	mpu6050_device.flags |= MPU6050_FLAG_RECOVERY;

	armv7m_systick_timeout(TIMEOUT_SLOT_MPU6050, 10, mpu6050_recovery_timeout);
    }
    else
    {
	mpu6050_device.state = MPU6050_STATE_FAULT;

	control_imu_fault();
    }
}

static void mpu6050_recovery_timeout(void)
{
    uint32_t busy;

    mpu6050_device.recovery++;

    do
    {
	busy = mpu6050_device.busy;
    } 
    while (!armv7m_atomic_compare_and_exchange(&mpu6050_device.busy, busy, 1));

    if (busy)
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_MPU6050, 10, mpu6050_recovery_timeout);
    }
    else
    {
	if (mpu6050_device.flags & MPU6050_FLAG_RECOVERY)
	{
	    mpu6050_device.state = MPU6050_STATE_FAULT;

	    control_imu_fault();
	}
	else
	{
	    static const uint8_t mpu6050_ra_index = MPU6050_RA_GYRO_XOUT_H;
	
	    tm4c123_i2c_master_transfer(MPU6050_I2C_ADDRESS, &mpu6050_ra_index, 1, &mpu6050_device.sens_data[0], 6, mpu6050_recovery_callback);
	}
    }
}

static void mpu6050_ini_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
  mpu6050_device.status = status;

    if (status == I2C_STATUS_MASTER_DONE)
    {
	mpu6050_device.busy = 0;

#if (MPU6050_CONFIG_AK8975 == 1)
	ak8975_initialize();
#else /* MPU6050_CONFIG_AK8975 == 1 */
#if (MPU6050_CONFIG_HMC5883 == 1)
	hmc5883_initialize();
#endif /* MPU6050_CONFIG_HMC5883 == 1 */
#endif /* MPU6050_CONFIG_AK8975 == 1 */
    }
    else
    {
	mpu6050_device.state = MPU6050_STATE_FAULT;

	control_imu_initialize(0);
    }
}

void mpu6050_done(void)
{
    mpu6050_device.busy = 0;

    if (mpu6050_device.int_pending)
    {
	static const uint8_t mpu6050_ra_index = MPU6050_RA_ACCEL_XOUT_H;

	tm4c123_i2c_master_transfer(MPU6050_I2C_ADDRESS, &mpu6050_ra_index, 1, &mpu6050_device.sens_data[0], 14, mpu6050_data_callback);

	mpu6050_device.busy = 1;
	mpu6050_device.sens_tick = mpu6050_device.int_tick;
	mpu6050_device.int_pending = 0;
    }
}

void mpu6050_enable(int success)
{
    if (success)
    {
	mpu6050_device.state = MPU6050_STATE_READY;
    }

    control_imu_initialize(success);
}

void mpu6050_initialize(void)
{
    mpu6050_device.accel_delay = MPU6050_ACCEL_DELAY * (SystemCoreClock / 1000000);
    mpu6050_device.gyro_delay = MPU6050_GYRO_DELAY * (SystemCoreClock / 1000000);

    armv7m_pendsv_callback(PENDSV_SLOT_MPU6050, mpu6050_pendsv_callback);

    tm4c123_i2c_master_sequence(MPU6050_I2C_ADDRESS, 0, &mpu6050_ini_sequence[0], mpu6050_ini_callback);
}
