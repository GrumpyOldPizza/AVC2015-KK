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

#if !defined(_MPU6050_H)
#define _MPU6050_H

#define MPU6050_CONFIG_AK8975           1     /* MPU-9150              */
#define MPU6050_CONFIG_HMC5883          0     /* MPU-6050 + HMC5883    */
#define MPU6050_CONFIG_ORIENTATION      3     /* KITTY 3, SPEED BUMP 0 */

#define MPU6050_CONFIG_GYRO_COUNT       1000  /* samples for mpu6050_gyro_reset */

#define MPU6050_FLAG_I2C_ERROR          0x08
#define MPU6050_FLAG_RECOVERY           0x10
#define MPU6050_FLAG_FIFO_OVERFLOW      0x20
#define MPU6050_FLAG_GYRO_INVALID       0x40
#define MPU6050_FLAG_ACCEL_INVALID      0x80

#define MPU6050_ACCEL_DELAY             4900
#define MPU6050_ACCEL_SCALE             (4.0 / 32768.0)
#define MPU6050_GYRO_DELAY              4800
#define MPU6050_GYRO_SCALE              ((250.0 / 32768.0) * DEG2RAD)

typedef struct _mpu6050_fifo_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    int16_t           ax;
    int16_t           ay;
    int16_t           az;
    int16_t           gx;
    int16_t           gy;
    int16_t           gz;
} mpu6050_fifo_entry_t;

#define MPU6050_I2C_ADDRESS             0x68

#define MPU6050_RA_XG_OFFS_TC           0x00 /* Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD */
#define MPU6050_RA_YG_OFFS_TC           0x01 /* Bit 7 I2C_MST_VDDIO, bits 6:1 YG_OFFS_TC */
#define MPU6050_RA_ZG_OFFS_TC           0x02 /* bits 6:1 ZG_OFFS_TC */
#define MPU6050_RA_X_FINE_GAIN          0x03
#define MPU6050_RA_Y_FINE_GAIN          0x04
#define MPU6050_RA_Z_FINE_GAIN          0x05
#define MPU6050_RA_XA_OFFS_H            0x06
#define MPU6050_RA_XA_OFFS_L_TC         0x07
#define MPU6050_RA_YA_OFFS_H            0x08
#define MPU6050_RA_YA_OFFS_L_TC         0x09
#define MPU6050_RA_ZA_OFFS_H            0x0A
#define MPU6050_RA_ZA_OFFS_L_TC         0x0B
#define MPU6050_RA_PRODUCT_ID           0x0C
#define MPU6050_RA_SELF_TEST_X          0x0D
#define MPU6050_RA_SELF_TEST_Y          0x0E
#define MPU6050_RA_SELF_TEST_Z          0x0F
#define MPU6050_RA_SELF_TEST_A          0x10
#define MPU6050_RA_XG_OFFS_USRH         0x13
#define MPU6050_RA_XG_OFFS_USRL         0x14
#define MPU6050_RA_YG_OFFS_USRH         0x15
#define MPU6050_RA_YG_OFFS_USRL         0x16
#define MPU6050_RA_ZG_OFFS_USRH         0x17
#define MPU6050_RA_ZG_OFFS_USRL         0x18
#define MPU6050_RA_SMPLRT_DIV           0x19
#define MPU6050_RA_CONFIG               0x1A
#define MPU6050_RA_GYRO_CONFIG          0x1B
#define MPU6050_RA_ACCEL_CONFIG         0x1C
#define MPU6050_RA_FIFO_EN              0x23
#define MPU6050_RA_I2C_MST_CTRL         0x24
#define MPU6050_RA_I2C_SLV0_ADDR        0x25
#define MPU6050_RA_I2C_SLV0_REG         0x26
#define MPU6050_RA_I2C_SLV0_CTRL        0x27
#define MPU6050_RA_I2C_SLV1_ADDR        0x28
#define MPU6050_RA_I2C_SLV1_REG         0x29
#define MPU6050_RA_I2C_SLV1_CTRL        0x2A
#define MPU6050_RA_I2C_SLV2_ADDR        0x2B
#define MPU6050_RA_I2C_SLV2_REG         0x2C
#define MPU6050_RA_I2C_SLV2_CTRL        0x2D
#define MPU6050_RA_I2C_SLV3_ADDR        0x2E
#define MPU6050_RA_I2C_SLV3_REG         0x2F
#define MPU6050_RA_I2C_SLV3_CTRL        0x30
#define MPU6050_RA_I2C_SLV4_ADDR        0x31
#define MPU6050_RA_I2C_SLV4_REG         0x32
#define MPU6050_RA_I2C_SLV4_DO          0x33
#define MPU6050_RA_I2C_SLV4_CTRL        0x34
#define MPU6050_RA_I2C_SLV4_DI          0x35
#define MPU6050_RA_I2C_MST_STATUS       0x36
#define MPU6050_RA_INT_PIN_CFG          0x37
#define MPU6050_RA_INT_ENABLE           0x38
#define MPU6050_RA_INT_STATUS           0x3A
#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define MPU6050_RA_ACCEL_XOUT_L         0x3C
#define MPU6050_RA_ACCEL_YOUT_H         0x3D
#define MPU6050_RA_ACCEL_YOUT_L         0x3E
#define MPU6050_RA_ACCEL_ZOUT_H         0x3F
#define MPU6050_RA_ACCEL_ZOUT_L         0x40
#define MPU6050_RA_TEMP_OUT_H           0x41
#define MPU6050_RA_TEMP_OUT_L           0x42
#define MPU6050_RA_GYRO_XOUT_H          0x43
#define MPU6050_RA_GYRO_XOUT_L          0x44
#define MPU6050_RA_GYRO_YOUT_H          0x45
#define MPU6050_RA_GYRO_YOUT_L          0x46
#define MPU6050_RA_GYRO_ZOUT_H          0x47
#define MPU6050_RA_GYRO_ZOUT_L          0x48
#define MPU6050_RA_EXT_SENS_DATA_00     0x49
#define MPU6050_RA_EXT_SENS_DATA_01     0x4A
#define MPU6050_RA_EXT_SENS_DATA_02     0x4B
#define MPU6050_RA_EXT_SENS_DATA_03     0x4C
#define MPU6050_RA_EXT_SENS_DATA_04     0x4D
#define MPU6050_RA_EXT_SENS_DATA_05     0x4E
#define MPU6050_RA_EXT_SENS_DATA_06     0x4F
#define MPU6050_RA_EXT_SENS_DATA_07     0x50
#define MPU6050_RA_EXT_SENS_DATA_08     0x51
#define MPU6050_RA_EXT_SENS_DATA_09     0x52
#define MPU6050_RA_EXT_SENS_DATA_10     0x53
#define MPU6050_RA_EXT_SENS_DATA_11     0x54
#define MPU6050_RA_EXT_SENS_DATA_12     0x55
#define MPU6050_RA_EXT_SENS_DATA_13     0x56
#define MPU6050_RA_EXT_SENS_DATA_14     0x57
#define MPU6050_RA_EXT_SENS_DATA_15     0x58
#define MPU6050_RA_EXT_SENS_DATA_16     0x59
#define MPU6050_RA_EXT_SENS_DATA_17     0x5A
#define MPU6050_RA_EXT_SENS_DATA_18     0x5B
#define MPU6050_RA_EXT_SENS_DATA_19     0x5C
#define MPU6050_RA_EXT_SENS_DATA_20     0x5D
#define MPU6050_RA_EXT_SENS_DATA_21     0x5E
#define MPU6050_RA_EXT_SENS_DATA_22     0x5F
#define MPU6050_RA_EXT_SENS_DATA_23     0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO          0x63
#define MPU6050_RA_I2C_SLV1_DO          0x64
#define MPU6050_RA_I2C_SLV2_DO          0x65
#define MPU6050_RA_I2C_SLV3_DO          0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_USER_CTRL            0x6A
#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_RA_PWR_MGMT_2           0x6C
#define MPU6050_RA_FIFO_COUNTH          0x72
#define MPU6050_RA_FIFO_COUNTL          0x73
#define MPU6050_RA_FIFO_R_W             0x74
#define MPU6050_RA_WHO_AM_I             0x75

#define MPU6050_I2C_MST_VDDIO           0x80 /* YG_OFFS_TC */

#define MPU6050_EXT_SYNC_DISABLED       0x00
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x08
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x10
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x18
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x20
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x28
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x30
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x38
#define MPU6050_DLPF_BW_256             0x00
#define MPU6050_DLPF_BW_188             0x01
#define MPU6050_DLPF_BW_98              0x02
#define MPU6050_DLPF_BW_42              0x03
#define MPU6050_DLPF_BW_20              0x04
#define MPU6050_DLPF_BW_10              0x05
#define MPU6050_DLPF_BW_5               0x06 

#define MPU6050_GYRO_XA_ST_BIT          0x80
#define MPU6050_GYRO_YA_ST_BIT          0x40
#define MPU6050_GYRO_ZA_ST_BIT          0x20
#define MPU6050_GYRO_FS_250             0x00
#define MPU6050_GYRO_FS_500             0x08
#define MPU6050_GYRO_FS_1000            0x10
#define MPU6050_GYRO_FS_2000            0x18

#define MPU6050_ACCEL_XA_ST_BIT         0x80
#define MPU6050_ACCEL_YA_ST_BIT         0x40
#define MPU6050_ACCEL_ZA_ST_BIT         0x20
#define MPU6050_ACCEL_FS_2              0x00
#define MPU6050_ACCEL_FS_4              0x08
#define MPU6050_ACCEL_FS_8              0x10
#define MPU6050_ACCEL_FS_16             0x18
 
#define MPU6050_TEMP_FIFO_EN            0x80
#define MPU6050_XG_FIFO_EN              0x40
#define MPU6050_YG_FIFO_EN              0x20
#define MPU6050_ZG_FIFO_EN              0x10
#define MPU6050_ACCEL_FIFO_EN           0x08
#define MPU6050_SLV2_FIFO_EN            0x04
#define MPU6050_SLV1_FIFO_EN            0x02
#define MPU6050_SLV0_FIFO_EN            0x01 

#define MPU6050_MULT_MST_EN             0x80
#define MPU6050_WAIT_FOR_ES             0x40
#define MPU6050_SLV_3_FIFO_EN           0x20
#define MPU6050_I2C_MST_P_NSR           0x10
#define MPU6050_I2C_MST_CLK_348         0x00
#define MPU6050_I2C_MST_CLK_333         0x01
#define MPU6050_I2C_MST_CLK_320         0x02
#define MPU6050_I2C_MST_CLK_308         0x03
#define MPU6050_I2C_MST_CLK_296         0x04
#define MPU6050_I2C_MST_CLK_286         0x05
#define MPU6050_I2C_MST_CLK_276         0x06
#define MPU6050_I2C_MST_CLK_267         0x07
#define MPU6050_I2C_MST_CLK_258         0x08
#define MPU6050_I2C_MST_CLK_500         0x09
#define MPU6050_I2C_MST_CLK_471         0x0A
#define MPU6050_I2C_MST_CLK_444         0x0B
#define MPU6050_I2C_MST_CLK_421         0x0C
#define MPU6050_I2C_MST_CLK_400         0x0D
#define MPU6050_I2C_MST_CLK_381         0x0E
#define MPU6050_I2C_MST_CLK_364         0x0F

#define MPU6050_I2C_SLV0_RW             0x80
#define MPU6050_I2C_SLV0_ADDR_MASK      0x7F
#define MPU6050_I2C_SLV0_ADDR_SHIFT     0

#define MPU6050_I2C_SLV0_EN             0x80
#define MPU6050_I2C_SLV0_BYTE_SW        0x40
#define MPU6050_I2C_SLV0_REG_DIS        0x20
#define MPU6050_I2C_SLV0_GRP            0x10
#define MPU6050_I2C_SLV0_LEN_MASK       0x0F
#define MPU6050_I2C_SLV0_LEN_SHIFT      0

#define MPU6050_I2C_SLV1_RW             0x80
#define MPU6050_I2C_SLV1_ADDR_MASK      0x7F
#define MPU6050_I2C_SLV1_ADDR_SHIFT     0

#define MPU6050_I2C_SLV1_EN             0x80
#define MPU6050_I2C_SLV1_BYTE_SW        0x40
#define MPU6050_I2C_SLV1_REG_DIS        0x20
#define MPU6050_I2C_SLV1_GRP            0x10
#define MPU6050_I2C_SLV1_LEN_MASK       0x0F
#define MPU6050_I2C_SLV1_LEN_SHIFT      0

#define MPU6050_I2C_SLV2_RW             0x80
#define MPU6050_I2C_SLV2_ADDR_MASK      0x7F
#define MPU6050_I2C_SLV2_ADDR_SHIFT     0

#define MPU6050_I2C_SLV2_EN             0x80
#define MPU6050_I2C_SLV2_BYTE_SW        0x40
#define MPU6050_I2C_SLV2_REG_DIS        0x20
#define MPU6050_I2C_SLV2_GRP            0x10
#define MPU6050_I2C_SLV2_LEN_MASK       0x0F
#define MPU6050_I2C_SLV2_LEN_SHIFT      0

#define MPU6050_I2C_SLV3_RW             0x80
#define MPU6050_I2C_SLV3_ADDR_MASK      0x7F
#define MPU6050_I2C_SLV3_ADDR_SHIFT     0

#define MPU6050_I2C_SLV3_EN             0x80
#define MPU6050_I2C_SLV3_BYTE_SW        0x40
#define MPU6050_I2C_SLV3_REG_DIS        0x20
#define MPU6050_I2C_SLV3_GRP            0x10
#define MPU6050_I2C_SLV3_LEN_MASK       0x0F
#define MPU6050_I2C_SLV3_LEN_SHIFT      0

#define MPU6050_I2C_SLV4_RW             0x80
#define MPU6050_I2C_SLV4_ADDR_MASK      0x7F
#define MPU6050_I2C_SLV4_ADDR_SHIFT     0

#define MPU6050_I2C_SLV4_EN             0x80
#define MPU6050_I2C_SLV4_INT_EN         0x40
#define MPU6050_I2C_SLV4_REG_DIS        0x20
#define MPU6050_I2C_SLV4_MST_DLY_MASK   0x1F
#define MPU6050_I2C_SLV4_MST_DLY_SHIFT  0

#define MPU6050_PASS_THROUGH            0x80
#define MPU6050_I2C_SLV4_DONE           0x40
#define MPU6050_I2C_LOST_ARB            0x20
#define MPU6050_I2C_SLV4_NACK           0x10
#define MPU6050_I2C_SLV3_NACK           0x08
#define MPU6050_I2C_SLV2_NACK           0x04
#define MPU6050_I2C_SLV1_NACK           0x02
#define MPU6050_I2C_SLV0_NACK           0x01

#define MPU6050_INT_LEVEL               0x80
#define MPU6050_INT_OPEN                0x40
#define MPU6050_LATCH_INT_EN            0x20
#define MPU6050_INT_RD_CLEAR            0x10
#define MPU6050_FSYNC_INT_LEVEL         0x08
#define MPU6050_FSYNC_INT_EN            0x04
#define MPU6050_I2C_BYPASS_EN           0x02

#define MPU6050_FIFO_OFLOW_EN           0x10
#define MPU6050_I2C_MST_INT_EN          0x08
#define MPU6050_DATA_RDY_EN             0x01

#define MPU6050_FIFO_OFLOW_INT          0x10
#define MPU6050_I2C_MST_INT_INT         0x08
#define MPU6050_DATA_RDY_INT            0x01

#define MPU6050_DELAY_ES_SHADOW         0x80
#define MPU6050_I2C_SLV4_DLY_EN         0x10
#define MPU6050_I2C_SLV3_DLY_EN         0x08
#define MPU6050_I2C_SLV2_DLY_EN         0x04
#define MPU6050_I2C_SLV1_DLY_EN         0x03
#define MPU6050_I2C_SLV0_DLY_EN         0x02

#define MPU6050_GYRO_RESET              0x04
#define MPU6050_ACCEL_RESET             0x02
#define MPU6050_TEMP_RESET              0x01

#define MPU6050_ACCEL_ON_DELAY_MASK     0x30
#define MPU6050_ACCEL_ON_DELAY_SHIFT    4

#define MPU6050_FIFO_EN                 0x40
#define MPU6050_I2C_MST_EN              0x20
#define MPU6050_I2C_IF_DIS              0x10
#define MPU6050_FIFO_RESET              0x04
#define MPU6050_I2C_MST_RESET           0x02
#define MPU6050_SIG_COND_RESET          0x01

#define MPU6050_DEVICE_RESET            0x80
#define MPU6050_SLEEP                   0x40
#define MPU6050_CYCLE                   0x20
#define MPU6050_TEMP_DIS                0x10
#define MPU6050_CLKSEL_INTERNAL         0x00
#define MPU6050_CLKSEL_PLL_XGYRO        0x01
#define MPU6050_CLKSEL_PLL_YGYRO        0x02
#define MPU6050_CLKSEL_PLL_ZGYRO        0x03
#define MPU6050_CLKSEL_PLL_EXT32K       0x04
#define MPU6050_CLKSEL_PLL_EXT19M       0x05
#define MPU6050_CLKSEL_KEEP_RESET       0x07

#define MPU6050_LP_WAKE_1_25            0x00
#define MPU6050_LP_WAKE_2_5             0x40
#define MPU6050_LP_WAKE_5               0x80
#define MPU6050_LP_WAKE_10              0xc0
#define MPU6050_STBY_XA                 0x20
#define MPU6050_STBY_YA                 0x10
#define MPU6050_STBY_ZA                 0x08
#define MPU6050_STBY_XG                 0x04
#define MPU6050_STBY_YG                 0x02
#define MPU6050_STBY_ZG                 0x01

extern void mpu6050_int_callback(uint64_t tick);
extern void mpu6050_done(void);
extern void mpu6050_enable(int success);
extern void mpu6050_initialize(void);

#endif /* _MPU6050_H */

