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

#define I2C_STATE_IDLE              0
#define I2C_STATE_START             1
#define I2C_STATE_WRITE_CONTINUE    2
#define I2C_STATE_WRITE_FINISH      3
#define I2C_STATE_READ_START        4
#define I2C_STATE_READ_CONTINUE     5
#define I2C_STATE_READ_FINISH       6

#define I2C_STEP_READ               0
#define I2C_STEP_WRITE              1

typedef struct _tm4c123_i2c_device_t {
    volatile uint8_t                state;
    uint8_t                         address;
    const uint8_t                   *wdata;
    volatile uint32_t               wcount;
    uint8_t                         *rdata;
    volatile uint32_t               rcount;
    uint8_t                         *cdata;
    volatile tm4c123_i2c_callback_t callback;
    const tm4c123_i2c_sequence_t    *sequence;
    uint8_t                         sstep;
    uint8_t                         saccum[2];
    uint8_t                         scount;
    uint8_t                         sdata[I2C_SEQUENCE_DATA_SIZE];
    volatile tm4c123_i2c_callback_t scallback;
    uint32_t                        MCS;
} tm4c123_i2c_device_t;

static tm4c123_i2c_device_t tm4c123_i2c_device;


static void tm4c123_i2c_reset(void)
{
    uint32_t speed, tpr;

    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

    /*
     * speed = SystemCoreClock / (20 * (1 + TPR))
     * TPR = (SystemCoreClock / (20 * speed)) - 1;
     */

    speed = 400000;

    tpr = (SystemCoreClock / (20 * speed)) - 1;

    /* Enable I2C3 */
    I2C3->MCR  = I2C_MCR_MFE;
    I2C3->MTPR = tpr;
    I2C3->MIMR = I2C_MIMR_IM;
    I2C3->MICR = (I2C_MICR_CLKIC | I2C_MICR_IC);
}

/* I2C3, PD0/PD1 */
void I2C3_IRQHandler(void)
{
    uint32_t status = ~0l;

    ARMV7M_PROFILE_TAG_PUSH(I2C);

    if (tm4c123_i2c_device.state == I2C_STATE_START)
    {
	if (tm4c123_i2c_device.wcount != 0)
	{
	    I2C3->MSA = (tm4c123_i2c_device.address << 1) | 0x00;
	    I2C3->MDR = *tm4c123_i2c_device.wdata;
	
	    tm4c123_i2c_device.wdata++;
	    tm4c123_i2c_device.wcount--;
	
	    if (tm4c123_i2c_device.wcount == 0)
	    {
		if (tm4c123_i2c_device.rcount == 0)
		{
		    tm4c123_i2c_device.state = I2C_STATE_WRITE_FINISH;

		    I2C3->MCS = (I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN);
		}
		else
		{
		    tm4c123_i2c_device.state = I2C_STATE_READ_START;

		    I2C3->MCS = (I2C_MCS_START | I2C_MCS_RUN);
		}
	    }
	    else
	    {
		tm4c123_i2c_device.state = I2C_STATE_WRITE_CONTINUE;

		I2C3->MCS = (I2C_MCS_START | I2C_MCS_RUN);
	    }
	}
	else
	{
	    I2C3->MSA = (tm4c123_i2c_device.address << 1) | 0x01;
	    
	    if (tm4c123_i2c_device.rcount == 1)
	    {
		tm4c123_i2c_device.state = I2C_STATE_READ_FINISH;
		
		I2C3->MCS = (I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN);
	    }
	    else
	    {
		tm4c123_i2c_device.state = I2C_STATE_READ_CONTINUE;
		
		I2C3->MCS = (I2C_MCS_ACK | I2C_MCS_START | I2C_MCS_RUN);
	    }
	}
    }
    else
    {
	I2C3->MICR = I2C_MICR_IC;

	if (I2C3->MCS & (I2C_MCS_ERROR | I2C_MCS_ARBLST))
	{
	    if (!(I2C3->MCS & I2C_MCS_ARBLST))
	    {
		/* Need to finish the current transfer with a stop condition.
		 */
		
		if (!((tm4c123_i2c_device.state == I2C_STATE_WRITE_FINISH) || (tm4c123_i2c_device.state == I2C_STATE_READ_FINISH)))
		{
		    /* On any error but a lost arbitration, the transfer has to be
		     * finished with a STOP condition.
		     */
		    I2C3->MCS = (I2C_MCS_STOP);
		}

		if (I2C3->MCS & I2C_MCS_ADRACK)
		{
		    status = I2C_STATUS_ADDRESS_NACK;
		}
		else if (I2C3->MCS & I2C_MCS_DATACK)
		{
		    status = I2C_STATUS_DATA_NACK;
		}
		else
		{
		    status = I2C_STATUS_BUS_ERROR;
		}
	    }
	    else
	    {
		tm4c123_i2c_reset();

		status = I2C_STATUS_ARBITRATION_LOST;
	    }

	    tm4c123_i2c_device.state = I2C_STATE_IDLE;
	}
	else
	{
	    switch (tm4c123_i2c_device.state) {

	    case I2C_STATE_IDLE:
		/* This can happen if a timeout occus ...
		 */
		break;

	    case I2C_STATE_START:
		/* This cannot happen ...
		 */
		break;
	    
	    case I2C_STATE_WRITE_CONTINUE:
		I2C3->MDR = *tm4c123_i2c_device.wdata;

		tm4c123_i2c_device.wdata++;
		tm4c123_i2c_device.wcount--;

		
		if (tm4c123_i2c_device.wcount == 0)
		{
		    if (tm4c123_i2c_device.rcount == 0)
		    {
			I2C3->MCS = (I2C_MCS_STOP | I2C_MCS_RUN);
			
			tm4c123_i2c_device.state = I2C_STATE_WRITE_FINISH;
		    }
		    else
		    {
			I2C3->MCS = (I2C_MCS_RUN);
			
			tm4c123_i2c_device.state = I2C_STATE_READ_START;
		    }
		}
		else
		{
		    I2C3->MCS = (I2C_MCS_RUN);

		    tm4c123_i2c_device.state = I2C_STATE_WRITE_CONTINUE;
		}
		break;

	    case I2C_STATE_WRITE_FINISH:
		status = I2C_STATUS_MASTER_DONE;

		tm4c123_i2c_device.state = I2C_STATE_IDLE;
		break;

	    case I2C_STATE_READ_START:
	        I2C3->MSA = (tm4c123_i2c_device.address << 1) | 0x01;

		if (tm4c123_i2c_device.rcount == 1)
		{
		    I2C3->MCS = (I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN);
		    
		    tm4c123_i2c_device.state = I2C_STATE_READ_FINISH;
		}
		else
		{
		    I2C3->MCS = (I2C_MCS_ACK | I2C_MCS_START | I2C_MCS_RUN);
		    
		    tm4c123_i2c_device.state = I2C_STATE_READ_CONTINUE;
		}
		break;

	    case I2C_STATE_READ_CONTINUE:
		*tm4c123_i2c_device.rdata = I2C3->MDR;

		tm4c123_i2c_device.rdata++;
		tm4c123_i2c_device.rcount--;
		
		if (tm4c123_i2c_device.rcount == 1)
		{
		    I2C3->MCS = (I2C_MCS_STOP | I2C_MCS_RUN);
		    
		    tm4c123_i2c_device.state = I2C_STATE_READ_FINISH;
		}
		else
		{
		    I2C3->MCS = (I2C_MCS_ACK | I2C_MCS_RUN);
		    
		    tm4c123_i2c_device.state = I2C_STATE_READ_CONTINUE;
		}
		break;

	    case I2C_STATE_READ_FINISH:
		*tm4c123_i2c_device.rdata = I2C3->MDR;

		tm4c123_i2c_device.rdata++;
		tm4c123_i2c_device.rcount--;

		status = I2C_STATUS_MASTER_DONE;

		tm4c123_i2c_device.state = I2C_STATE_IDLE;
		break;
	    }
	}

	if (status != ~0l)
	{

	    tm4c123_i2c_device.wcount = 0;
	    tm4c123_i2c_device.rcount = 0;

	    if (tm4c123_i2c_device.callback != NULL)
	    {
		if (tm4c123_i2c_device.cdata != NULL)
		{
		    (*tm4c123_i2c_device.callback)(status, tm4c123_i2c_device.cdata, tm4c123_i2c_device.rdata - tm4c123_i2c_device.cdata);
		}
		else
		{
		    (*tm4c123_i2c_device.callback)(status, NULL, 0);
		}
	    }
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}

void tm4c123_i2c_master_transfer(uint8_t address, const uint8_t *wdata, uint32_t wcount, uint8_t *rdata, uint32_t rcount, tm4c123_i2c_callback_t callback)
{
    if ((rdata == NULL) && (rcount != 0))
    {
	rdata = &tm4c123_i2c_device.sdata[0];
    }

    tm4c123_i2c_device.address  = address;
    tm4c123_i2c_device.wdata    = wdata;
    tm4c123_i2c_device.wcount   = wcount;
    tm4c123_i2c_device.rdata    = rdata;
    tm4c123_i2c_device.rcount   = rcount;
    tm4c123_i2c_device.cdata    = rdata;
    tm4c123_i2c_device.callback = callback;
    tm4c123_i2c_device.state    = I2C_STATE_START;

    NVIC_SetPendingIRQ(I2C3_IRQn);
}

static void tm4c123_i2c_transfer_callback(uint32_t status, uint8_t *rdata, uint32_t rcount);

static void tm4c123_i2c_sequence_execute(void)
{
    const tm4c123_i2c_sequence_t *sequence = tm4c123_i2c_device.sequence;

    if (sequence == NULL)
    {
	(*tm4c123_i2c_device.scallback)(I2C_STATUS_MASTER_DONE, &tm4c123_i2c_device.sdata[0], tm4c123_i2c_device.scount);
    }
    else
    {
	if (sequence->mode & I2C_SEQUENCE_READ)
	{
	    tm4c123_i2c_master_transfer(tm4c123_i2c_device.address,
					&sequence->data[0], 1, &tm4c123_i2c_device.sdata[tm4c123_i2c_device.scount], sequence->data[1], &tm4c123_i2c_transfer_callback);
	}
	else if (sequence->mode & (I2C_SEQUENCE_AND | I2C_SEQUENCE_OR | I2C_SEQUENCE_WAIT | I2C_SEQUENCE_WAIT_NOT))
	{
	    tm4c123_i2c_device.sstep = I2C_STEP_READ;

	    tm4c123_i2c_master_transfer(tm4c123_i2c_device.address, &sequence->data[0], 1, &tm4c123_i2c_device.saccum[0], 1, &tm4c123_i2c_transfer_callback);
	}
	else
	{
	  tm4c123_i2c_master_transfer(tm4c123_i2c_device.address, &sequence->data[0], ((sequence->mode & I2C_SEQUENCE_COMMAND) ? 1 : 2), NULL, 0, &tm4c123_i2c_transfer_callback);
	}
    }
}

static void tm4c123_i2c_transfer_callback(uint32_t status, uint8_t *rdata, uint32_t rcount)
{
    const tm4c123_i2c_sequence_t *sequence;
    int sequence_next;

    if (status != I2C_STATUS_MASTER_DONE)
    {
	tm4c123_i2c_device.sequence = NULL;

	(*tm4c123_i2c_device.scallback)(status, &tm4c123_i2c_device.sdata[0], tm4c123_i2c_device.scount + rcount);
    }
    else
    {
	sequence = tm4c123_i2c_device.sequence;

	if ((sequence->mode & (I2C_SEQUENCE_AND | I2C_SEQUENCE_OR)) && (tm4c123_i2c_device.sstep == I2C_STEP_READ))
	{
	    if (sequence->mode & I2C_SEQUENCE_AND)
	    {
		tm4c123_i2c_device.saccum[1] = tm4c123_i2c_device.saccum[0] & sequence->data[1];
	    }
	    else
	    {
		tm4c123_i2c_device.saccum[1] = tm4c123_i2c_device.saccum[0] | sequence->data[1];
	    }

	    tm4c123_i2c_device.saccum[0] = sequence->data[0];

	    tm4c123_i2c_device.sstep = I2C_STEP_WRITE;

	    tm4c123_i2c_master_transfer(tm4c123_i2c_device.address, &tm4c123_i2c_device.saccum[0], 2, NULL, 0, &tm4c123_i2c_transfer_callback);
	}
	else
	{
	    if (sequence->mode & I2C_SEQUENCE_WAIT)
	    {
		if (sequence->mode & (I2C_SEQUENCE_WAIT_NOT & ~I2C_SEQUENCE_WAIT))
		{
		    sequence_next = ((tm4c123_i2c_device.saccum[0] & sequence->data[1]) == 0);
		}
		else
		{
		    sequence_next = ((tm4c123_i2c_device.saccum[0] & sequence->data[1]) == sequence->data[1]);
		}
	    }
	    else
	    {
		if (sequence->mode & I2C_SEQUENCE_READ)
		{
		    tm4c123_i2c_device.scount += sequence->data[1];
		}
		
		sequence_next = 1;
	    }
	    
	    if (sequence_next)
	    {
		if (sequence->mode & I2C_SEQUENCE_END)
		{
		    tm4c123_i2c_device.sequence = NULL;
		}
		else
		{
		    tm4c123_i2c_device.sequence = sequence +1;
		}
	    }
	    
	    if (sequence->mode & I2C_SEQUENCE_DELAY)
	    {
		armv7m_systick_timeout(TIMEOUT_SLOT_I2C, (sequence->mode & I2C_SEQUENCE_DELAY), tm4c123_i2c_sequence_execute);
	    }
	    else
	    {
		tm4c123_i2c_sequence_execute();
	    }
	}
    }
}

extern void tm4c123_i2c_master_sequence(uint8_t address, uint32_t delay, const tm4c123_i2c_sequence_t *sequence, tm4c123_i2c_callback_t callback)
{
    tm4c123_i2c_device.address   = address;
    tm4c123_i2c_device.scount    = 0;
    tm4c123_i2c_device.sequence  = sequence;
    tm4c123_i2c_device.scallback = callback;

    if (delay)
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_I2C, delay, tm4c123_i2c_sequence_execute);
    }
    else
    {
	tm4c123_i2c_sequence_execute();
    }
}

void tm4c123_i2c_initialize(void)
{
  uint32_t count;

    tm4c123_i2c_device.state = I2C_STATE_IDLE;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    /* Disable I2C3 */
    I2C3->MCR  = I2C_MCR_MFE;

    ROM_GPIOPinTypeGPIOOutputOD(GPIOD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeGPIOInput(GPIOD_BASE, GPIO_PIN_1);
    
    /* Delay for 10usec */
    armv7m_udelay(10);

    /* If SDA is tied low by a slave, issue clock pulses till it releases
     * the bus.
     */
    for (count = 0; count < 128; count++)
    {
	if (GPIOD->DATA & GPIO_PIN_1)
	{
	    break;
	}

        /* Set SCL to L */ 
	armv7m_bitband_peripheral_write(&GPIOD->DATA, 0, 0);
        armv7m_udelay(5);    
	
        /* Set SCL to H */ 
	armv7m_bitband_peripheral_write(&GPIOD->DATA, 0, 1);
        armv7m_udelay(5);    
    }

    ROM_GPIOPinTypeGPIOOutput(GPIOD_BASE, GPIO_PIN_1);

    armv7m_bitband_peripheral_write(&GPIOD->DATA, 0, 1);
    armv7m_bitband_peripheral_write(&GPIOD->DATA, 1, 1);
    armv7m_udelay(5);    


    /* Now SCL is H and SDA is H, so generate a STOP condition.
     */

    /* Set SCL to L */ 
    armv7m_bitband_peripheral_write(&GPIOD->DATA, 0, 0);
    armv7m_udelay(5);    

    /* Set SDA to L */ 
    armv7m_bitband_peripheral_write(&GPIOD->DATA, 1, 0);
    armv7m_udelay(5);    
    
    /* Set SCL to H */ 
    armv7m_bitband_peripheral_write(&GPIOD->DATA, 0, 1);
    armv7m_udelay(5);    

    /* Set SDA to H */ 
    armv7m_bitband_peripheral_write(&GPIOD->DATA, 1, 1);
    armv7m_udelay(5);    

    /* After this recovery, switch to regular I2C mode.
     */


    /* Enable pin PD0 for I2C3 I2C3SCL
     */
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinTypeI2CSCL(GPIOD_BASE, GPIO_PIN_0);

    /* Enable pin PD1 for I2C3 I2C3SDA
     */
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);
    ROM_GPIOPinTypeI2C(GPIOD_BASE, GPIO_PIN_1);

    tm4c123_i2c_reset();

    NVIC_SetPriority(I2C3_IRQn, 3);
    NVIC_EnableIRQ(I2C3_IRQn);
}
