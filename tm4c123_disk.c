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
#include "rfat_disk.h"
#include "rfat_port.h"

typedef struct _tm4c123_disk_device_t {
    uint32_t ssi_cr0;
    uint32_t ssi_cpsr;
    uint64_t timestamp; 
    uint8_t  ssi_scratch[8];
} tm4c123_disk_device_t;

static tm4c123_disk_device_t tm4c123_disk_device;


void tm4c123_disk_time_start(void)
{
    tm4c123_disk_device.timestamp = armv7m_systick_clock();
}

bool tm4c123_disk_time_elapsed(uint32_t time)
{
    return ((armv7m_systick_clock() - tm4c123_disk_device.timestamp) > time);
}

int tm4c123_disk_lock(void)
{
    tm4c123_led(TM4C123_LED_BLUE);
    
    return F_NO_ERROR;
}


void tm4c123_disk_unlock(void)
{
    tm4c123_led(TM4C123_LED_NONE);
}

bool tm4c123_disk_present(void)
{
    bool present;

#if (TM4C123_SPI_CONFIG_LEGACY == 0)
    present = !!(GPIOA->DATA & GPIO_PIN_3);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    present = !!(GPIOA->DATA & GPIO_PIN_4);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */

    return present;
}

uint32_t tm4c123_disk_mode(int mode)
{
    unsigned int n;
    uint32_t ssiclock, speed, cpsdvsr, scr;

    if (mode == RFAT_DISK_MODE_NONE)
    {
	speed = 0;

	/* Switch CS_SDCARD to be input
	 */

#if (TM4C123_SPI_CONFIG_LEGACY == 0)
	armv7m_bitband_peripheral_write(&GPIOA->DIR, 3, 0);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
	armv7m_bitband_peripheral_write(&GPIOA->DIR, 4, 0);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    }
    else
    {
	if (mode == RFAT_DISK_MODE_IDENTIFY)
	{
	    speed = 400000;

	    /* Switch CS_SDCARD to be output
	     */

#if (TM4C123_SPI_CONFIG_LEGACY == 0)
	    armv7m_bitband_peripheral_write(&GPIOA->DIR, 3, 1);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
	    armv7m_bitband_peripheral_write(&GPIOA->DIR, 4, 1);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */
	}
	else
	{
	    speed = TM4C123_DISK_SSI_SPEED;

	    tm4c123_disk_deselect();
	}

	/* 
	 * "speed" cannot be above ssiclock / 2 !
	 *
	 *     speed = (ssiclock / (cpsdvsr * (1 + scr));
	 *
	 * The idea is now to compute the minimum "cpsdvsr" that does 
	 * not overflow "scr" (0..255):
	 *
	 *     cpsdvsr * (1 + scr) = ssiclock / speed;
	 *     cpsdvsr = max(2, ((ssiclock / speed) / (1 + scr_max) +1) & ~1)) = max(2, ((ssiclock / speed) / 256 + 1) & ~1);
	 *
	 * With that a "scr" can be computed:
	 *
	 *     (1 + scr) = (ssiclock / speed) / cpsdvsr;
	 *     scr = (ssiclock / speed) / cpsdvsr -1;
	 *
	 * However this is all pretty pointless. Lets assume we have a 50MHz ssiclock:
	 *
	 *     speed  c s
	 *  25000000  2 0
	 *  12500000  2 1
	 *   8333333  2 2
	 *   6250000  2 3
	 *   5000000  2 4
	 *    400000  2 62
	 */

	ssiclock = SystemCoreClock;

	cpsdvsr = ((ssiclock / speed) / 256 +1) & ~1;
	
	if (cpsdvsr == 0)
	{
	    cpsdvsr = 2;
	}
	
	scr = (ssiclock / speed + (cpsdvsr -1)) / cpsdvsr -1;
	
	tm4c123_disk_device.ssi_cr0  = ((scr << 8) | SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8);
	tm4c123_disk_device.ssi_cpsr = cpsdvsr;
	
	while (SSI2->SR & SSI_SR_BSY) { continue; }
	
	SSI2->CR1  = 0;
	SSI2->CR0  = tm4c123_disk_device.ssi_cr0;
	SSI2->CPSR = tm4c123_disk_device.ssi_cpsr;
	SSI2->CR1  = SSI_CR1_SSE;

	speed = (ssiclock / (cpsdvsr * (1 + scr)));

	if (mode == RFAT_DISK_MODE_IDENTIFY)
	{
	    __disable_irq();

	    armv7m_bitband_peripheral_write(&GPIOB->AFSEL, 7, 0);
	    armv7m_bitband_peripheral_write(&GPIOB->DIR, 7, 1);
	    armv7m_bitband_peripheral_write(&GPIOB->DATA, 7, 1);

	    /* Here CS/MOSI are driven both to H.
	     *
	     * Specs says to issue 74 clock cycles in SPI mode while CS/MOSI are H,
	     * so simply send 10 bytes over the clock line.
	     */
    
	    for (n = 0; n < 10; n++)
	    {
		tm4c123_disk_receive();
	    }
    
	    while (SSI2->SR & SSI_SR_BSY) { continue; }

	    armv7m_bitband_peripheral_write(&GPIOB->AFSEL, 7, 1);
	}

	tm4c123_disk_select();
    }
    
    return speed;
}

void tm4c123_disk_select(void)
{
    /* Setup/Enable SPI port for shared access.
     */
    SSI2->CR0  = tm4c123_disk_device.ssi_cr0;
    SSI2->CPSR = tm4c123_disk_device.ssi_cpsr;
    SSI2->CR1  = SSI_CR1_SSE;

    /* CS output, drive CS to L */
#if (TM4C123_SPI_CONFIG_LEGACY == 0)
    armv7m_bitband_peripheral_write(&GPIOA->DATA, 3, 0);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    armv7m_bitband_peripheral_write(&GPIOA->DATA, 4, 0);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */

    /* The card will not drive DO for one more clock
     * after CS goes L, but will accept data right away.
     * The first thing after a select will be always
     * either a command (send_command), or a "Stop Token".
     * In both cases there will be a byte over the
     * bus, and hence DO will be stable.
     */
}

void tm4c123_disk_deselect(void)
{
    while (SSI2->SR & SSI_SR_BSY) { continue; }
	
    /* CS is output, drive CS to H */
#if (TM4C123_SPI_CONFIG_LEGACY == 0)
    armv7m_bitband_peripheral_write(&GPIOA->DATA, 3, 1);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    armv7m_bitband_peripheral_write(&GPIOA->DATA, 4, 1);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */

    /* The card drives the DO line at least one more
     * clock cycle after CS goes H. Hence send
     * one extra byte over the bus, if we get
     * here while SSI was enabled.
     */
    
    tm4c123_disk_send(0x00);

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    /* Disable SPI port for shared access
     */
    SSI2->CR1 = 0;
}

/*
 * tm4c123_disk_send(uint8_t data)
 *
 * Send one byte, discard read data. The assumption
 * is that at this point both TX and RX FIFOs are
 * empty, so that a write is always possible. On
 * the read part there is a wait for the RX FIFO to
 * become not empty.
 */

void tm4c123_disk_send(uint8_t data)
{
  
    SSI2->DR = data;

    while (!(SSI2->SR & SSI_SR_RNE)) { continue; }

    SSI2->DR;
}


/*
 * tm4c123_disk_receive()
 *
 * Receive one byte, send 0xff as data. The assumption
 * is that at this point both TX and RX FIFOs are
 * empty, so that a write is always possible. On
 * the read part there is a wait for the RX FIFO to
 * become not empty.
 */

uint8_t tm4c123_disk_receive(void)
{
    SSI2->DR = 0xff;

    while (!(SSI2->SR & SSI_SR_RNE)) { continue; }

    return SSI2->DR;
}

/*
 * tm4c123_disk_send_block(const uint8_t *data)
 */

__attribute__((optimize("-O3"))) void tm4c123_disk_send_block(const uint8_t *data)
{
#if (TM4C123_DISK_CONFIG_DMA == 1)
    uint32_t crc16;

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    tm4c123_disk_device.ssi_scratch[0] = 0x00;

    /* SSI2 RX */
    tm4c123_udma_control_table[12][0] = (uint32_t)&SSI2->DR;
    tm4c123_udma_control_table[12][1] = (uint32_t)&tm4c123_disk_device.ssi_scratch[0];
    tm4c123_udma_control_table[12][2] = (UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8 |
					 UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 |
					 UDMA_CHCTL_ARBSIZE_4 | UDMA_CHCTL_XFERMODE_BASIC |
					 (511 << UDMA_CHCTL_XFERSIZE_S));

    /* SSI2 TX */
    tm4c123_udma_control_table[13][0] = (uint32_t)&data[511];
    tm4c123_udma_control_table[13][1] = (uint32_t)&SSI2->DR;
    tm4c123_udma_control_table[13][2] = (UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8 |
					 UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_SRCSIZE_8 |
					 UDMA_CHCTL_ARBSIZE_4 | UDMA_CHCTL_XFERMODE_BASIC |
					 (511 << UDMA_CHCTL_XFERSIZE_S));

    __DMB();

    UDMA->ENASET = ((1ul << 12) | (1ul << 13));

    SSI2->DMACTL = SSI_DMACTL_TXDMAE | SSI_DMACTL_RXDMAE;

    /* Overlap DMA with CRC generation.
     */
#if (RFAT_CONFIG_DISK_CRC == 1)
    crc16 = rfat_compute_crc16(data, 512);
#else /* (RFAT_CONFIG_DISK_CRC == 1) */
    crc16 = 0;
#endif /* (RFAT_CONFIG_DISK_CRC == 1) */

    ARMV7M_PROFILE_TAG_PUSH(IDLE);
    
    while ((UDMA->CHIS & ((1ul << 12) | (1ul << 13))) != ((1ul << 12) | (1ul << 13)))
    {
	continue;
    }

    UDMA->CHIS = ((1ul << 12) | (1ul << 13));

    ARMV7M_PROFILE_TAG_POP();

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->DMACTL = 0;

    tm4c123_disk_send(crc16 >> 8);
    tm4c123_disk_send(crc16 & 0xff);

#else /* TM4C123_DISK_CONFIG_DMA == 1 */
    unsigned int n;
    uint8_t data_l, data_h;
    uint32_t data16, crc16;

    crc16 = 0;

    /*
     * Idea is to stuff first up data into the TX FIFO till it's full
     * (or better said till there ae no more splots in the RX FIFO).
     * Then wait for at least one item in the RX FIFO to read it back,
     * and refill the TX FIFO. At the end, the RX FIFO is drained.
     */

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->CR1 = 0;
    SSI2->CR0 = (tm4c123_disk_device.ssi_cr0 & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_16;
    SSI2->CR1 = SSI_CR1_SSE;
    
    for (n = 0; n < TM4C123_DISK_SSI_FIFO_COUNT; n++)
    {
	data_h = *data++;
	data_l = *data++;

#if (RFAT_CONFIG_DISK_CRC == 1)
	RFAT_UPDATE_CRC16(crc16, data_h);
	RFAT_UPDATE_CRC16(crc16, data_l);
#endif /* (RFAT_CONFIG_DISK_CRC == 1) */
	
	data16 = (data_h << 8) | data_l;

	SSI2->DR = data16;
    }
    
    for (n = TM4C123_DISK_SSI_FIFO_COUNT; n < (512 / 2); n++)
    {
	data_h = *data++;
	data_l = *data++;

#if (RFAT_CONFIG_DISK_CRC == 1)
	RFAT_UPDATE_CRC16(crc16, data_h);
	RFAT_UPDATE_CRC16(crc16, data_l);
#endif /* (RFAT_CONFIG_DISK_CRC == 1) */
	
	data16 = (data_h << 8) | data_l;

	while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
        
	SSI2->DR;
	SSI2->DR = data16;
    }
    
    while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
    
    SSI2->DR;
    SSI2->DR = crc16;
    
    for (n = 0; n < TM4C123_DISK_SSI_FIFO_COUNT; n++)
    {
	while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
        
	SSI2->DR;
    }

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->CR1 = 0;
    SSI2->CR0 = (tm4c123_disk_device.ssi_cr0 & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_8;
    SSI2->CR1 = SSI_CR1_SSE;
#endif /* TM4C123_DISK_CONFIG_DMA == 1 */
}


/*
 * tm4c123_disk_receive_block(const uint8_t *data)
 *
 * Returns 0 on success, and non-zero on a CRC error.
 */

__attribute__((optimize("-O3"))) uint32_t tm4c123_disk_receive_block(uint8_t *data)
{
#if (TM4C123_DISK_CONFIG_DMA == 1)
    uint32_t crc16;

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    tm4c123_disk_device.ssi_scratch[0] = 0xff;

    /* SSI2 RX */
    tm4c123_udma_control_table[12][0] = (uint32_t)&SSI2->DR;
    tm4c123_udma_control_table[12][1] = (uint32_t)&data[511];
    tm4c123_udma_control_table[12][2] = (UDMA_CHCTL_DSTINC_8 | UDMA_CHCTL_DSTSIZE_8 |
					 UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 |
					 UDMA_CHCTL_ARBSIZE_4 | UDMA_CHCTL_XFERMODE_BASIC |
					 (511 << UDMA_CHCTL_XFERSIZE_S));

    /* SSI2 TX */
    tm4c123_udma_control_table[13][0] = (uint32_t)&tm4c123_disk_device.ssi_scratch[0];
    tm4c123_udma_control_table[13][1] = (uint32_t)&SSI2->DR;
    tm4c123_udma_control_table[13][2] = (UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8 |
					 UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 |
					 UDMA_CHCTL_ARBSIZE_4 | UDMA_CHCTL_XFERMODE_BASIC |
					 (511 << UDMA_CHCTL_XFERSIZE_S));

    __DMB();

    UDMA->ENASET = ((1ul << 12) | (1ul << 13));

    SSI2->DMACTL = SSI_DMACTL_TXDMAE | SSI_DMACTL_RXDMAE;

    ARMV7M_PROFILE_TAG_PUSH(IDLE);

    while ((UDMA->CHIS & ((1ul << 12) | (1ul << 13))) != ((1ul << 12) | (1ul << 13)))
    {
	continue;
    }

    UDMA->CHIS = ((1ul << 12) | (1ul << 13));

    ARMV7M_PROFILE_TAG_POP();

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->DMACTL = 0;

    crc16 = tm4c123_disk_receive() << 8;
    crc16 |= tm4c123_disk_receive() & 0xff;

#if (RFAT_CONFIG_DISK_CRC == 1)
    crc16 ^= rfat_compute_crc16(data, 512);
#else /* (RFAT_CONFIG_DISK_CRC == 1) */
    crc16 = 0;
#endif /* (RFAT_CONFIG_DISK_CRC == 1) */

#else /* TM4C123_DISK_CONFIG_DMA == 1 */
    unsigned int n;
    uint8_t data_l, data_h;
    uint32_t data16, crc16;

    crc16 = 0;

    /*
     * Idea is to stuff first up data into the TX FIFO till it's full
     * (or better said till there ae no more splots in the RX FIFO).
     * Then wait for at least one item in the RX FIFO to read it back,
     * and refill the TX FIFO. At the end, the RX FIFO is drained.
     */

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->CR1 = 0;
    SSI2->CR0 = (tm4c123_disk_device.ssi_cr0 & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_16;
    SSI2->CR1 = SSI_CR1_SSE;
    
    for (n = 0; n < TM4C123_DISK_SSI_FIFO_COUNT; n++)
    {
	SSI2->DR = 0xffff;
    }
    
    for (n = 0; n < (((512 + 2) / 2) - TM4C123_DISK_SSI_FIFO_COUNT); n++)
    {
	while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
	
	data16 = SSI2->DR;
	SSI2->DR = 0xffff;

	data_h = data16 >> 8;
	data_l = data16;

#if (RFAT_CONFIG_DISK_CRC == 1)
	RFAT_UPDATE_CRC16(crc16, data_h);
	RFAT_UPDATE_CRC16(crc16, data_l);
#endif /* (RFAT_CONFIG_DISK_CRC == 1) */

	*data++ = data_h;
	*data++ = data_l;
    }
    
    for (; n < (RFAT_BLK_SIZE / 2); n++)
    {
	while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
	
	data16 = SSI2->DR;

	data_h = data16 >> 8;
	data_l = data16;

#if (RFAT_CONFIG_DISK_CRC == 1)
	RFAT_UPDATE_CRC16(crc16, data_h);
	RFAT_UPDATE_CRC16(crc16, data_l);
#endif /* (RFAT_CONFIG_DISK_CRC == 1) */

	*data++ = data_h;
	*data++ = data_l;
    }
    
    while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
    
#if (RFAT_CONFIG_DISK_CRC == 1)
    crc16 ^= (SSI2->DR & 0xffff);
#else /* (RFAT_CONFIG_DISK_CRC == 1) */
    SSI2->DR;
#endif /* (RFAT_CONFIG_DISK_CRC == 1) */

    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->CR1 = 0;
    SSI2->CR0 = (tm4c123_disk_device.ssi_cr0 & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_8;
    SSI2->CR1 = SSI_CR1_SSE;
#endif /* TM4C123_DISK_CONFIG_DMA == 1 */

    return crc16;
}

void tm4c123_disk_initialize(void)
{
}
