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

typedef struct _tm4c123_tft_device_t {
    uint32_t     ssi_cr0;
    uint32_t     ssi_cpsr;
    uint32_t     ssi_fifo_count;
} tm4c123_tft_device_t;

static tm4c123_tft_device_t tm4c123_tft_device;

void tm4c123_tft_select(void)
{
    SSI2->CR0  = tm4c123_tft_device.ssi_cr0;
    SSI2->CPSR = tm4c123_tft_device.ssi_cpsr;
    SSI2->CR1  = SSI_CR1_SSE;

    /* CS output, drive CS to L */
#if (TM4C123_SPI_CONFIG_LEGACY == 0)
    armv7m_bitband_peripheral_write(&GPIOA->DATA, 4, 0);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    armv7m_bitband_peripheral_write(&GPIOB->DATA, 5, 0);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */
}

void tm4c123_tft_deselect(void)
{
    while (SSI2->SR & SSI_SR_BSY) { continue; }

    /* CS is output, drive CS to H */
#if (TM4C123_SPI_CONFIG_LEGACY == 0)
    armv7m_bitband_peripheral_write(&GPIOA->DATA, 4, 1);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    armv7m_bitband_peripheral_write(&GPIOB->DATA, 5, 1);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */

    /* Disable SPI port for shared access
     */
    SSI2->CR1 = 0;
}

void tm4c123_tft_command(void)
{
    while (SSI2->SR & SSI_SR_BSY) { continue; }

    /* D/C output, drive D/C to L */
#if (TM4C123_SPI_CONFIG_LEGACY == 0)
    armv7m_bitband_peripheral_write(&GPIOE->DATA, 4, 0);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    armv7m_bitband_peripheral_write(&GPIOE->DATA, 5, 0);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */
}

void tm4c123_tft_data(void)
{
    while (SSI2->SR & SSI_SR_BSY) { continue; }

    /* D/C output, drive D/C to H */
#if (TM4C123_SPI_CONFIG_LEGACY == 0)
    armv7m_bitband_peripheral_write(&GPIOE->DATA, 4, 1);
#else /* TM4C123_SPI_CONFIG_LEGACY == 0 */
    armv7m_bitband_peripheral_write(&GPIOE->DATA, 5, 1);
#endif /* TM4C123_SPI_CONFIG_LEGACY == 0 */
}

void tm4c123_tft_send(uint8_t data)
{
    SSI2->DR = data;

    while (!(SSI2->SR & SSI_SR_RNE)) { continue; }

    SSI2->DR;
}

uint8_t tm4c123_tft_receive(void)
{
    SSI2->DR = 0xff;

    while (!(SSI2->SR & SSI_SR_RNE)) { continue; }

    return SSI2->DR;
}

void tm4c123_tft_setup_data16(void)
{
    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->CR1 = 0;
    SSI2->CR0 = (SSI2->CR0 & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_16;
    SSI2->CR1 = SSI_CR1_SSE;

    tm4c123_tft_device.ssi_fifo_count = 0;
}

void tm4c123_tft_finish_data16(void)
{
    while (tm4c123_tft_device.ssi_fifo_count)
    {
	while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
	    
	SSI2->DR;
	
	tm4c123_tft_device.ssi_fifo_count--;
    }
    
    while (SSI2->SR & SSI_SR_BSY) { continue; }

    SSI2->CR1 = 0;
    SSI2->CR0 = (SSI2->CR0 & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_8;
    SSI2->CR1 = SSI_CR1_SSE;
}

void tm4c123_tft_send_data16(uint16_t data)
{
    if (tm4c123_tft_device.ssi_fifo_count < TM4C123_TFT_SSI_FIFO_SIZE)
    {
	SSI2->DR = data;

	tm4c123_tft_device.ssi_fifo_count++;
    }
    else
    {
	while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
	    
	SSI2->DR;
	SSI2->DR = data;
    }
}

__attribute__((optimize("-O3"))) void tm4c123_tft_send_data16_multiple(uint16_t data, uint32_t count)
{
    unsigned int n;

    if (count <= TM4C123_TFT_SSI_FIFO_SIZE)
    {
	while (count--)
	{
	    tm4c123_tft_send_data16(data);
	}
    }
    else
    {
	for (n = tm4c123_tft_device.ssi_fifo_count; n < TM4C123_TFT_SSI_FIFO_SIZE; n++)
	{
	    SSI2->DR = data;
	}
	
	for (n = TM4C123_TFT_SSI_FIFO_SIZE - tm4c123_tft_device.ssi_fifo_count; n < count; n++)
	{
	    while (!(SSI2->SR & SSI_SR_RNE)) { continue; }
	    
	    SSI2->DR;
	    SSI2->DR = data;
	}

	tm4c123_tft_device.ssi_fifo_count = TM4C123_TFT_SSI_FIFO_SIZE;
    }
}

void tm4c123_tft_initialize(void)
{
    uint32_t cpsdvsr, scr;

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
    
    cpsdvsr = ((SystemCoreClock / TM4C123_TFT_SSI_SPEED) / 256 +1) & ~1;
      
    if (cpsdvsr == 0)
    {
	cpsdvsr = 2;
    }
    
    scr = (SystemCoreClock / TM4C123_TFT_SSI_SPEED + (cpsdvsr -1)) / cpsdvsr -1;
    
    tm4c123_tft_device.ssi_cr0  = ((scr << 8) | SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8);
    tm4c123_tft_device.ssi_cpsr = cpsdvsr;
    
    SSI2->CR1  = 0;
    SSI2->CR0  = tm4c123_tft_device.ssi_cr0;
    SSI2->CPSR = tm4c123_tft_device.ssi_cpsr;
    SSI2->IM   = 0u;
    SSI2->ICR  = ~0u;	
    SSI2->CR1  = SSI_CR1_SSE;
}
