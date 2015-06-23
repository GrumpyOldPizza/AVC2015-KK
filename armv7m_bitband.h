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

#if !defined(_ARMV7M_BITBAND_H)
#define _ARMV7M_BITBAND_H

static void inline armv7m_bitband_sram_write(volatile uint32_t *p_data, uint32_t index, uint32_t data)
{
    *((volatile uint32_t*)(0x22000000 + (((uint32_t)p_data - 0x20000000) * 32) + (index * 4))) = data;
}

static uint32_t inline armv7m_bitband_sram_read(volatile uint32_t *p_data, uint32_t index)
{
    return *((volatile uint32_t*)(0x22000000 + (((uint32_t)p_data - 0x20000000) * 32) + (index * 4)));
}

static void inline armv7m_bitband_peripheral_write(volatile uint32_t *p_data, uint32_t index, uint32_t data)
{
    *((volatile uint32_t*)(0x42000000 + (((uint32_t)p_data - 0x40000000) * 32) + (index * 4))) = data;
}

static uint32_t inline armv7m_bitband_peripheral_read(volatile uint32_t *p_data, uint32_t index)
{
    return *((volatile uint32_t*)(0x42000000 + (((uint32_t)p_data - 0x40000000) * 32) + (index * 4)));
}

#endif /* _ARMV7M_BITBAND_H */
