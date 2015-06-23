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

typedef struct _record_device_t {
    uint8_t           state;
    volatile uint8_t  active;
    volatile uint8_t  start;
    volatile uint8_t  stop;
    F_FILE            *file;
    uint32_t          sequence;
    char              filename[16];
    volatile uint32_t read;
    volatile uint32_t write;
    uint32_t          data[RECORD_DATA_SIZE / 4];
} record_device_t;

static record_device_t record_device;

static void record_open(void)
{
    uint32_t sequence;
    char *cp;
    F_FILE *file;
    
    if (record_device.state != RECORD_STATE_OPEN)
    {
	if (record_device.state != RECORD_STATE_FAULT)
	{
	    sequence = record_device.sequence;
	    
	    cp = record_device.filename;
	    
	    *cp++ = 'L';
	    *cp++ = 'O';
	    *cp++ = 'G';
	    *cp++ = '0' + (sequence / 10000); sequence -= ((sequence / 10000) * 10000);
	    *cp++ = '0' + (sequence / 1000);  sequence -= ((sequence / 1000)  * 1000);
	    *cp++ = '0' + (sequence / 100);   sequence -= ((sequence / 100)   * 100);
	    *cp++ = '0' + (sequence / 10);    sequence -= ((sequence / 10)    * 10);
	    *cp++ = '0' + (sequence / 1);     sequence -= ((sequence / 1)     * 1);
	    *cp++ = '.';
	    *cp++ = 'D';
	    *cp++ = 'A';
	    *cp++ = 'T';
	    *cp++ = '\0';
    
	    file = f_open(record_device.filename, "w,32M");
    
	    if (file)
	    {
		record_device.state = RECORD_STATE_OPEN;
		record_device.file = file;
	    }
	    else
	    {
		record_device.state = RECORD_STATE_FAULT;
	    }
	}
    }

    record_device.start = 0;
}

static void record_close(void)
{
    uint32_t size, read, write;

    if (record_device.state == RECORD_STATE_OPEN)
    {
	ARMV7M_PROFILE_TAG_PUSH(RECORD);

	record_device.state = RECORD_STATE_CLOSED;

	read = record_device.read;
	write = record_device.write;

	if (read > write)
	{
	    size = RECORD_DATA_SIZE - read;

	    f_write((uint8_t*)&record_device.data[0] + read, size, 1, record_device.file);

	    read = 0;
	}

	if (write > read)
	{
	    size = write - read;

	    f_write((uint8_t*)&record_device.data[0] + read, size, 1, record_device.file);
	}

	f_close(record_device.file);

	record_device.sequence++;
	record_device.file = NULL;
	record_device.read = 0;
	record_device.write = 0;

	tm4c123_led(TM4C123_LED_GREEN);

	ARMV7M_PROFILE_TAG_POP();
    }

    record_device.stop = 0;
}

void record_start(void)
{
    if (!record_device.active)
    {
	record_device.active = 1;
	record_device.start = 1;
    }
}

void record_stop(void)
{
    if (record_device.active)
    {
	record_device.active = 0;
	record_device.stop = 1;
    }
}

void record_enter(const void *data, unsigned int size)
{
    uint32_t avail, total, write, write_next, read;

    if (record_device.active)
    {
	ARMV7M_PROFILE_TAG_PUSH(RECORD);

	total = ((size + 3) & ~3);

	do 
	{
	    read  = record_device.read;
	    write = record_device.write;

	    if (read == write)
	    {
		avail = RECORD_DATA_SIZE - 4;
	    }
	    else
	    {
		if (read > write)
		{
		    avail = (read - write) - 4;
		}
		else
		{
		    avail = RECORD_DATA_SIZE - (write - read) - 4;
		}
	    }
	
	    if (avail < total)
	    {
		break;
	    }
	
	    write_next = write + total;
	
	    if (write_next >= RECORD_DATA_SIZE)
	    {
		write_next -= RECORD_DATA_SIZE;
	    }
	}
	while (!armv7m_atomic_compare_and_exchange(&record_device.write, write, write_next));

	if (avail >= total)
	{
	    if ((write + size) >= RECORD_DATA_SIZE)
	    {
		memcpy((uint8_t*)&record_device.data[0] + write, data, (RECORD_DATA_SIZE - write));

		data = (const void*)((const uint8_t*)data + (RECORD_DATA_SIZE - write));
		size -= (RECORD_DATA_SIZE - write);
		write = 0;
	    }

	    if (size)
	    {
		memcpy((uint8_t*)&record_device.data[0] + write, data, size);
	    }
	}

	ARMV7M_PROFILE_TAG_POP();
    }
}

void record_enter_extended(uint8_t type, uint8_t flags, uint64_t tick, const void *data, unsigned int size)
{
    uint16_t utime;
    uint32_t avail, total, write, write_next, read, ltime, data_l, data_h;

    if (record_device.active)
    {
	ARMV7M_PROFILE_TAG_PUSH(RECORD);

	total = 8 + ((size + 3) & ~3);

	do 
	{
	    read  = record_device.read;
	    write = record_device.write;

	    if (read == write)
	    {
		avail = RECORD_DATA_SIZE - 4;
	    }
	    else
	    {
		if (read > write)
		{
		    avail = (read - write) - 4;
		}
		else
		{
		    avail = RECORD_DATA_SIZE - (write - read) - 4;
		}
	    }
	
	    if (avail < total)
	    {
		break;
	    }
	
	    write_next = write + total;
	
	    if (write_next >= RECORD_DATA_SIZE)
	    {
		write_next -= RECORD_DATA_SIZE;
	    }
	}
	while (!armv7m_atomic_compare_and_exchange(&record_device.write, write, write_next));
	
	if (avail >= total)
	{
	    utime = tick >> 32;
	    ltime = tick & 0xffffffff;

	    data_l = (type << 0) | (flags << 8) | (utime << 16);
	    data_h = ltime;

	    record_device.data[write / 4] = data_l;
	    write += 4;

	    if (write >= RECORD_DATA_SIZE)
	    {
		write = 0;
	    }

	    record_device.data[write / 4] = data_h;
	    write += 4;

	    if (write >= RECORD_DATA_SIZE)
	    {
		write = 0;
	    }

	    if ((write + size) >= RECORD_DATA_SIZE)
	    {
		memcpy((uint8_t*)&record_device.data[0] + write, data, (RECORD_DATA_SIZE - write));

		data = (const void*)((const uint8_t*)data + (RECORD_DATA_SIZE - write));
		size -= (RECORD_DATA_SIZE - write);
		write = 0;
	    }

	    if (size)
	    {
		memcpy((uint8_t*)&record_device.data[0] + write, data, size);
	    }
	}

	ARMV7M_PROFILE_TAG_POP();
    }
}

uint32_t record_sequence(void)
{
    return (record_device.sequence | ((record_device.file && (f_error(record_device.file) == F_NO_ERROR)) ? RECORD_SEQUENCE_ACTIVE : 0));
}

int record_flush(void)
{
    int busy = 0;
    uint32_t size, read, read_next, write;

    if (record_device.start)
    {
	record_open();
    }

    if (record_device.stop)
    {
	record_close();
    }

    if (record_device.active)
    {
	ARMV7M_PROFILE_TAG_PUSH(RECORD);

	read = record_device.read;
	write = record_device.write;

	if (read > write)
	{
	    size = RECORD_DATA_SIZE - read;
	}
	else
	{
	    size = write - read;
	}
	
	if (size >= 512)
	{
	    size = 512;
	    
	    f_write((uint8_t*)&record_device.data[0] + read, 512, 1, record_device.file);
	    
	    read_next = read + 512;
	    
	    if (read_next >= RECORD_DATA_SIZE)
	    {
		read_next = 0;
	    }
	    
	    record_device.read = read_next;

	    busy = 1;
	}

	ARMV7M_PROFILE_TAG_POP();
    }

    return busy;
}

void record_initialize(void)
{
    uint32_t sequence;
    F_FIND find;

    f_initvolume();

    if (f_checkvolume() != F_NO_ERROR)
    {
	record_device.state = RECORD_STATE_FAULT;

	control_sdc_ini_callback(0);
    }
    else
    {
	sequence = 0;
	    
	if (f_findfirst("LOG?????.DAT", &find) == F_NO_ERROR)
	{
	    do
	    {
		sequence = (((find.name[3] - '0') * 10000) +
			    ((find.name[4] - '0') * 1000) +
			    ((find.name[5] - '0') * 100) +
			    ((find.name[6] - '0') * 10) +
			    ((find.name[7] - '0') * 1));
		
		if (record_device.sequence < sequence)
		{
		    record_device.sequence = sequence;
		}
	    }
	    while (f_findnext(&find) == F_NO_ERROR);
	}

	record_device.sequence++;

	control_sdc_ini_callback(1);
    }
}

