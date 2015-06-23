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

void *fifo_allocate(fifo_t *fifo)
{
    unsigned int sindex;
    void *data;

    sindex = fifo->sindex;

    if ((sindex ^ fifo->rindex) == FIFO_INDEX_WRAP)
    {
	fifo->level = FIFO_INDEX_WRAP;

	data = NULL;
    }
    else
    {
	data = (void*)((uint8_t*)fifo->data + ((sindex & FIFO_INDEX_MASK) * fifo->size));
    }

    return data;
}

void fifo_send(fifo_t *fifo)
{
    unsigned int sindex, rindex, level;

    sindex = (fifo->sindex & FIFO_INDEX_MASK) + 1;

    if (sindex == fifo->count)
    {
	sindex = FIFO_INDEX_WRAP;
    }

    fifo->sindex = (fifo->sindex & FIFO_INDEX_WRAP) ^ sindex;

    /* This code below is really just for debugging, so that
     * we can see the maximum FIFO fullness.
     */

    sindex = fifo->sindex;
    rindex = fifo->rindex;

    if ((sindex ^ rindex) == FIFO_INDEX_WRAP)
    {
	level = fifo->count;
    }
    else
    {
	sindex &= FIFO_INDEX_MASK;
	rindex &= FIFO_INDEX_MASK;

	if (sindex >= rindex)
	{
	    level = sindex - rindex;
	}
	else
	{
	    level = fifo->count - (rindex - sindex);
	}
    }

    if (fifo->level < level)
    {
	fifo->level = level;
    }
}

void *fifo_receive(fifo_t *fifo)
{
    unsigned int rindex;
    void *data = NULL;

    rindex = fifo->rindex;

    if ((rindex ^ fifo->sindex) != 0)
    {
	data = (void*)((uint8_t*)fifo->data + ((rindex & FIFO_INDEX_MASK) * fifo->size));
    }

    return data;
}

void fifo_release(fifo_t *fifo)
{
    unsigned int rindex;

    rindex = (fifo->rindex & FIFO_INDEX_MASK) + 1;

    if (rindex == fifo->count)
    {
	rindex = FIFO_INDEX_WRAP;
    }

    fifo->rindex = (fifo->rindex & FIFO_INDEX_WRAP) ^ rindex;
}
