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

#if !defined(_ARMV7M_SAFE_H)
#define _ARMV7M_SAFE_H

#define ARMV7M_SAFE_DECLARE(_type, _name)                \
    extern voltile _type _name;                          \
    extern voltile _type _name##_shadow

#define ARMV7M_SAFE_CREATE(_type, _name)                 \
    volatile _type _name;                                \
    volatile _type _name##_shadow = ((_type)(-1));

#define ARMV7M_SAFE_WRITE(_type, _name, _data) {	 \
    _type __data = (_data);     		         \
    _type __data_shadow = (__data ^ (((_type)-1)));	 \
    _name = __data;	                                 \
    _name##_shadow = __data_shadow;	                 \
}

#define ARMV7M_SAFE_READ(_type, _name, _data) {		 \
    _type __data, __data_shadow, __data_check;		 \
    do {                                                 \
       __data = _name;	                                 \
       __data_shadow = _name##_shadow;	                 \
       __data_check = _name;	                         \
    } while (__data != __data_check);		  	 \
    if ((__data ^ __data_shadow) != ((_type)(-1))) {	 \
        armv7m_safe_corruption();	                 \
    }                                                    \
    (_data) = __data;					 \
}

extern void armv7m_safe_corruption(void);

#endif /* _ARMV7M_SAFE_H */
