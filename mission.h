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

#if !defined(_MISSION_H)
#define _MISSION_H

#define WAYPOINT_SPEED_STOP        0
#define WAYPOINT_SPEED_SLOW        1
#define WAYPOINT_SPEED_MEDIUM      2
#define WAYPOINT_SPEED_FAST        3
#define WAYPOINT_SPEED_ULTRA       4
#define WAYPOINT_SPEED_BRAKE       5
#define WAYPOINT_SPEED_COAST       6
#define WAYPOINT_SPEED_RESERVED_7  7
#define WAYPOINT_SPEED_NUM         8
#define WAYPOINT_SPEED_MASK        7

typedef struct _waypoint_t {
    int32_t  latitude;
    int32_t  longitude;
    uint32_t mode;
} waypoint_t;

typedef struct _mission_t {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    float   variation;      /* magnetic variation */
    uint32_t wpt_count;
    const waypoint_t *wpt_table;
    float   speed_table[WAYPOINT_SPEED_NUM];
} mission_t;

extern const mission_t mission;

#endif /* _MISSION_H */
