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

#if !defined(SIMULATION)
#include "kitty.h"
#endif

typedef struct _guidance_device_t {
    guidance_target_t wpt_target[2];
    uint32_t          wpt_index;
    uint32_t          wpt_reset;
    lla_home_t        home;
} guidance_device_t;

guidance_device_t guidance_device;

void guidance_target(float x, float y, float *speed, float *course)
{
    float x_previous, y_previous, x_next, y_next, x_intercept, y_intercept, dx, dy, w, d, k, l;

    if (guidance_device.wpt_reset == 1)
    {
#if !defined(SIMULATION)
	record_enter_extended(RECORD_TYPE_GUIDANCE, 0, tm4c123_capture_clock(), &guidance_device.wpt_target[1], sizeof(guidance_target_t));
#endif

	guidance_device.wpt_reset = 0;
    }

    if (guidance_device.wpt_index == 0)
    {
	x_previous = x;
	y_previous = y;
    }
    else
    {
	x_previous = guidance_device.wpt_target[0].position[0];
	y_previous = guidance_device.wpt_target[0].position[1];
    }

    x_next = guidance_device.wpt_target[1].position[0];
    y_next = guidance_device.wpt_target[1].position[1];

    d = sqrtf((x_next - x) * (x_next - x) + (y_next - y) * (y_next - y));

    if (d < GUIDANCE_WAYPOINT_THRESHOLD)
    {
	guidance_device.wpt_target[0] = guidance_device.wpt_target[1];
	guidance_device.wpt_index++;
	
	if ((guidance_device.wpt_index == mission.wpt_count) ||
	    (mission.wpt_table[guidance_device.wpt_index].mode & WAYPOINT_SPEED_MASK) == WAYPOINT_SPEED_STOP)
	{
	    /* Always fake the last waypoint and stay there.
	     */
	    guidance_device.wpt_target[1].position[0] = x;
	    guidance_device.wpt_target[1].position[1] = y;
	    guidance_device.wpt_target[1].position[2] = 0.0;
	    guidance_device.wpt_target[1].speed = 0.0;
	    
	    guidance_device.wpt_index--;
	}
	else
	{
	    LLA2NED(mission.wpt_table[guidance_device.wpt_index].latitude,
		    mission.wpt_table[guidance_device.wpt_index].longitude,
		    mission.altitude,
		    &guidance_device.home,
		    &guidance_device.wpt_target[1].position[0]);
	    
	    guidance_device.wpt_target[1].speed = mission.speed_table[mission.wpt_table[guidance_device.wpt_index].mode & WAYPOINT_SPEED_MASK];
	}

#if !defined(SIMULATION)
	record_enter_extended(RECORD_TYPE_GUIDANCE, 0, tm4c123_capture_clock(), &guidance_device.wpt_target[1], sizeof(guidance_target_t));
#endif

	x_previous = guidance_device.wpt_target[0].position[0];
	y_previous = guidance_device.wpt_target[0].position[1];
	x_next     = guidance_device.wpt_target[1].position[0];
	y_next     = guidance_device.wpt_target[1].position[1];
    }

    if (guidance_device.wpt_target[1].speed > 0.0)
    {
	/* Compute the distance from x,y to the line x_previous,y_previous ... x_next,y_next.
	 * N.b. x & y are swapped !
	 */
	
	dy = y_next - y_previous;
	dx = x_next - x_previous;
	
	w = sqrtf(dy * dy + dx * dx);
	d = fabsf(dy * (x_previous - x) - dx *(y_previous - y)) / w;
	
	/* Compute the distance x_previous,y_previous to x,y ...
	 */
	
	k = sqrtf((x - x_previous) * (x - x_previous) + (y - y_previous) * (y - y_previous));
	
	/* The distance to x_intercept,y_intercept from x_previous,y_previous is sqrtf(k*k - d*d) + lookahead.
	 */
	l = sqrtf(k*k - d*d) + GUIDANCE_INTERCEPT_LOOKAEAHD;
	
	x_intercept = x_previous + dx * (l / w);
	y_intercept = y_previous + dy * (l / w);
	
	/* Compute the required course.
	 */

	dx = x_intercept - x;
	dy = y_intercept - y;

	*speed  = guidance_device.wpt_target[1].speed;
	*course = angle_normalize(atan2f(dy, dx));
    }
    else
    {
	*speed  = 0.0;
	*course = 0.0;
    }
}

void guidance_initialize(void)
{
    LLA2HOME(mission.latitude, mission.longitude, mission.altitude, &guidance_device.home);

    LLA2NED(mission.wpt_table[0].latitude,
	    mission.wpt_table[0].longitude,
	    mission.altitude,
	    &guidance_device.home,
	    &guidance_device.wpt_target[1].position[0]);

    guidance_device.wpt_target[1].speed = mission.speed_table[mission.wpt_table[0].mode & WAYPOINT_SPEED_MASK];
    guidance_device.wpt_index = 0;
    guidance_device.wpt_reset = 1;
}
