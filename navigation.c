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

#if defined(SIMULATION)
#define ARMV7M_PROFILE_TAG_PUSH(_tag) /**/
#define ARMV7M_PROFILE_TAG_POP(_tag)  /**/
#endif

#define NAVIGATION_CONFIG_DIFFERENTIAL 0

typedef struct _navigation_table_entry_t {
    uint64_t         tick;
    uint32_t         sequence;
    uint16_t         rpm_count;
    uint16_t         gyro_count;
    float            rpm_accum;
    float            gyro_accum;
} navigation_table_entry_t;

#define NAVIGATION_TABLE_SIZE 64
#define NAVIGATION_TABLE_MASK (NAVIGATION_TABLE_SIZE-1) 

#define NAVIGATION_TABLE_PREDICT_RATE    10                /* 10 gyro samples == PREDICT, 10 PREDICT == 1 CORRECT ... */
#define NAVIGATION_TABLE_GYRO_COUNT_DONE 0xffff

typedef struct _navigation_device_t {
    uint64_t         pps_reference[2];
    uint64_t         pps_period[2];
    uint32_t         pps_sequence[2];
    uint32_t         rpm_sequence;
    uint32_t         gyro_sequence;
    uint32_t         gps_sequence;
    uint32_t         gps_reference;    /* last GPS sequence @ ekf_correct/ekf_initialize */
#if (NAVIGATION_CONFIG_DIFFERENTIAL == 1)
    uint64_t         gps_tick;
    float            gps_position[3];
    float            gps_speed;
    float            gps_course;
#endif /* CONFIG_DIFFERENTIAL == 1 */
    uint32_t         sequence;
    navigation_location_t * volatile location;
    navigation_location_t location_data[2];
    navigation_table_entry_t table[NAVIGATION_TABLE_SIZE];
    lla_home_t       home;
} navigation_device_t;

navigation_device_t navigation_device;

volatile navigation_sensors_t navigation_sensors;

#if !defined(SIMULATION)

FIFO_CREATE(sizeof(gps_location_entry_t), 4, gps_fifo);

static void gps_pendsv_callback(void)
{
    uint64_t tick;
    gps_location_entry_t *entry;

    while ((entry = (gps_location_entry_t*)fifo_receive(&gps_fifo)))
    {
	record_enter(entry, sizeof(gps_location_entry_t));

	tick = ((uint64_t)entry->utime << 32) | (uint64_t)entry->ltime;

	navigation_gps_notify(tick, &entry->location);

	fifo_release(&gps_fifo);
    }
}

void gps_location_callback(uint64_t tick, const gps_location_t *location)
{
    gps_location_entry_t *entry;

    entry = (gps_location_entry_t*)fifo_allocate(&gps_fifo);

    if (entry)
    {
	entry->type = RECORD_TYPE_GPS_LOCATION;
	entry->flags = 1;
	entry->utime = tick >> 32;
	entry->ltime = tick & 0xffffffff;
	entry->location = *location;
	
	fifo_send(&gps_fifo);
	
	armv7m_pendsv_pending(PENDSV_SLOT_GPS);
    }
}

void gps_satellites_callback(uint64_t tick, const gps_satellites_t *satellites)
{
    record_enter_extended(RECORD_TYPE_GPS_SATELLITES, sizeof(satellites->info) / sizeof(satellites->info[0]), tick, satellites, sizeof(gps_satellites_t));
}

#endif /* SIMULATION */

static inline uint32_t navigation_sequence(uint64_t tick, uint32_t offset)
{
    uint32_t sequence;

    if (tick >= navigation_device.pps_reference[1])
    {
	sequence = (((tick - navigation_device.pps_reference[1]) * 100 + offset) / navigation_device.pps_period[1]) + navigation_device.pps_sequence[1];
    }
    else
    {
	sequence = (((tick - navigation_device.pps_reference[0]) * 100 + offset) / navigation_device.pps_period[0]) + navigation_device.pps_sequence[0];
    }

    return sequence;
}

static inline uint64_t navigation_threshold(uint64_t tick, uint32_t sequence)
{
    uint64_t threshold;

    if (tick >= navigation_device.pps_reference[1])
    {
	threshold = ((((sequence +1) - navigation_device.pps_sequence[1]) * navigation_device.pps_period[1]) / 100) + navigation_device.pps_reference[1];
    }
    else
    {
	threshold = ((((sequence +1) - navigation_device.pps_sequence[0]) * navigation_device.pps_period[0]) / 100) + navigation_device.pps_reference[0];
    }
    
    return threshold;
}

static inline void navigation_resolve(const navigation_table_entry_t *entry, float *rpm_speed, float *gyro_rate)
{
    if (entry->rpm_accum > 0.0)
    {
	if (entry->rpm_count >= 2)
	{
	    *rpm_speed = ((double)navigation_device.pps_period[1] / ((double)entry->rpm_accum / (double)entry->rpm_count));
	}
	else
	{
	    *rpm_speed = ((double)navigation_device.pps_period[1] / (double)entry->rpm_accum);
	}
	
	if (*rpm_speed < 0.001)
	{
	    *rpm_speed = 0.0;
	}
    }
    else
    {
	*rpm_speed = 0.0;
    }
    
    *gyro_rate = (entry->gyro_accum / entry->gyro_count);
}

static void navigation_predict(const navigation_table_entry_t *entry)
{
    float rpm_speed, gyro_rate, dT, S0, S1;
    navigation_location_t *out, *in;

    navigation_resolve(entry, &rpm_speed, &gyro_rate);

    in = navigation_device.location;

    if (in == &navigation_device.location_data[0])
    {
	out = &navigation_device.location_data[1];
    }
    else
    {
	out = &navigation_device.location_data[0];
    }

    dT = 0.01;
    
    S0 = in->rpm_scale * rpm_speed * dT;
    S1 = in->course + 0.5 * in->gyro_scale * (gyro_rate - in->gyro_bias) * dT;

    out->tick       = entry->tick;
    out->x          = in->x + S0 * cosf(S1);
    out->y          = in->y + S0 * sinf(S1);
    out->speed      = in->rpm_scale * rpm_speed;
    out->course     = angle_normalize(in->course + in->gyro_scale * (gyro_rate - in->gyro_bias) * dT);
    out->rpm_scale  = in->rpm_scale;
    out->gyro_scale = in->gyro_scale;
    out->gyro_bias  = in->gyro_bias;

    navigation_device.location = out;
}

static void navigation_correct(uint64_t tick, uint32_t sequence, float x, float y, float speed, float course, float rpm_scale, float gyro_scale, float gyro_bias)
{
    navigation_location_t *out, *in;

    in = navigation_device.location;

    if (in == &navigation_device.location_data[0])
    {
	out = &navigation_device.location_data[1];
    }
    else
    {
	out = &navigation_device.location_data[0];
    }

    out->tick       = tick;
    out->x          = x;
    out->y          = y;
    out->speed      = speed;
    out->course     = course;
    out->rpm_scale  = rpm_scale;
    out->gyro_scale = gyro_scale;
    out->gyro_bias  = gyro_bias;

    navigation_device.sequence = sequence;
    navigation_device.location = out;
}

void navigation_pps_notify(uint64_t tick, uint32_t period)
{
    ARMV7M_PROFILE_TAG_PUSH(NAVIGATION);

    navigation_device.pps_reference[0] = navigation_device.pps_reference[1];
    navigation_device.pps_period[0]    = navigation_device.pps_period[1];
    navigation_device.pps_sequence[0]  = navigation_device.pps_sequence[1];
    
    navigation_device.pps_reference[1] = tick;
    navigation_device.pps_period[1]    = period;
    navigation_device.pps_sequence[1]  = navigation_device.pps_sequence[0] + (((tick - navigation_device.pps_reference[0]) * 100) / navigation_device.pps_period[0]);

    ARMV7M_PROFILE_TAG_POP();
}

void navigation_rpm_notify(uint64_t tick)
{
    navigation_table_entry_t *entry;

    ARMV7M_PROFILE_TAG_PUSH(NAVIGATION);

    navigation_sensors.rpm_delta = ((double)(tick - navigation_sensors.rpm_tick) + 2.0 * navigation_sensors.rpm_delta) / 3.0;
    navigation_sensors.rpm_tick = tick;

    navigation_device.rpm_sequence = navigation_sequence(tick, 0);

    entry = &navigation_device.table[navigation_device.rpm_sequence & NAVIGATION_TABLE_MASK];

    if (entry->sequence != navigation_device.rpm_sequence)
    {
	entry->sequence = navigation_device.rpm_sequence;
	entry->rpm_count = 0;
	entry->gyro_count = 0;
    }
    
    if (entry->rpm_count == 0)
    {
	entry->rpm_accum = navigation_sensors.rpm_delta;
    }
    else
    {
	entry->rpm_accum += navigation_sensors.rpm_delta;
    }
    
    entry->rpm_count++;

    ARMV7M_PROFILE_TAG_POP();
}

void navigation_accel_notify(uint64_t tick, float ax, float ay, float az)
{
    ARMV7M_PROFILE_TAG_PUSH(NAVIGATION);

    navigation_sensors.accel_tick = tick;
    navigation_sensors.accel_data[0] = ax;
    navigation_sensors.accel_data[1] = ay;
    navigation_sensors.accel_data[2] = az;

    ARMV7M_PROFILE_TAG_POP();
}

void navigation_gyro_notify(uint64_t tick, float gx, float gy, float gz)
{
    navigation_table_entry_t *entry;
    float gyro_delta, gyro_rate, rpm_speed;

    ARMV7M_PROFILE_TAG_PUSH(NAVIGATION);

    navigation_sensors.gyro_tick = tick;
    navigation_sensors.gyro_data[0] = gx;
    navigation_sensors.gyro_data[1] = gy;
    navigation_sensors.gyro_data[2] = gz;

    navigation_device.gyro_sequence = navigation_sequence(tick, 0);

    entry = &navigation_device.table[navigation_device.gyro_sequence & NAVIGATION_TABLE_MASK];
    
    if (entry->sequence != navigation_device.gyro_sequence)
    {
	entry->sequence = navigation_device.gyro_sequence;
	entry->rpm_count = 0;
	entry->gyro_count = 0;
		    
	gyro_delta = (float)(tick - navigation_sensors.rpm_tick);

	if (gyro_delta > navigation_sensors.rpm_delta)
	{
	    entry->rpm_accum = gyro_delta;
	}
	else
	{
	    entry->rpm_accum = navigation_sensors.rpm_delta;
	}
    }

    if (entry->gyro_count == 0)
    {
	entry->tick = navigation_threshold(tick, navigation_device.gyro_sequence);

	entry->gyro_accum = gz;
    }
    else
    {
	entry->gyro_accum += gz;
    }
    
    entry->gyro_count++;


    /* If there is a pending navigation location update, compute the new location.
     */
    if ((navigation_device.sequence +1) == navigation_device.gyro_sequence)
    {
	entry = &navigation_device.table[navigation_device.sequence & NAVIGATION_TABLE_MASK];

	if ((entry->sequence == navigation_device.sequence) && (entry->gyro_count != 0))
	{
	    navigation_predict(entry);

	    navigation_device.sequence++;

#if defined(SIMULATION)
	    {
		extern int doSimulation;

		if (doSimulation)
		{
		    printf("NAV_UPDATE(x=%f, y=%f, speed=%f, course=%f, rpm_scale=%f, gyro_scale=%f, gyro_bias=%f)\n",
			   navigation_device.location->x,
			   navigation_device.location->y,
			   navigation_device.location->speed,
			   navigation_device.location->course * RAD2DEG,
			   navigation_device.location->rpm_scale,
			   navigation_device.location->gyro_scale,
			   navigation_device.location->gyro_bias);
		}
	    }
#else
	    record_enter_extended(RECORD_TYPE_NAVIGATION, 0, tm4c123_capture_clock(), navigation_device.location, sizeof(navigation_location_t));
#endif
	}
    }

    if (((navigation_device.gps_sequence +1) == navigation_device.gyro_sequence) &&
	(navigation_device.gps_sequence < (navigation_device.gps_reference + NAVIGATION_TABLE_PREDICT_RATE)))
    {
	entry = &navigation_device.table[navigation_device.gps_sequence & NAVIGATION_TABLE_MASK];

	if ((entry->sequence == navigation_device.gps_sequence) && (entry->gyro_count != 0))
	{
	    navigation_resolve(entry, &rpm_speed, &gyro_rate);
			    
	    ekf_predict(rpm_speed, gyro_rate, NULL);
			    
	    entry->gyro_count = NAVIGATION_TABLE_GYRO_COUNT_DONE;

	    navigation_device.gps_sequence++;
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}

void navigation_mag_notify(uint64_t tick, float mx, float my, float mz)
{
    ARMV7M_PROFILE_TAG_PUSH(NAVIGATION);

    navigation_sensors.mag_tick = tick;
    navigation_sensors.mag_data[0] = mx;
    navigation_sensors.mag_data[1] = my;
    navigation_sensors.mag_data[2] = mz;

    navigation_sensors.mag_heading = angle_normalize(atan2f(-my, mx) + mission.variation);

    ARMV7M_PROFILE_TAG_POP();
}

void navigation_gps_notify(uint64_t tick, const gps_location_t *location)
{
    navigation_table_entry_t *entry;
    int ekf_reset;
    uint32_t sequence;
    float rpm_speed, gyro_rate, speed, course, ned[3];
    ekf_state_t ekf_state;
#if (NAVIGATION_CONFIG_DIFFERENTIAL == 1)
    float dx, dy, dT;
#endif /* NAVIGATION_CONFIG_DIFFERENTIAL == 1 */

    ARMV7M_PROFILE_TAG_PUSH(NAVIGATION);

    ekf_reset = 0;

    navigation_sensors.gps_tick = tick;
    navigation_sensors.gps_location = *location;

    if (location->type >= GPS_LOCATION_TYPE_2D)
    {
	/* gps_sequence needs to round up, so that it will
	 * hit the next bucket, rather than the previous
	 * bucket due to rounding down. The IMU/RPM bucketing
	 * needs to round down.
	 */

	navigation_device.gps_sequence = navigation_sequence(tick, 80);

	speed = (float)location->speed * 1e-3;
	course = (float)location->course * 1e-5 * DEG2RAD;

	if (navigation_device.home.latitude && navigation_device.home.longitude)
	{
	    for (sequence = navigation_device.gps_reference; sequence < navigation_device.gps_sequence; sequence++)
	    {
		entry = &navigation_device.table[sequence & NAVIGATION_TABLE_MASK];
		
		if ((entry->sequence != sequence) || (entry->gyro_count == 0))
		{
		    ekf_reset = 1;
		    break;
		}

		if (entry->gyro_count != NAVIGATION_TABLE_GYRO_COUNT_DONE)
		{
		    navigation_resolve(entry, &rpm_speed, &gyro_rate);

		    ekf_predict(rpm_speed, gyro_rate, NULL);

		    entry->gyro_count = NAVIGATION_TABLE_GYRO_COUNT_DONE;
		}
	    }

	    LLA2NED(location->latitude, location->longitude, location->altitude, &navigation_device.home, ned);

#if (NAVIGATION_CONFIG_DIFFERENTIAL == 1)
	    dT = (double)(tick - navigation_device.gps_tick) / (double)navigation_device.pps_period[1];
	    dx = ned[0] - navigation_device.gps_position[0];
	    dy = ned[1] - navigation_device.gps_position[1];

	    // printf("EKF_SCREWUP %f / %f\n", course * RAD2DEG, angle_normalize(atan2f(dy, dx)) * RAD2DEG);

	    speed = sqrtf(dx *dx + dy * dy) / dT;
	    course = angle_normalize(atan2f(dy, dx));
#endif /* NAVIGATION_CONFIG_DIFFERENTIAL == 1 */

	    entry = &navigation_device.table[navigation_device.gps_sequence & NAVIGATION_TABLE_MASK];
	    navigation_resolve(entry, &rpm_speed, &gyro_rate);

	    if ((speed < 0.1) && (rpm_speed > 10.0))
	    {
#if defined(SIMULATION)
		{
		    extern int doSimulation;

		    if (1 || doSimulation)
		    {
			printf("EKF_CRASH\n");
		    }
		}
#endif /* SIMULATION */
#if !defined(SIMULATION)
		control_crash();
#endif /* SIMULATION */
	    }

	    if (ekf_reset)
	    {
		ekf_reset = 0;
		
		if (speed < 0.1)
		{
		    ekf_initialize(ned[0], ned[1], speed, navigation_sensors.mag_heading, calibration.rpm_scale, calibration.gyro_scale, calibration.gyro_bias, &ekf_state);
		}
		else
		{
		    ekf_initialize(ned[0], ned[1], speed, course, calibration.rpm_scale, calibration.gyro_scale, calibration.gyro_bias, &ekf_state);
		}
	    }
	    else
	    {
		if (speed < 0.1)
		{
		    ekf_initialize(ned[0], ned[1], speed, navigation_sensors.mag_heading, calibration.rpm_scale, calibration.gyro_scale, calibration.gyro_bias, &ekf_state);
		}
		else
		{
		    ekf_correct(ned[0], ned[1], speed, course, &ekf_state);
		}
	    }

#if (NAVIGATION_CONFIG_DIFFERENTIAL == 1)
	    navigation_device.gps_tick         = tick;
	    navigation_device.gps_position[0]  = ned[0];
	    navigation_device.gps_position[1]  = ned[1];
	    navigation_device.gps_position[2]  = ned[2];
	    navigation_device.gps_speed        = speed;
	    navigation_device.gps_course       = course;
#endif /* CONFIG_DIFFERENTIAL == 1 */

	    /* Here we have a new ekf_state, so loop forward to produce the most current nav_state.
	     */
	    navigation_correct(tick, navigation_device.gps_sequence, ekf_state.x, ekf_state.y, ekf_state.speed, ekf_state.course, ekf_state.rpm_scale, ekf_state.gyro_scale, ekf_state.gyro_bias);

	    while (navigation_device.sequence < navigation_device.gyro_sequence)
	    {
		entry = &navigation_device.table[navigation_device.sequence & NAVIGATION_TABLE_MASK];

		if ((entry->sequence != navigation_device.sequence) || (entry->gyro_count == 0))
		{
		    /* If there is nothing to do, just wait till later */
		    break;
		}

		navigation_predict(entry);
						    
		navigation_device.sequence++;
	    }

#if defined(SIMULATION)
	    {
		extern int doSimulation;

		if (doSimulation)
		{
		    printf("NAV_UPDATE(x=%f, y=%f, speed=%f, course=%f, rpm_scale=%f, gyro_scale=%f, gyro_bias=%f)\n",
			   navigation_device.location->x,
			   navigation_device.location->y,
			   navigation_device.location->speed,
			   navigation_device.location->course * RAD2DEG,
			   navigation_device.location->rpm_scale,
			   navigation_device.location->gyro_scale,
			   navigation_device.location->gyro_bias);
		}
	    }
#else
	    record_enter_extended(RECORD_TYPE_NAVIGATION, 0, tm4c123_capture_clock(), navigation_device.location, sizeof(navigation_location_t));
#endif

	    navigation_device.gps_reference = navigation_device.gps_sequence;
	    
	    /* Loop ahead with all data that we see.
	     */
	    while ((navigation_device.gps_sequence < navigation_device.gyro_sequence) &&
		   (navigation_device.gps_sequence < (navigation_device.gps_reference + NAVIGATION_TABLE_PREDICT_RATE)))
	    {
		entry = &navigation_device.table[navigation_device.gps_sequence & NAVIGATION_TABLE_MASK];

		if ((entry->sequence != navigation_device.gps_sequence) || (entry->gyro_count == 0))
		{
		    /* If there is nothing to do, just wait till later */
		    break;
		}

		if (entry->gyro_count != NAVIGATION_TABLE_GYRO_COUNT_DONE)
		{
		    navigation_resolve(entry, &rpm_speed, &gyro_rate);

		    ekf_predict(rpm_speed, gyro_rate, NULL);

		    entry->gyro_count = NAVIGATION_TABLE_GYRO_COUNT_DONE;
		}
			    
		navigation_device.gps_sequence++;
	    }
	}
	else
	{
#if !defined(SIMULATION)
	    uint32_t status = control_status();

	    if ((status & CONTROL_STATUS_STATE_SET) && !(status & CONTROL_STATUS_STATE_HALT))
#endif
	    {
		if (location->type == GPS_LOCATION_TYPE_3D)
		{
		    LLA2HOME(mission.latitude, mission.longitude, mission.altitude, &navigation_device.home);
		    
		    ekf_initialize(0.0, 0.0, speed, navigation_sensors.mag_heading, calibration.rpm_scale, calibration.gyro_scale, calibration.gyro_bias, NULL);
		    
		    navigation_device.gps_reference = navigation_device.gps_sequence;

#if (NAVIGATION_CONFIG_DIFFERENTIAL == 1)
		    navigation_device.gps_tick        = tick;
		    navigation_device.gps_position[0] = ned[0];
		    navigation_device.gps_position[1] = ned[1];
		    navigation_device.gps_position[2] = ned[2];
		    navigation_device.gps_speed       = speed;
		    navigation_device.gps_course      = navigation_sensors.mag_heading;
#endif /* CONFIG_DIFFERENTIAL == 1 */
		}
	    }
	}
    }

    ARMV7M_PROFILE_TAG_POP();
}

int navigation_active(void)
{
    return ((navigation_device.location != NULL) && (navigation_sensors.gps_location.type == GPS_LOCATION_TYPE_3D));
}

int navigation_location(uint64_t tick, float *x, float *y, float *speed, float *course)
{
    navigation_location_t *location;
    float dT;

    location = navigation_device.location;

    if (location != NULL)
    {
	dT = (float)((tick - location->tick) / navigation_device.pps_period[1]);

	*x      = location->x + location->speed * dT * cosf(location->course);
	*y      = location->y + location->speed * dT * sinf(location->course);
	*speed  = location->speed;
	*course = angle_normalize(location->course);
    }

    return (location != NULL);

}

void navigation_initialize(void)
{
    navigation_device.pps_reference[0] = 0;
    navigation_device.pps_reference[1] = 0;
    navigation_device.pps_period[0]    = 80000000;
    navigation_device.pps_period[1]    = 80000000;
    navigation_device.pps_sequence[0]  = 0;
    navigation_device.pps_sequence[1]  = 0;

    navigation_device.rpm_sequence     = 0;
    navigation_device.gyro_sequence    = 0;
    navigation_device.gps_sequence     = 0; /* last PREDICT sequence */
    navigation_device.gps_reference    = 0; /* last CORRECT sequence */
#if (NAVIGATION_CONFIG_DIFFERENTIAL == 1)
    navigation_device.gps_tick        = 0;
    navigation_device.gps_position[0] = 0.0;
    navigation_device.gps_position[1] = 0.0;
    navigation_device.gps_position[2] = 0.0;
    navigation_device.gps_speed       = 0.0;
    navigation_device.gps_course      = 0.0;
#endif /* CONFIG_DIFFERENTIAL == 1 */
    navigation_device.sequence         = 0; /* last NAVIGATION sequence */
    navigation_device.location         = NULL;

    navigation_device.home.latitude    = 0;
    navigation_device.home.longitude   = 0;
    navigation_device.home.altitude    = 0;

#if !defined(SIMULATION)
    armv7m_pendsv_callback(PENDSV_SLOT_GPS, gps_pendsv_callback);
#endif
}



void LLA2HOME(int32_t latitude, int32_t longitude, int32_t altitude, lla_home_t *home )
{
    double lat, slat, clat, Rn, Rm;

    const double a = 6378137;
    const double e = 8.1819190842622e-2;
	
    lat = (double)latitude * (DEG2RAD / 1e7);
    
    slat = sin(lat);
    clat = cos(lat);
    
    Rn = a / sqrt(1 - e*e * slat*slat);
    Rm = (Rn * (1-e*e)) / (1 - e*e * slat*slat);

    home->latitude = latitude;
    home->longitude = longitude;
    home->altitude = altitude;
    home->S[0] = (double)(DEG2RAD / 1e7) * Rm;
    home->S[1] = (double)(DEG2RAD / 1e7) * Rn * clat;
    home->S[2] = (double)(1.0 / 1e3) * (-1.0);
}

void LLA2NED(int32_t latitude, int32_t longitude, int32_t altitude, const lla_home_t *home, float ned[3] )
{
    float dlat, dlon, dalt;

    dlat = (float)(latitude - home->latitude);
    dlon = (float)(longitude - home->longitude);
    dalt = (float)(altitude - home->altitude);

    ned[0] = home->S[0] * dlat;
    ned[1] = home->S[1] * dlon;
    ned[2] = home->S[2] * dalt;
}
