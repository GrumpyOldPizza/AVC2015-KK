/*
 * Copyright (c) 2014 Thomas Roell.  All rights reserved.
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

#define SIMULATION

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tm4c123_button.h"
#include "tm4c123_capture.h"
#include "tm4c123_receiver.h"

#include "ak8975.h"
#include "constants.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "gps.h"
#include "record.h"
#include "control.h"

#include "calibration.h"
#include "ekf_math.h"
#include "ekf_core.h"
#include "guidance.h"
#include "mission.h"
#include "navigation.h"

#include "calibration.c"
#include "ekf_math.c"
#include "ekf_core.c"
#include "guidance.c"
#include "mission.c"
#include "navigation.c"


typedef struct _control_legacy_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    uint16_t          steering;
    uint16_t          throttle;
    uint32_t          magic[2];
} control_legacy_entry_t;

int doSimulation = 0;
int doSort = 1;
int doPrint = 1;
int doRPMCal = 0;
int doMagCal = 0;
int doWaypoints = 0;

#define GRAVITY     9.80665f /* m/s^2 */
#define PI          3.14159265358979323846f
#define DEG2RAD     (PI / 180.0f)
#define RAD2DEG     (180.0f / PI)
#define KNOTS2MPS   0.514444444f
#define GAUSS2TESLA (0.0001f)

#define MAG_SAMPLE_COUNT 4

const float mag_variation = 8.51708;

uint32_t mag_index;
int32_t mag_accum[3];
			   
static const char *record_name[] = {
    "            NONE",
    "         MPU6050",
    "          AK8975",
    "         HMC5883",
    "             RPM",
    "             PPS",
    "    GPS_LOCATION",
    "  GPS_SATELLITES",
    "          BUTTON",
    "          SWITCH",
    "        RECEIVER",
    "  CONTROL_LEGACY",
    "         PROFILE",
    "             EKF",
    "      NAVIGATION",
    "        GUIDANCE",
    "         CONTROL",
};

static const unsigned int record_size[] = {
    sizeof(record_entry_t),
    sizeof(mpu6050_fifo_entry_t),
    sizeof(ak8975_fifo_entry_t),
    sizeof(hmc5883_fifo_entry_t),
    sizeof(tm4c123_rpm_fifo_entry_t),
    sizeof(tm4c123_pps_fifo_entry_t),
    sizeof(gps_location_entry_t),
    sizeof(gps_satellites_entry_t),
    sizeof(tm4c123_button_fifo_entry_t),
    sizeof(tm4c123_switch_fifo_entry_t),
    sizeof(tm4c123_receiver_entry_t),
    sizeof(control_legacy_entry_t),
    sizeof(profile_entry_t),
    sizeof(ekf_record_entry_t),
    sizeof(navigation_record_entry_t),
    sizeof(guidance_record_entry_t),
    sizeof(control_record_entry_t),
};

uint8_t record_data[64 * 1024 * 1024];

typedef struct _record_index_t {
    uint32_t offset;
    uint64_t time;
} record_index_t;

record_index_t record_index[8 * 1024 * 1024];

uint32_t record_offset;
uint32_t record_count;

int index_compare(const void *a, const void *b)
{
    record_index_t *index_a, *index_b;

    index_a = ((record_index_t*)a);
    index_b = ((record_index_t*)b);

    return (index_a->time - index_b->time);
}

int main(int argc, char *argv[])
{
    uint64_t time, tick, delta, pps_time[2], rpm_tick;
    unsigned int index, count, size, i, sequence;
    uint64_t cycles;
    uint32_t mag_index;
    int32_t mag_accum[3];
    record_entry_t *record_entry;
    float gps_position[2][3], dx, dy, distance;
    int gps_reset;
    lla_home_t home;

    pps_time[0] = 0;
    pps_time[1] = 0;

    gps_reset = 0;

    guidance_initialize();
    navigation_initialize();

    LLA2HOME(mission.latitude, mission.longitude, mission.altitude, &home);
    
    while (fread(&record_data[record_offset], sizeof(record_entry_t), 1, stdin) == 1)
    {
	record_entry = (record_entry_t*)&record_data[record_offset];

	size = (record_size[record_entry->type] +3) & ~3;

	if ((record_entry->type == RECORD_TYPE_GPS_LOCATION) && (record_entry->flags == 0))
	{
	    size -= (2 * sizeof(uint32_t));
	}

#if 0
	if ((record_entry->type == RECORD_TYPE_GPS_SATELLITES) && (record_entry->flags != 0))
	{
	    size = sizeof(gps_satellites_t) - sizeof(((gps_satellites_t*)NULL)->info) + (record_entry->flags * sizeof(((gps_satellites_t*)NULL)->info[0]));
	}
#endif

	if ((size == sizeof(record_entry_t)) ||
	    (fread(&record_data[record_offset+sizeof(record_entry_t)], (size - sizeof(record_entry_t)), 1, stdin) == 1))
	{
	    tick = ((uint64_t)record_entry->utime << 32) | (uint64_t)record_entry->ltime;

	    /* The PPS pulse is the time reference. If a tick was generated before the current 
	     * PPS pluse, it needs to use the old time teference. If it was generated after the
	     * current time reference, the new reference needs to be used.
	     */
	    
	    if (tick >= navigation_device.pps_reference[1])
	    {
		delta = tick - navigation_device.pps_reference[1];
		
		time = pps_time[1] + ((delta / navigation_device.pps_period[1]) * 1000000) + (((delta % navigation_device.pps_period[1]) * 1000000) / navigation_device.pps_period[1]);
	    }
	    else
	    {
		delta = tick - navigation_device.pps_reference[0];
		
		time = pps_time[0] + ((delta / navigation_device.pps_period[0]) * 1000000) + (((delta % navigation_device.pps_period[0]) * 1000000) / navigation_device.pps_period[0]);
	    }

	    if (record_entry->type == RECORD_TYPE_AK8975)
	    {
		float mxf, myf, mzf;
		ak8975_fifo_entry_t *entry;

		entry = (ak8975_fifo_entry_t*)record_entry;
		
		if (mag_index == 0)
		{
		    mag_accum[0] = entry->mx;
		    mag_accum[1] = entry->my;
		    mag_accum[2] = entry->mz;
		}
		else
		{
		    mag_accum[0] += entry->mx;
		    mag_accum[1] += entry->my;
		    mag_accum[2] += entry->mz;
		}
		
		mag_index++;
		
		if (mag_index == MAG_SAMPLE_COUNT)
		{
		    mag_index = 0;
		    
		    mxf = ((float)mag_accum[0] / (float)MAG_SAMPLE_COUNT) - calibration.mag_bias[0];
		    myf = ((float)mag_accum[1] / (float)MAG_SAMPLE_COUNT) - calibration.mag_bias[1];
		    mzf = ((float)mag_accum[2] / (float)MAG_SAMPLE_COUNT) - calibration.mag_bias[2];
		    
		    mxf = (mxf * calibration.mag_matrix[0][0] + myf * calibration.mag_matrix[0][1] + mzf * calibration.mag_matrix[0][2]);
		    myf = (myf * calibration.mag_matrix[1][0] + myf * calibration.mag_matrix[1][1] + myf * calibration.mag_matrix[1][2]);
		    mzf = (mzf * calibration.mag_matrix[2][0] + mzf * calibration.mag_matrix[2][1] + mzf * calibration.mag_matrix[2][2]);
		    
		    navigation_mag_notify(tick, mxf, myf, mzf);

		    if (doMagCal)
		    {
			printf("%lf %lf %lf\n",
			       ((float)mag_accum[0] / (float)MAG_SAMPLE_COUNT),
			       ((float)mag_accum[1] / (float)MAG_SAMPLE_COUNT),
			       ((float)mag_accum[2] / (float)MAG_SAMPLE_COUNT));
		    }
		}
	    }

	    if (record_entry->type == RECORD_TYPE_MPU6050)
	    {
		float axf, ayf, azf, gxf, gyf, gzf;
		mpu6050_fifo_entry_t *entry;

		entry = (mpu6050_fifo_entry_t*)record_entry;

		axf = (float)entry->ax * MPU6050_ACCEL_SCALE;
		ayf = (float)entry->ay * MPU6050_ACCEL_SCALE;
		azf = (float)entry->az * MPU6050_ACCEL_SCALE;

		axf = calibration.accel_scale[0] * (axf - calibration.accel_bias[0]);
		ayf = calibration.accel_scale[0] * (ayf - calibration.accel_bias[0]);
		azf = calibration.accel_scale[0] * (azf - calibration.accel_bias[0]);

		navigation_accel_notify(tick - (MPU6050_ACCEL_DELAY * 80), axf, ayf, azf);

		gxf = (float)entry->gx * MPU6050_GYRO_SCALE;
		gyf = (float)entry->gy * MPU6050_GYRO_SCALE;
		gzf = (float)entry->gz * MPU6050_GYRO_SCALE;
		
		navigation_gyro_notify(tick - (MPU6050_GYRO_DELAY * 80), gxf, gyf, gzf);

		time -= MPU6050_GYRO_DELAY;

		if (doSimulation)
		{
		    if ((entry->ax <= -32768) || (entry->ay <= -32768) || (entry->az <= -32768))
		    {
			printf("EKF_IMPACT\n");
		    }
		}

	    }

	    if (record_entry->type == RECORD_TYPE_RPM)
	    {
		navigation_rpm_notify(tick);
	    }

	    if (record_entry->type == RECORD_TYPE_GPS_LOCATION)
	    {
		gps_location_entry_t *entry;

		entry = (gps_location_entry_t*)record_entry;

		navigation_gps_notify(tick, &entry->location);

		if (!gps_reset)
		{
		    if (entry->location.type == GPS_LOCATION_TYPE_3D)
		    {
			LLA2NED(entry->location.latitude, entry->location.longitude, entry->location.altitude, &home, &gps_position[0][0]);

			gps_reset = 1;
		    }
		}
		else
		{
		    LLA2NED(entry->location.latitude, entry->location.longitude, entry->location.altitude, &home, &gps_position[1][0]);

		    dx = gps_position[1][0] - gps_position[0][0];
		    dy = gps_position[1][1] - gps_position[0][1];

		    distance = sqrt(dx * dx + dy * dy);

		    if (distance >= 2.0)
		    {
			if (doWaypoints)
			{
			    printf("%.7f,%.7f,1\n",
				   ((double)entry->location.latitude / (double)1e7),
				   ((double)entry->location.longitude / (double)1e7));
			}

			gps_position[0][0] = gps_position[1][0];
			gps_position[0][1] = gps_position[1][1];
		    }
		}
	    }

	    if (record_entry->type == RECORD_TYPE_PPS)
	    {
		tm4c123_pps_fifo_entry_t *entry;

		entry = (tm4c123_pps_fifo_entry_t*)record_entry;

		navigation_pps_notify(tick, entry->period);

		/* Move the time scale ahead ...
		 */
		delta = (tick - navigation_device.pps_reference[0]);
		
		pps_time[0] = pps_time[1];
		pps_time[1] = pps_time[0] + ((delta / navigation_device.pps_period[0]) * 1000000) + (((delta % navigation_device.pps_period[0]) * 1000000) / navigation_device.pps_period[0]);
	    }

	    record_index[record_count].offset = record_offset;
	    record_index[record_count].time = time;
	    record_count++;
	    record_offset += size;
	}
    }


    if (doSort)
    {
	qsort(record_index, record_count, sizeof(record_index_t), index_compare);
    }

    if (doPrint)
    {
	mag_index  = 0;

	for (index = 0; index < record_count; index++)
	{
	    record_entry = (record_entry_t*)&record_data[record_index[index].offset];

	    time = record_index[index].time;
	    tick = ((uint64_t)record_entry->utime << 32) | (uint64_t)record_entry->ltime;

	    printf("%s @ %d: ", record_name[record_entry->type], time);

	    switch (record_entry->type) {
		
	    case RECORD_TYPE_MPU6050: {
		float axf, ayf, azf, amf, gxf, gyf, gzf, gmf;
		mpu6050_fifo_entry_t *entry;

		entry = (mpu6050_fifo_entry_t*)record_entry;

		axf = (float)entry->ax * MPU6050_ACCEL_SCALE;
		ayf = (float)entry->ay * MPU6050_ACCEL_SCALE;
		azf = (float)entry->az * MPU6050_ACCEL_SCALE;

		axf = calibration.accel_scale[0] * (axf - calibration.accel_bias[0]);
		ayf = calibration.accel_scale[0] * (ayf - calibration.accel_bias[0]);
		azf = calibration.accel_scale[0] * (azf - calibration.accel_bias[0]);

		gxf = (float)entry->gx * MPU6050_GYRO_SCALE;
		gyf = (float)entry->gy * MPU6050_GYRO_SCALE;
		gzf = (float)entry->gz * MPU6050_GYRO_SCALE;

		printf("ACCEL=%d,%d,%d, GYRO=%d,%d,%d (%f %f %f, %f %f %f)", entry->ax, entry->ay, entry->az, entry->gx, entry->gy, entry->gz, axf, ayf, azf, gxf * RAD2DEG, gyf * RAD2DEG, gzf * RAD2DEG);

		if ((entry->ax <= -32767) || (entry->ay <= -32767) || (entry->az <= -32767))
		{
		    printf(" ACCEL_OVERFLOW");
		}

		if ((entry->gx <= -32767) || (entry->gy <= -32767) || (entry->gz <= -32767))
		{
		    printf(" GYRO_OVERFLOW");
		}

		if (entry->flags & MPU6050_FLAG_I2C_ERROR)
		{
		    printf(" I2C_ERROR");
		}
		
		if (entry->flags & MPU6050_FLAG_RECOVERY)
		{
		    printf(" REOVERY");
		}
		
		if (entry->flags & MPU6050_FLAG_FIFO_OVERFLOW)
		{
		    printf(" FIFO_OVERFLOW");
		}
		
		if (entry->flags & MPU6050_FLAG_GYRO_INVALID)
		{
		    printf(" GYRO_INVALID");
		}
		
		if (entry->flags & MPU6050_FLAG_ACCEL_INVALID)
		{
		    printf(" ACCEL_INVALID");
		}
		break;
	    }

	    case RECORD_TYPE_AK8975: {
		float mxf, myf, mzf, mmf, heading;
		ak8975_fifo_entry_t *entry;

		entry = (ak8975_fifo_entry_t*)record_entry;

		if (mag_index == 0)
		{
		    mag_accum[0] = entry->mx;
		    mag_accum[1] = entry->my;
		    mag_accum[2] = entry->mz;
		}
		else
		{
		    mag_accum[0] += entry->mx;
		    mag_accum[1] += entry->my;
		    mag_accum[2] += entry->mz;
		}

		mag_index++;

		if (mag_index == MAG_SAMPLE_COUNT)
		{
		    mag_index = 0;

		    mxf = ((float)mag_accum[0] / (float)MAG_SAMPLE_COUNT) - calibration.mag_bias[0];
		    myf = ((float)mag_accum[1] / (float)MAG_SAMPLE_COUNT) - calibration.mag_bias[1];
		    mzf = ((float)mag_accum[2] / (float)MAG_SAMPLE_COUNT) - calibration.mag_bias[2];

		    mxf = (mxf * calibration.mag_matrix[0][0] + myf * calibration.mag_matrix[0][1] + mzf * calibration.mag_matrix[0][2]);
		    myf = (myf * calibration.mag_matrix[1][0] + myf * calibration.mag_matrix[1][1] + myf * calibration.mag_matrix[1][2]);
		    mzf = (mzf * calibration.mag_matrix[2][0] + mzf * calibration.mag_matrix[2][1] + mzf * calibration.mag_matrix[2][2]);
		
		    mmf = sqrt(mxf * mxf + myf *myf + mzf * mzf);

		    heading = angle_normalize(atan2f(-myf, mxf) + mission.variation);

		    printf("MAG=%d,%d,%d (%f, %f, %f [%f]) HEADING=%f", entry->mx, entry->my, entry->mz, mxf, myf, mzf, mmf, heading * RAD2DEG);
		    
		    if (entry->flags & AK8975_FLAG_I2C_ERROR)
		    {
			printf(" I2C_ERROR");
		    }

		    if (entry->flags & AK8975_FLAG_FIFO_OVERFLOW)
		    {
			printf(" FIFO_OVERFLOW");
		    }

		    if (entry->flags & AK8975_FLAG_MAG_INVALID)
		    {
			printf(" MAG_INVALID");
		    }
		}
		else
		{
		    printf("MAG=%d,%d,%d", entry->mx, entry->my, entry->mz);
		}
		break;
	    }

	    case RECORD_TYPE_HMC5883: {
		hmc5883_fifo_entry_t *entry;
		
		entry = (hmc5883_fifo_entry_t*)record_entry;

		printf("MAG=%d,%d,%d", entry->mx, entry->my, entry->mz);
		break;
	    }

	    case RECORD_TYPE_RPM: {
		break;
	    }

	    case RECORD_TYPE_PPS: {
		tm4c123_pps_fifo_entry_t *entry;

		entry = (tm4c123_pps_fifo_entry_t*)record_entry;

		printf("PERIOD=%d", entry->period);
		break;
	    }

	    case RECORD_TYPE_GPS_LOCATION: {
		gps_location_entry_t *entry;

		static const char *gps_location_type[] = {
		    "NONE",
		    "TIME",
		    "2D",
		    "3D",
		};
		
		static const char *gps_location_quality[] = {
		    "",
		    "",
		    "/DGPS",
		    "/PP",
		    "/RTK",
		    "/RTK",
		    "/DR",
		    "/MANUAL",
		    "/SIMULATION",
		};
		
		entry = (gps_location_entry_t*)record_entry;

		if (entry->flags == 0)
		{
		    printf("UTC=%04d/%02d/%02d,%02d/%02d/%02.3f,%d, LLA=%.7f,%.7f,%.3f, SCC=%.3f,%.5f,%.3f, CLOCK=%d,%d, EPE=%.3f,%.3f, MODE=%s%s, NUMSV=%d, DOP=%.2f/%.2f/%.2f",
			   entry->location.time.year,
			   entry->location.time.month,
			   entry->location.time.day,
			   entry->location.time.hour,
			   entry->location.time.min,
			   ((double)entry->location.time.sec / (double)1e3),
			   entry->location.correction,
		       
			   ((double)entry->location.latitude / (double)1e7),
			   ((double)entry->location.longitude / (double)1e7),
			   ((double)entry->location.altitude / (double)1e3),
		       
			   ((double)entry->location.speed / (double)1e3),
			   ((double)entry->location.course / (double)1e5),
			   ((double)entry->location.climb / (double)1e3),
		       
			   entry->location.bias,
			   entry->location.drift,
		       
			   ((double)entry->location.ehpe / (double)1e3),
			   ((double)entry->location.evpe / (double)1e3),
		       
			   gps_location_type[entry->location.type],
			   gps_location_quality[entry->location.quality],
			   entry->location.numsv,
		       
			   ((double)entry->location.hdop / (double)1e2),
			   ((double)entry->location.vdop / (double)1e2),
			   ((double)entry->location.tdop / (double)1e2));
		}
		else
		{
		    printf("UTC=%04d/%02d/%02d,%02d/%02d/%02.3f,%d, LLA=%.7f,%.7f,%.3f, SCC=%.3f,%.5f,%.3f, CLOCK=%d,%d, EPE=%.3f,%.3f, EVE=%.3f,%.5f, MODE=%s%s, NUMSV=%d, DOP=%.2f/%.2f/%.2f",
			   entry->location.time.year,
			   entry->location.time.month,
			   entry->location.time.day,
			   entry->location.time.hour,
			   entry->location.time.min,
			   ((double)entry->location.time.sec / (double)1e3),
			   entry->location.correction,
		       
			   ((double)entry->location.latitude / (double)1e7),
			   ((double)entry->location.longitude / (double)1e7),
			   ((double)entry->location.altitude / (double)1e3),
		       
			   ((double)entry->location.speed / (double)1e3),
			   ((double)entry->location.course / (double)1e5),
			   ((double)entry->location.climb / (double)1e3),
		       
			   entry->location.bias,
			   entry->location.drift,
		       
			   ((double)entry->location.ehpe / (double)1e3),
			   ((double)entry->location.evpe / (double)1e3),

			   ((double)entry->location.esve / (double)1e3),
			   ((double)entry->location.ecve / (double)1e5),
		       
			   gps_location_type[entry->location.type],
			   gps_location_quality[entry->location.quality],
			   entry->location.numsv,
		       
			   ((double)entry->location.hdop / (double)1e2),
			   ((double)entry->location.vdop / (double)1e2),
			   ((double)entry->location.tdop / (double)1e2));
		}
		break;
	    }

	    case RECORD_TYPE_GPS_SATELLITES: {
		gps_satellites_entry_t *entry;
		unsigned int index, svid;

		entry = (gps_satellites_entry_t*)record_entry;

		printf("UTC=%04d/%02d/%02d,%02d/%02d/%02.3f,%d, NUMSV=%d\n",
		       entry->satellites.time.year,
		       entry->satellites.time.month,
		       entry->satellites.time.day,
		       entry->satellites.time.hour,
		       entry->satellites.time.min,
		       ((double)entry->satellites.time.sec / (double)1e3),
		       entry->satellites.correction,
		       entry->satellites.count);
    
		for (index = 0; index < entry->satellites.count; index++)
		{
		    svid = entry->satellites.info[index].id;

		    printf("\t\t\t");

		    if ((svid >= 1) && (svid <= 32))
		    {
			printf("G%d", svid);
		    }
		    else if ((svid >= 33) && (svid <= 64))
		    {
			printf("S%d", (svid + 87));
		    }
		    else if ((svid >= 65) && (svid <= 96))
		    {
			printf("R%d", (svid - 64));
		    }
		    else if ((svid >= 193) && (svid <= 197))
		    {
			printf("Q%d", (svid - 192));
		    }
		    else if ((svid >= 201) && (svid <= 237))
		    {
			printf("B%d", (svid - 200));
		    }
		    else if (svid == 255)
		    {
			printf("R?");
		    }
		    else
		    {
			printf("?%d", svid);
		    }
	
		    printf("(ELEV=%d, AZIM=%d, SNR=%d, STATE=%02x)\n",
			   entry->satellites.info[index].elev,
			   entry->satellites.info[index].azim,
			   entry->satellites.info[index].snr,
			   entry->satellites.info[index].state);
		}
		break;
	    }

	    case RECORD_TYPE_BUTTON: {
		tm4c123_button_fifo_entry_t *entry;

		static const char *button_event[] = {
		    "NONE",
		    "BUTTON_1_PRESS",
		    "BUTTON_1_TIMEOUT",
		    "BUTTON_1_RELEASE",
		    "BUTTON_2_PRESS",
		    "BUTTON_2_TIMEOUT",
		    "BUTTON_2_RELEASE",
		};

		entry = (tm4c123_button_fifo_entry_t*)record_entry;

		printf("%s", button_event[entry->flags]);
		break;
	    }

	    case RECORD_TYPE_SWITCH: {
		tm4c123_switch_fifo_entry_t *entry;

		static const char *switch_event[] = {
		    "NONE",
		    "SWITCH_ON",
		    "SWITCH_OFF",
		};

		entry = (tm4c123_switch_fifo_entry_t*)record_entry;

		printf("%s", switch_event[entry->flags]);
		break;
	    }

	    case RECORD_TYPE_RECEIVER: {
		tm4c123_receiver_entry_t *entry;

		entry = (tm4c123_receiver_entry_t*)record_entry;

		printf("CH%dIN=%d", entry->flags, entry->pulse);
		break;
	    }

	    case RECORD_TYPE_RESERVED_11: {
		control_legacy_entry_t *entry;

		entry = (control_legacy_entry_t*)record_entry;

		printf("STEERING=%d, THROTTLE=%d", entry->steering, entry->throttle);
		break;
	    }

	    case RECORD_TYPE_PROFILE: {
		profile_entry_t *entry;

		entry = (profile_entry_t*)record_entry;

		for (i = 0, cycles = 0; i < 16; i++)
		{
		    cycles += entry->cycles[i];
		}

		printf("STACK=(%08x, %08x)\n", entry->stack[0], entry->stack[1]);
		printf("\t\t\tOTHER      %5.2f\n", (100.0 * (double)entry->cycles[0] / (double)(cycles)));
		printf("\t\t\tRECEIVER   %5.2f\n", (100.0 * (double)entry->cycles[1] / (double)(cycles)));
		printf("\t\t\tCAPTURE    %5.2f\n", (100.0 * (double)entry->cycles[2] / (double)(cycles)));
		printf("\t\t\tI2C        %5.2f\n", (100.0 * (double)entry->cycles[3] / (double)(cycles)));
		printf("\t\t\tUART       %5.2f\n", (100.0 * (double)entry->cycles[4] / (double)(cycles)));
		printf("\t\t\tSERVO      %5.2f\n", (100.0 * (double)entry->cycles[5] / (double)(cycles)));
		printf("\t\t\tBUTTON     %5.2f\n", (100.0 * (double)entry->cycles[6] / (double)(cycles)));
		printf("\t\t\tSYSTICK    %5.2f\n", (100.0 * (double)entry->cycles[7] / (double)(cycles)));
		printf("\t\t\tCONTROL    %5.2f\n", (100.0 * (double)entry->cycles[8] / (double)(cycles)));
		printf("\t\t\tNAVIGATION %5.2f\n", (100.0 * (double)entry->cycles[9] / (double)(cycles)));
		printf("\t\t\tRECORD     %5.2f\n", (100.0 * (double)entry->cycles[10] / (double)(cycles)));
		printf("\t\t\tDISPLAY    %5.2f\n", (100.0 * (double)entry->cycles[11] / (double)(cycles)));
		printf("\t\t\tIDLE       %5.2f\n", (100.0 * (double)entry->cycles[15] / (double)(cycles)));
		break;
	    }

	    case RECORD_TYPE_EKF: {
		ekf_record_entry_t *entry;

		entry = (ekf_record_entry_t*)record_entry;

		if (entry->flags == EKF_EVENT_INITIALIZE)
		{
		    printf("INITIALIZE ");
		}
		
		if (entry->flags == EKF_EVENT_CORRECT)
		{
		    printf("CORRECT ");
		}
		
		if (entry->flags == EKF_EVENT_PREDICT)
		{
		    printf("PREDICT ");
		}
		
		printf("X=%f, Y=%f, SPEED=%f, COURSE=%f, RPM_SCALE=%f, GYRO_SCALE=%f, GYRO_BIAS=%f",
		       entry->state.x,
		       entry->state.y,
		       entry->state.speed,
		       entry->state.course * RAD2DEG,
		       entry->state.rpm_scale,
		       entry->state.gyro_scale,
		       entry->state.gyro_bias);
		break;
	    }

	    case RECORD_TYPE_NAVIGATION: {
		navigation_record_entry_t *entry;

		entry = (navigation_record_entry_t*)record_entry;

		printf("X=%f, Y=%f, SPEED=%f, COURSE=%f, RPM_SCALE=%f, GYRO_SCALE=%f, GYRO_BIAS=%f",
		       entry->location.x,
		       entry->location.y,
		       entry->location.speed,
		       entry->location.course * RAD2DEG,
		       entry->location.rpm_scale,
		       entry->location.gyro_scale,
		       entry->location.gyro_bias);
		break;
	    }

	    case RECORD_TYPE_GUIDANCE: {
		guidance_record_entry_t *entry;

		entry = (guidance_record_entry_t*)record_entry;

		printf("X=%f, Y=%f, SPEED=%f",
		       entry->target.position[0],
		       entry->target.position[1],
		       entry->target.speed);
		break;
	    }

	    case RECORD_TYPE_CONTROL: {
		control_record_entry_t *entry;

		entry = (control_record_entry_t*)record_entry;

		printf("STATUS=%08x, X=%f, Y=%f, SPEED=%f, COURSE=%f, TARGET_SPEED=%f, TARGET_COURSE=%f, STEERING=%f, THROTTLE=%f,%f,%f,%f, SERVO=%d,%d",
		       entry->state.status,
		       entry->state.x,
		       entry->state.y,
		       entry->state.speed,
		       entry->state.course * RAD2DEG,
		       entry->state.target_speed,
		       entry->state.target_course * RAD2DEG,
		       entry->state.steering_servo,
		       entry->state.throttle_integral,
		       entry->state.throttle_differential,
		       entry->state.throttle_delta,
		       entry->state.throttle_servo,
		       entry->state.servo_steering,
		       entry->state.servo_throttle);
		break;
	    }

	    default:
		break;
	    }
	
	    printf("\n");
	}
    }
}
