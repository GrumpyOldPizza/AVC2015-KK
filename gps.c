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


#include <stdint.h>
#include <string.h>
#include "kitty.h"
#include "gps.h"

/************************************************************************************/

#define GPS_PORT_UART_SEND(_data, _count)  tm4c123_uart_send((_data),(_count))
#define GPS_PORT_UART_CONFIGURE(_speed)    tm4c123_uart_configure((_speed))

/************************************************************************************/

#define NMEA_TALKER_MASK_GPS                          0x01
#define NMEA_TALKER_MASK_GLONASS                      0x02
#define NMEA_TALKER_MASK_GNSS                         0x04

#define NMEA_SENTENCE_MASK_GPGGA                      0x00000001
#define NMEA_SENTENCE_MASK_GPGSA                      0x00000002
#define NMEA_SENTENCE_MASK_GPGSV                      0x00000004
#define NMEA_SENTENCE_MASK_GPRMC                      0x00000008
#define NMEA_SENTENCE_MASK_GLGSA                      0x00000010
#define NMEA_SENTENCE_MASK_GLGSV                      0x00000020
#define NMEA_SENTENCE_MASK_SOLUTION                   0x00008000

#define NMEA_FIELD_SEQUENCE_START                     0
#define NMEA_FIELD_SEQUENCE_SKIP                      1
#define NMEA_FIELD_SEQUENCE_GGA_TIME                  2
#define NMEA_FIELD_SEQUENCE_GGA_LATITUDE              3
#define NMEA_FIELD_SEQUENCE_GGA_LATITUDE_NS           4
#define NMEA_FIELD_SEQUENCE_GGA_LONGITUDE             5
#define NMEA_FIELD_SEQUENCE_GGA_LONGITUDE_EW          6
#define NMEA_FIELD_SEQUENCE_GGA_QUALITY               7
#define NMEA_FIELD_SEQUENCE_GGA_NUMSV                 8
#define NMEA_FIELD_SEQUENCE_GGA_HDOP                  9
#define NMEA_FIELD_SEQUENCE_GGA_ALTITUDE              10
#define NMEA_FIELD_SEQUENCE_GGA_ALTITUDE_UNIT         11
#define NMEA_FIELD_SEQUENCE_GGA_SEPARATION            12
#define NMEA_FIELD_SEQUENCE_GGA_SEPARATION_UNIT       13
#define NMEA_FIELD_SEQUENCE_GGA_DIFFERENTIAL_AGE      14
#define NMEA_FIELD_SEQUENCE_GGA_DIFFERENTIAL_STATION  15
#define NMEA_FIELD_SEQUENCE_GSA_OPERATION             16
#define NMEA_FIELD_SEQUENCE_GSA_NAVIGATION            17
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_1         18
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_2         19
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_3         20
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_4         21
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_5         22
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_6         23
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_7         24
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_8         25
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_9         26
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_10        27
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_11        28
#define NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_12        29
#define NMEA_FIELD_SEQUENCE_GSA_PDOP                  30
#define NMEA_FIELD_SEQUENCE_GSA_HDOP                  31
#define NMEA_FIELD_SEQUENCE_GSA_VDOP                  32
#define NMEA_FIELD_SEQUENCE_GSV_SENTENCES             33
#define NMEA_FIELD_SEQUENCE_GSV_CURRENT               34
#define NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_COUNT      35
#define NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_ID         36
#define NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_ELEV       37
#define NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_AZIM       38
#define NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_SNR        39
#define NMEA_FIELD_SEQUENCE_RMC_TIME                  40
#define NMEA_FIELD_SEQUENCE_RMC_STATUS                41
#define NMEA_FIELD_SEQUENCE_RMC_LATITUDE              42
#define NMEA_FIELD_SEQUENCE_RMC_LATITUDE_NS           43
#define NMEA_FIELD_SEQUENCE_RMC_LONGITUDE             44
#define NMEA_FIELD_SEQUENCE_RMC_LONGITUDE_EW          45
#define NMEA_FIELD_SEQUENCE_RMC_SPEED                 46
#define NMEA_FIELD_SEQUENCE_RMC_COURSE                47
#define NMEA_FIELD_SEQUENCE_RMC_DATE                  48
#define NMEA_FIELD_SEQUENCE_RMC_VARIATION             49
#define NMEA_FIELD_SEQUENCE_RMC_VARIATION_UNIT        50
#define NMEA_FIELD_SEQUENCE_RMC_MODE                  51
#define NMEA_FIELD_SEQUENCE_GGA_END                   52
#define NMEA_FIELD_SEQUENCE_GSA_END                   53
#define NMEA_FIELD_SEQUENCE_GSV_END                   54
#define NMEA_FIELD_SEQUENCE_RMC_END                   55

#if (GPS_CONFIG_MEDIATEK >= 1)
#define NMEA_FIELD_SEQUENCE_PMTK001_COMMAND           56
#define NMEA_FIELD_SEQUENCE_PMTK001_STATUS            57
#define NMEA_FIELD_SEQUENCE_PMTK010_STATUS            58
#define NMEA_FIELD_SEQUENCE_PMTK869_MODE              59
#define NMEA_FIELD_SEQUENCE_PMTK869_STATUS            60
#define NMEA_FIELD_SEQUENCE_PMTK001_END               61
#define NMEA_FIELD_SEQUENCE_PMTK010_END               62
#define NMEA_FIELD_SEQUENCE_PMTK869_END               63
#endif /* GPS_CONFIG_MEDIATEK >= 1 */

#define NMEA_FIELD_MASK_TIME                          0x0001
#define NMEA_FIELD_MASK_POSITION                      0x0002
#define NMEA_FIELD_MASK_ALTITUDE                      0x0004
#define NMEA_FIELD_MASK_SPEED                         0x0008
#define NMEA_FIELD_MASK_COURSE                        0x0010
#define NMEA_FIELD_MASK_HDOP                          0x0020
#define NMEA_FIELD_MASK_VDOP                          0x0040

#define NMEA_OPERATION_MANUAL                         0
#define NMEA_OPERATION_AUTOMATIC                      1

#define NMEA_NAVIGATION_NONE                          0
#define NMEA_NAVIGATION_2D                            1
#define NMEA_NAVIGATION_3D                            2

#define NMEA_STATUS_RECEIVER_WARNING                  0
#define NMEA_STATUS_DATA_VALID                        1

typedef struct _nmea_context_t {
    uint8_t           talker;                             /* NMEA TALKER                  */
    uint8_t           sequence;                           /* FIELD SEQUENCE               */
    uint16_t          mask;                               /* FIELD MASK                   */
    uint8_t           navigation;                         /* GSA                          */
    uint8_t           status;                             /* RMC                          */
#if (GPS_CONFIG_SATELLITES >= 1)
    uint8_t           sv_in_view_sentences;               /* GSV                          */
    uint8_t           sv_in_view_count;                   /* GSV                          */
    uint8_t           sv_in_view_index;                   /* GSV                          */
    uint8_t           sv_used_count;                      /* GSA                          */
    uint32_t          sv_used_mask[3];                    /* GSA                          */
#endif /* GPS_CONFIG_SATELLITES >= 1 */
#if (GPS_CONFIG_MEDIATEK >= 1)
    uint16_t          mtk_command;
    uint16_t          mtk_status;
#endif /* GPS_CONFIG_MEDIATEK >= 1 */
} nmea_context_t;

/************************************************************************************/

#define SRF_MESSAGE_MASK_MEASURED_TRACKER             0x00010000
#define SRF_MESSAGE_MASK_CLOCK_STATUS                 0x00020000
#define SRF_MESSAGE_MASK_GEODETIC_NAVIGATION          0x00040000
#define SRF_MESSAGE_MASK_SOLUTION                     0x00008000

typedef struct _srf_context_t {
    uint32_t          week;
    uint32_t          tow;
#if (GPS_CONFIG_SATELLITES >= 1)
    uint32_t          sv_used_mask;
#endif /* GPS_CONFIG_SATELLITES >= 1 */
} srf_context_t;

/************************************************************************************/

#define UBX_MESSAGE_MASK_NAV_CLOCK                    0x00010000
#define UBX_MESSAGE_MASK_NAV_DOP                      0x00020000
#define UBX_MESSAGE_MASK_NAV_POSLLH                   0x00040000
#define UBX_MESSAGE_MASK_NAV_PVT                      0x00080000
#define UBX_MESSAGE_MASK_NAV_SOL                      0x00100000
#define UBX_MESSAGE_MASK_NAV_SVINFO                   0x00200000
#define UBX_MESSAGE_MASK_NAV_TIMEGPS                  0x00400000
#define UBX_MESSAGE_MASK_NAV_TIMEUTC                  0x00800000
#define UBX_MESSAGE_MASK_NAV_VELNED                   0x01000000
#define UBX_MESSAGE_MASK_SOLUTION                     0x00008000

typedef struct _ubx_context_t {
    uint32_t          week;
    uint32_t          tow;
    uint32_t          itow;
} ubx_context_t;

/************************************************************************************/

#define GPS_PROTOCOL_MASK_NMEA       0x01
#define GPS_PROTOCOL_MASK_SRF        0x02
#define GPS_PROTOCOL_MASK_UBX        0x04

#define GPS_STATE_START              0
#define GPS_STATE_NMEA_PAYLOAD       1
#define GPS_STATE_NMEA_CHECKSUM_1    2
#define GPS_STATE_NMEA_CHECKSUM_2    3
#define GPS_STATE_NMEA_END_CR        4
#define GPS_STATE_NMEA_END_LF        5
#define GPS_STATE_SRF_START_2        6
#define GPS_STATE_SRF_LENGTH_1       7
#define GPS_STATE_SRF_LENGTH_2       8
#define GPS_STATE_SRF_MESSAGE        9
#define GPS_STATE_SRF_PAYLOAD        10
#define GPS_STATE_SRF_CHECKSUM_1     11
#define GPS_STATE_SRF_CHECKSUM_2     12
#define GPS_STATE_SRF_END_1          13
#define GPS_STATE_SRF_END_2          14
#define GPS_STATE_UBX_SYNC_2         15
#define GPS_STATE_UBX_MESSAGE_1      16
#define GPS_STATE_UBX_MESSAGE_2      17
#define GPS_STATE_UBX_LENGTH_1       18
#define GPS_STATE_UBX_LENGTH_2       19
#define GPS_STATE_UBX_PAYLOAD        20
#define GPS_STATE_UBX_CK_A           21
#define GPS_STATE_UBX_CK_B           22

#define GPS_INIT_DONE                               0

#define GPS_INIT_MTK_BAUD_RATE                      1
#define GPS_INIT_MTK_STARTUP                        2
#define GPS_INIT_MTK_INIT_TABLE                     128

#define GPS_INIT_SRF_BAUD_RATE                      3
#define GPS_INIT_SRF_STARTUP                        4
#define GPS_INIT_SRF_INIT_TABLE                     128

#define GPS_INIT_UBX_BAUD_RATE                      5
#define GPS_INIT_UBX_STARTUP                        6
#define GPS_INIT_UBX_INIT_TABLE                     128

#define GPS_RESPONSE_NONE                           0
#define GPS_RESPONSE_ACK                            1
#define GPS_RESPONSE_NACK                           2
#define GPS_RESPONSE_STARTUP                        3
#define GPS_RESPONSE_NMEA_SENTENCE                  4
#define GPS_RESPONSE_SRF_MESSAGE                    5
#define GPS_RESPONSE_UBX_MESSAGE                    6

#define GPS_DATA_SIZE         96

typedef struct _gps_device_t {
    uint8_t           reset;
    uint8_t           protocol;
    uint8_t           init;
    uint8_t           state;
    uint32_t          seen;
    uint32_t          expected;
    uint16_t          checksum;
    uint8_t           ck_a;
    uint8_t           ck_b;
    uint16_t          count;
    uint16_t          length;
    uint16_t          first;
    uint16_t          last;
    uint16_t          message;
    uint8_t           data[GPS_DATA_SIZE];
#if (GPS_CONFIG_SIRF == 0) && (GPS_CONFIG_UBLOX == 0)
    nmea_context_t    nmea;
#endif /* GPS_CONFIG_SIRF == 0 && GPS_CONFIG_UBLOX == 0 */
#if (GPS_CONFIG_SIRF >= 1)
    srf_context_t     srf;
#endif /* GPS_CONFIG_SIRF >= 1 */
#if (GPS_CONFIG_UBLOX >= 1)
    ubx_context_t     ubx;
#endif /* GPS_CONFIG_UBLOX >= 1 */
    gps_location_t    location;
#if (GPS_CONFIG_SATELLITES >= 1)
    gps_satellites_t  satellites;
#endif /* GPS_CONFIG_SATELLITES >= 1 */
    volatile uint32_t command;
    uint64_t          tick;                     /* CPU time @ last solution          */
    utc_time_t        pps_time;                 /* UTC time @ last PPS pulse         */
    uint8_t           pps_correction;           /* GPS/UTC offset @ last PPS pulse   */
    volatile uint8_t  pps_pending;              /* pending PPS pulse                 */
    volatile uint64_t pps_reference;            /* CPU ticks @ pending PPS pulse     */
    uint64_t          pps_tick;                 /* CPU ticks @ accepted PPS pulse    */
    uint32_t          pps_period;               /* ticks per second                  */
    gps_ini_callback_t ini_callback;
    gps_location_callback_t location_callback;
    gps_satellites_callback_t satellites_callback;
} gps_device_t;

static gps_device_t gps_device;

#if (GPS_CONFIG_MEDIATEK >= 1)
static void mtk_initialize(gps_device_t *device, unsigned int response, uint32_t command);
#endif /* GPS_CONFIG_MEDIATEK >= 1 */
#if (GPS_CONFIG_SIRF >= 1)
static void srf_initialize(gps_device_t *device, unsigned int response, uint32_t command);
#endif /* GPS_CONFIG_UBLOX >= 1 */
#if (GPS_CONFIG_UBLOX >= 1)
static void ubx_initialize(gps_device_t *device, unsigned int response, uint32_t command);
#endif /* GPS_CONFIG_UBLOX >= 1 */

/************************************************************************************/

static const uint16_t utc_days_since_month[2][12] = {
    {   0,  31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334, },
    {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335, },
};

static int utc_diff_time(const utc_time_t *t0, uint32_t offset0, const utc_time_t *t1, uint32_t offset1)
{
    /* The difference between t1 time and the current time is mostly a few seconds,
     * but the arithmetic might overflow, so be careful to incrementally compute the difference
     * in days, minutes, seconds, milliseconds.
     */
  return (((((((((((t0->year * 365 + (1 + ((t0->year -1) / 4))) +
		   utc_days_since_month[(t0->year & 3) == 0][t0->month -1] +
		   (t0->day -1)) -
		  ((t1->year * 365 + (1 + ((t1->year -1) / 4))) +
		   utc_days_since_month[(t1->year & 3) == 0][t1->month -1] +
		   (t1->day -1))) * 24) +
		t0->hour -
		t1->hour) * 60) +
	      t0->min -
	      t1->min) * 60) +
	    offset0 -
	    offset1) * 1000) +
	  t0->sec -
	  t1->sec);
}

/* utc_offset_time:
 *
 * Compute the UTC offset (or GPS leap second) by computing the elapsed UTC seconds
 * since 06/01/1980, and subtract that from week/tow (which is ahead by said 
 * leap seconds).
 */

static int utc_offset_time(const utc_time_t *time, uint16_t week, uint32_t tow)
{
    return ((((uint32_t)week * 604800) + ((tow + 5) / 1000)) -
	    (((((((((time->year * 365 + (1 + ((time->year -1) / 4))) +
		    utc_days_since_month[(time->year & 3) == 0][time->month -1] +
		    (time->day -1)) -
		   ((1980 * 365 + (1 + ((1980 -1) / 4))) +
		    (6 -1))) * 24) +
		 time->hour) * 60) +
	       time->min ) * 60) +
	     ((time->sec + 5) / 1000)));
}

/************************************************************************************/

static void gps_configure(gps_device_t *device, uint32_t speed)
{
    GPS_PORT_UART_CONFIGURE(speed);
}

static void gps_send(gps_device_t *device, uint32_t command, const uint8_t *data, unsigned int count)
{
    device->command = command;
    
    GPS_PORT_UART_SEND(data, count);
}

static void gps_location(gps_device_t *device)
{
    int delta;
    unsigned int sec, msec;

    if ((device->location.type != GPS_LOCATION_TYPE_NONE) && (device->pps_time.year == 0))
      {
	device->pps_pending = 1;
	device->pps_reference = 0;
      }
    
    switch (device->location.type) {
    case GPS_LOCATION_TYPE_NONE:
	/* If there was a reference PPS plus and a fix, then allow the
	 * UTC time to freewheel.
	 */
	if (device->pps_time.year != 0)
	{
	    device->location.mask &= (GPS_LOCATION_MASK_TIME |
				      GPS_LOCATION_MASK_CORRECTION |
				      GPS_LOCATION_MASK_CLOCK);
	}
	else
	{
	    device->location.mask = 0;
	}

	device->location.numsv = 0;
	device->location.quality = GPS_LOCATION_QUALITY_NONE;
	break;

    case GPS_LOCATION_TYPE_TIME:
	device->location.mask &= (GPS_LOCATION_MASK_TIME |
				  GPS_LOCATION_MASK_CORRECTION |
				  GPS_LOCATION_MASK_CLOCK |
				  GPS_LOCATION_MASK_TDOP);

	device->location.quality = GPS_LOCATION_QUALITY_NONE;
	break;

    case GPS_LOCATION_TYPE_2D:
	device->location.mask &= (GPS_LOCATION_MASK_TIME |
				  GPS_LOCATION_MASK_CORRECTION |
				  GPS_LOCATION_MASK_POSITION |
				  GPS_LOCATION_MASK_SPEED |
				  GPS_LOCATION_MASK_COURSE |
				  GPS_LOCATION_MASK_CLOCK |
				  GPS_LOCATION_MASK_EHPE |
				  GPS_LOCATION_MASK_HDOP |
				  GPS_LOCATION_MASK_TDOP);
	break;

    case GPS_LOCATION_TYPE_3D:
	break;

    default:
	break;
    }

    if (device->location.mask & GPS_LOCATION_MASK_TIME)
    {
	if (!(device->location.mask & GPS_LOCATION_MASK_CORRECTION))
	{
	    device->location.correction = 0;
	}

	sec = device->location.time.sec / 1000;
	msec = device->location.time.sec - (sec * 1000);

	// Fake out PPS pending
	if ((device->location.type != GPS_LOCATION_TYPE_NONE) && device->pps_pending && (msec == 0))
	{
	    device->pps_time = device->location.time;
	    device->pps_correction = device->location.correction;
	    device->pps_tick = device->pps_reference;
	    device->pps_pending = 0;

	    device->tick = device->pps_tick;

	    device->location.mask |= GPS_LOCATION_MASK_PPS;
	}
	else
	{
	    if (msec == 0)
	    {
		device->pps_pending = 0;
	    }

	    if (device->pps_time.year != 0)
	    {
		/* Here the time stamping has to be done along the UTC time. The problem
		 * with that is that leap seconds can pop up, mainly not because the rare response
		 * of an actual leap second, but because the reciever starts off with a build
		 * in leap second that is only later on resolved via satellite. So the code has
		 * to find out about that event by looking at the UTC time going backwards in
		 * case we don't know the currently used leap second offset. That is all fine and
		 * dandy if every single solution is visible on the wire, and no data ever gets lost.
		 */
	    
		delta = utc_diff_time(&device->location.time, device->location.correction, &device->pps_time, device->pps_correction);

		while (delta <= 0)
		{
		    delta += 1000;
		}
		
		sec = (uint32_t)delta / 1000;
		msec = (uint32_t)delta - (sec * 1000);

		device->tick = device->pps_tick + ((uint64_t)sec * device->pps_period) + ((msec / (1000 / GPS_CONFIG_RATE)) * (device->pps_period / GPS_CONFIG_RATE));
	    }
	    else
	    {
		device->tick = 0;
	    }
	}
    }
    else
    {
        device->tick = 0;

	device->location.time.year  = 0;
	device->location.time.month = 0;
	device->location.time.day   = 0;
	device->location.time.hour  = 0;
	device->location.time.min   = 0;
	device->location.time.sec   = 0;
	device->location.correction = 0;

	device->location.mask = 0;
	device->location.numsv = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_POSITION))
    {
	device->location.latitude = 0;
	device->location.longitude = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_ALTITUDE))
    {
	device->location.altitude = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_SPEED))
    {
	device->location.speed = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_COURSE))
    {
	device->location.course = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_CLIMB))
    {
	device->location.climb = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_CLOCK))
    {
	device->location.bias = 0;
	device->location.drift = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_EHPE))
    {
	device->location.ehpe = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_EVPE))
    {
	device->location.evpe = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_ESVE))
    {
	device->location.esve = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_ECVE))
    {
	device->location.ecve = 0;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_HDOP))
    {
	device->location.hdop = 9999;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_VDOP))
    {
	device->location.vdop = 9999;
    }

    if (!(device->location.mask & GPS_LOCATION_MASK_TDOP))
    {
	device->location.tdop = 9999;
    }

    if (device->location_callback)
    {
	(*device->location_callback)(device->tick, &device->location);
    }

#if (GPS_CONFIG_SATELLITES >= 1)
    device->satellites.time = device->location.time;
    device->satellites.correction = device->location.correction;
    device->satellites.mask |= (device->location.mask & (GPS_LOCATION_MASK_TIME | GPS_LOCATION_MASK_CORRECTION));
#endif /* GPS_CONFIG_SATELLITES >= 1 */

    device->location.type = 0;
    device->location.mask = 0;
}

#if (GPS_CONFIG_SATELLITES >= 1)

static void gps_satellites(gps_device_t *device)
{
    if (device->satellites.count > GPS_SATELLITES_COUNT_MAX)
    {
	device->satellites.count = GPS_SATELLITES_COUNT_MAX;
    }

    if (device->satellites_callback)
    {
	(*device->satellites_callback)(device->tick, &device->satellites);
    }

    device->satellites.mask = 0;
}

#endif /* GPS_CONFIG_SATELLITES >= 1 */

/************************************************************************************/

static const char nmea_hex_ascii[16] = "0123456789ABCDEF";

static void nmea_send(gps_device_t *device, const char *data)
{
  gps_send(&gps_device, ~0l, (const uint8_t*)data, strlen(data));
}

#if (GPS_CONFIG_SIRF == 0) && (GPS_CONFIG_UBLOX == 0)

static const uint32_t nmea_scale[10] = {
    1,
    10,
    100,
    1000,
    10000,
    100000,
    1000000,
    10000000,
    100000000,
    1000000000,
};

static int nmea_same_time(const utc_time_t *t0, const utc_time_t *t1)
{
    return ((t0->sec  == t1->sec) &&
            (t0->min  == t1->min) &&
            (t0->hour == t1->hour));
}

static int nmea_parse_time(const uint8_t *data, utc_time_t *p_time)
{
    uint32_t hour, min, sec, msec, digits;

    if ((data[0] >= '0') && (data[0] <= '9') && (data[1] >= '0') && (data[1] <= '9'))
    {
	hour = (data[0] - '0') * 10 + (data[1] - '0');
	data += 2;
	
	if ((hour < 24) && (data[0] >= '0') && (data[0] <= '9') && (data[1] >= '0') && (data[1] <= '9'))
	{
	    min = (data[0] - '0') * 10 + (data[1] - '0');
	    data += 2;
	    
	    if ((min < 60) && (data[0] >= '0') && (data[0] <= '9') && (data[1] >= '0') && (data[1] <= '9'))
	    {
		sec = (data[0] - '0') * 10 + (data[1] - '0');
		data += 2;
		
		/* A 60 is legal here for leap seconds.
		 */
		if (sec <= 60)
		{
		    msec = 0;
		    
		    if (data[0] == '.')
		    {
			digits = 0;
			data++;
			
			while ((data[0] >= '0') && (data[0] <= '9'))
			{
			    msec = msec * 10 + (data[0] - '0');
			    digits++;
			    data++;
			}
			
			if (data[0] == '\0')
			{
			    if (digits < 3)
			    {
				msec = msec * nmea_scale[3 - digits];
			    }
			    else if (digits > 3)
			    {
				msec = (msec + (nmea_scale[digits - 3] / 2)) / nmea_scale[digits - 3];
			    }
			}
		    }
		    
		    if (data[0] == '\0')
		    {
			p_time->hour = hour;
			p_time->min  = min;
			p_time->sec  = (sec * 1000) + msec;

			return 1;
		    }
		}
	    }
	}
    }

    return 0;
}

static int nmea_parse_unsigned(const uint8_t *data, uint32_t *p_unsigned)
{
    uint32_t integer;

    integer = 0;
                    
    while ((data[0] >= '0') && (data[0] <= '9'))
    {
	integer = integer * 10 + (data[0] - '0');
	data++;
    }

    if (data[0] == '\0')
    {
	*p_unsigned = integer;
        
	return 1;
    }

    return 0;
}

static int nmea_parse_fixed(const uint8_t *data, uint32_t *p_fixed, uint32_t scale)
{
    uint32_t integer, fraction, digits;

    integer = 0;
    
    while ((data[0] >= '0') && (data[0] <= '9'))
    {
	integer = integer * 10 + (data[0] - '0');
	data++;
    }
    
    
    fraction = 0;
    
    if (data[0] == '.')
    {
	digits = 0;
	data++;
        
	while ((data[0] >= '0') && (data[0] <= '9'))
	{
	    fraction = fraction * 10 + (data[0] - '0');
	    digits++;
	    data++;
	}
        
	if (data[0] == '\0')
	{
	    if (digits < scale)
	    {
		fraction = fraction * nmea_scale[scale - digits];
	    }
	    else if (digits > scale)
	    {
		fraction = (fraction + (nmea_scale[digits - scale] / 2)) / nmea_scale[digits - scale];
	    }
	}
    }
    
    if (data[0] == '\0')
    {
	*p_fixed = integer * nmea_scale[scale] + fraction;
        
	return 1;
    }
    
    return 0;
}

static int nmea_parse_latitude(const uint8_t *data, uint32_t *p_latitude)
{
    uint32_t degrees, minutes;
    
    if ((data[0] >= '0') && (data[0] <= '9') && (data[1] >= '0') && (data[1] <= '9'))
    {
	degrees = (data[0] - '0') * 10 + (data[1] - '0');
	data += 2;
        
	if ((degrees < 90) && (data[0] != '\0') && nmea_parse_fixed(data, &minutes, 7))
	{
	    if (minutes < 600000000)
	    {
		*p_latitude = (uint32_t)(degrees * 10000000u) + (uint32_t)((minutes + 30) / 60);
		
		return 1;
	    }
	}
    }

    return 0;
}

static int nmea_parse_longitude(const uint8_t *data, uint32_t *p_longitude)
{
    uint32_t degrees, minutes;

    if ((data[0] >= '0') && (data[0] <= '9') && (data[1] >= '0') && (data[1] <= '9') && (data[2] >= '0') && (data[2] <= '9'))
    {
	degrees = (data[0] - '0') * 100 + (data[1] - '0') * 10 + (data[2] - '0');
	data += 3;
        
	if ((degrees < 180) && (data[0] != '\0') && nmea_parse_fixed(data, &minutes, 7))
	{
	    if (minutes < 600000000)
	    {
		*p_longitude = (uint32_t)(degrees * 10000000u) + (uint32_t)((minutes + 30) / 60);
		
		return 1;
	    }
	}
    }
    
    return 0;
}

static void nmea_start_sentence(gps_device_t *device)
{
    nmea_context_t *context = &device->nmea;

    switch (context->sequence) {

    case NMEA_FIELD_SEQUENCE_GGA_END:
	break;

    case NMEA_FIELD_SEQUENCE_GSA_END:
#if (GPS_CONFIG_SATELLITES >= 1)
	context->sv_used_count = 0;
	context->sv_used_mask[0] = 0;
	context->sv_used_mask[1] = 0;
	context->sv_used_mask[2] = 0;
#endif /* GPS_CONFIG_SATELLITES >= 1 */
	break;

    case NMEA_FIELD_SEQUENCE_GSV_END:
#if (GPS_CONFIG_SATELLITES >= 1)
	context->sv_in_view_sentences = 0;
#endif /* GPS_CONFIG_SATELLITES >= 1 */
	break;

    case NMEA_FIELD_SEQUENCE_RMC_END:
	break;

#if (GPS_CONFIG_MEDIATEK >= 1)
    case NMEA_FIELD_SEQUENCE_PMTK001_END:
	break;

    case NMEA_FIELD_SEQUENCE_PMTK010_END:
	break;

    case NMEA_FIELD_SEQUENCE_PMTK869_END:
	break;
#endif /* GPS_CONFIG_MEDIATEK >= 1 */

    default:
	break;
    }

    context->sequence = NMEA_FIELD_SEQUENCE_START;
}

static void nmea_parse_sentence(gps_device_t *device, const uint8_t *data, unsigned int length)
{
    nmea_context_t *context = &device->nmea;
    uint32_t sequence, sequence_next;
    uint32_t latitude, longitude, altitude, separation, speed, course;
    uint32_t quality, numsv, hdop, vdop, date;
    uint32_t command, status;
    utc_time_t time;
#if (GPS_CONFIG_SATELLITES >= 1)
    uint32_t count, index, sentences, current, svid, elev, azim, snr;
#endif /* GPS_CONFIG_SATELLITES >= 1 */

    sequence = context->sequence;
    sequence_next = sequence +1;

    switch (sequence) {

    case NMEA_FIELD_SEQUENCE_START:
	sequence_next = NMEA_FIELD_SEQUENCE_SKIP;

	if (data[0] == 'P')
	{
#if (GPS_CONFIG_MEDIATEK >= 1)
	    if (!strcmp((const char*)data, "PMTK001"))
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_PMTK001_COMMAND;
	    }

	    else if (!strcmp((const char*)data, "PMTK010"))
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_PMTK010_STATUS;
	    }
	    else if (!strcmp((const char*)data, "PMTK869"))
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_PMTK869_MODE;
	    }
#endif /* GPS_CONFIG_MEDIATEK */
	}
	else
	{
	    context->talker = 0;

	    if (data[0] == 'G')
	    {
		if (data[1] == 'P')
		{
		    context->talker = NMEA_TALKER_MASK_GPS;
		}
		else if (data[1] == 'L')
		{
		    context->talker = NMEA_TALKER_MASK_GLONASS;
		}
		else if (data[1] == 'N')
		{
		    context->talker = NMEA_TALKER_MASK_GNSS;
		}
	    }

	    /* --GSA is the switch detector in NMEA 0183. If it's GPGSA or GLGSA, then the system
	     * is set up as single GPS/GLONASS system, and we'd see only either a GPGSV or GLGSV
	     * later on. If it's a GNGSA, then another GNGSA will follow, one for GPS and one for 
	     * GLONASS. The constellation will be reported as GPGSV and GLGSV.
	     */
	    if ((context->talker & (NMEA_TALKER_MASK_GPS | NMEA_TALKER_MASK_GLONASS | NMEA_TALKER_MASK_GNSS)) && !strcmp((const char*)(data+2), "GSA"))
	    {
		if (device->seen & NMEA_SENTENCE_MASK_GPGGA)
		{
		    sequence_next = NMEA_FIELD_SEQUENCE_GSA_OPERATION;
		    
		    context->mask = (NMEA_FIELD_MASK_VDOP);
		}
	    }

#if (GPS_CONFIG_SATELLITES >= 1)
	    /* -- GSV is used to report the satellite constellation with either GPGSV or GLGSV.
	     * GNGSV is not legal.
	     */
	    else if ((context->talker & (NMEA_TALKER_MASK_GPS | NMEA_TALKER_MASK_GLONASS)) && !strcmp((const char*)(data+2), "GSV"))
	    {
		if (device->seen & (NMEA_SENTENCE_MASK_GPGGA | NMEA_SENTENCE_MASK_SOLUTION))
		{
		    sequence_next = NMEA_FIELD_SEQUENCE_GSV_SENTENCES;
		}
	    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */

	    /* According to the standard, if a receiver is supporting GPS only, the talker would 
	     * be "GP". If it's a GLONASS only system, or a combined GPS+GLONASS system, then
	     * the talker should be "GN". However some GLONASS only systems use a "GL" talker,
	     * and quite a few GPS_GLONASS systems mix "GP" and "GN" randomly. So just make
	     * sure that a valid talker is used, and simply ignore it. The system detection
	     * is done via --GSA anyway.
	     */
	    else if (context->talker)
	    {
		if (!strcmp((const char*)(data+2), "GGA"))
		{
		    sequence_next = NMEA_FIELD_SEQUENCE_GGA_TIME;

		    /* GSA/GSV are subsequent to a GGA */
		    device->seen &= ~(NMEA_SENTENCE_MASK_GPGGA |
				      NMEA_SENTENCE_MASK_GPGSA |
				      NMEA_SENTENCE_MASK_GPGSV |
				      NMEA_SENTENCE_MASK_GLGSA |
				      NMEA_SENTENCE_MASK_GLGSV |
				      NMEA_SENTENCE_MASK_SOLUTION); 
			
		    context->mask = (NMEA_FIELD_MASK_POSITION |
				     NMEA_FIELD_MASK_ALTITUDE |
				     NMEA_FIELD_MASK_HDOP);

#if (GPS_CONFIG_SATELLITES >= 1)			
		    context->sv_in_view_sentences = 0;
			
		    context->sv_used_count = 0;
		    context->sv_used_mask[0] = 0;
		    context->sv_used_mask[1] = 0;
		    context->sv_used_mask[2] = 0;
			
		    device->satellites.count = 0;
#endif /* GPS_CONFIG_SATELLITES >= 1 */			
		}

		else if (!strcmp((const char*)(data+2), "RMC"))
		{
		    sequence_next = NMEA_FIELD_SEQUENCE_RMC_TIME;
			
		    device->seen &= ~(NMEA_SENTENCE_MASK_GPRMC | NMEA_SENTENCE_MASK_SOLUTION);
			
		    context->mask = (NMEA_FIELD_MASK_TIME |
				     NMEA_FIELD_MASK_SPEED |
				     NMEA_FIELD_MASK_COURSE);
		}
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_SKIP:
	sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	break;

    case NMEA_FIELD_SEQUENCE_GGA_TIME:
    case NMEA_FIELD_SEQUENCE_RMC_TIME:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_TIME;
	}
	else if (nmea_parse_time(data, &time))
	{
	    /* If there is a valid time stamp, and another sentence with a time stamp already been seen,
	     * make sure they have the same time. If not nuke the accumulated sentences.
	     */

	    if (device->seen & (NMEA_SENTENCE_MASK_GPGGA | NMEA_SENTENCE_MASK_GPRMC))
	    {
		if (!nmea_same_time(&device->location.time, &time))
		{
		    device->seen = 0;
		    device->location.type = 0;
		    device->location.mask = 0;
		}
	    }

	    device->location.time.hour = time.hour;
	    device->location.time.min  = time.min;
	    device->location.time.sec  = time.sec;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_LATITUDE:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_POSITION;
	}
	else if (nmea_parse_latitude(data, &latitude))
	{
	    device->location.latitude = latitude;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_LATITUDE_NS:
	if (context->mask & NMEA_FIELD_MASK_POSITION)
	{
	    if (data[0] == 'S')
	    {
		device->location.latitude = - device->location.latitude;
	    }
	    else if (data[0] != 'N')
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_LONGITUDE:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_POSITION;
	}
	else if (nmea_parse_longitude(data, &longitude))
	{
	    device->location.longitude = longitude;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_LONGITUDE_EW:
	if (context->mask & NMEA_FIELD_MASK_POSITION)
	{
	    if (data[0] == 'W')
	    {
		device->location.longitude = - device->location.longitude;
	    }
	    else if (data[0] != 'E')
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_QUALITY:
	if ((data[0] == '\0') || !nmea_parse_unsigned(data, &quality))
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    device->location.quality = quality;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_NUMSV:
	if ((data[0] == '\0') || !(nmea_parse_unsigned(data, &numsv)))
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    device->location.numsv = numsv;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_HDOP:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_HDOP;
	}
	else if (nmea_parse_fixed(data, &hdop, 2))
	{
	    device->location.hdop = hdop;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_ALTITUDE:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_ALTITUDE;
	}
	else if (nmea_parse_fixed(((data[0] == '-') ? data+1 : data), &altitude, 3))
	{
	    if (data[0] == '-')
	    {
		device->location.altitude = -altitude;
	    }
	    else
	    {
		device->location.altitude = altitude;
	    }
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_ALTITUDE_UNIT:
	if (context->mask & NMEA_FIELD_MASK_ALTITUDE)
	{
	    if (data[0] != 'M')
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_SEPARATION:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_ALTITUDE;
	}
	else if (nmea_parse_fixed(((data[0] == '-') ? data+1 : data), &separation, 3))
	{
	    if (context->mask & NMEA_FIELD_MASK_ALTITUDE)
	    {
		if (data[0] == '-')
		{
		    device->location.altitude -= separation;
		}
		else
		{
		    device->location.altitude += separation;
		}
	    }
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_SEPARATION_UNIT:
	if (context->mask & NMEA_FIELD_MASK_ALTITUDE)
	{
	    if (data[0] != 'M')
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_GGA_DIFFERENTIAL_AGE:
    case NMEA_FIELD_SEQUENCE_GSA_OPERATION:
    case NMEA_FIELD_SEQUENCE_GSA_PDOP:
    case NMEA_FIELD_SEQUENCE_GSA_HDOP:
    case NMEA_FIELD_SEQUENCE_RMC_LATITUDE:
    case NMEA_FIELD_SEQUENCE_RMC_LATITUDE_NS:
    case NMEA_FIELD_SEQUENCE_RMC_LONGITUDE:
    case NMEA_FIELD_SEQUENCE_RMC_LONGITUDE_EW:
    case NMEA_FIELD_SEQUENCE_RMC_VARIATION:
    case NMEA_FIELD_SEQUENCE_RMC_VARIATION_UNIT:
	/* SKIP FIELD */
	break;

    case NMEA_FIELD_SEQUENCE_GGA_DIFFERENTIAL_STATION:
	/* SKIP FIELD */
	sequence_next = NMEA_FIELD_SEQUENCE_GGA_END;
	break;

    case NMEA_FIELD_SEQUENCE_GSA_NAVIGATION:
	if (data[0] == '1')
	{
	    context->navigation = NMEA_NAVIGATION_NONE;
	}
	else if (data[0] == '2')
	{
	    context->navigation = NMEA_NAVIGATION_2D;
	}
	else if (data[0] == '3')
	{
	    context->navigation = NMEA_NAVIGATION_3D;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_1:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_2:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_3:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_4:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_5:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_6:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_7:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_8:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_9:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_10:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_11:
    case NMEA_FIELD_SEQUENCE_GSA_SV_USED_PRN_12:
#if (GPS_CONFIG_SATELLITES >= 1)
	if (data[0] != '\0')
	{
	    if (nmea_parse_unsigned(data, &svid))
	    {
		context->sv_used_count++;

		if ((svid >= 1) && (svid <= 96))
		{
		    context->sv_used_mask[(svid -1) >> 5] |= (1ul << ((svid -1) & 31));
		}
	    }
	    else
	    {
		context->sv_used_count = 0;
		context->sv_used_mask[0] = 0;
		context->sv_used_mask[1] = 0;
		context->sv_used_mask[2] = 0;

		sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	    }
	}
#endif /* GPS_CONFIG_SATELLITES >= 1 */
	break;

    case NMEA_FIELD_SEQUENCE_GSA_VDOP:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_VDOP;

	    sequence_next = NMEA_FIELD_SEQUENCE_GSA_END;
	}
	else if (nmea_parse_fixed(data, &vdop, 2))
	{
	    device->location.vdop = vdop;
	    
	    sequence_next = NMEA_FIELD_SEQUENCE_GSA_END;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

#if (GPS_CONFIG_SATELLITES >= 1)
    case NMEA_FIELD_SEQUENCE_GSV_SENTENCES:
	if ((data[0] == '\0') || !nmea_parse_unsigned(data, &sentences))
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    if (context->sv_in_view_sentences == 0)
	    {
		context->sv_in_view_sentences = sentences;
		context->sv_in_view_count = 0;
		context->sv_in_view_index = 0;
	    }
	    else
	    {
		if (context->sv_in_view_sentences != sentences)
		{
		    context->sv_in_view_sentences = 0;

		    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
		}
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSV_CURRENT:
	if ((data[0] == '\0') || !nmea_parse_unsigned(data, &current))
	{
	    context->sv_in_view_sentences = 0;

	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    if (context->sv_in_view_index != ((current -1) << 2))
	    {
		context->sv_in_view_sentences = 0;

		sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_COUNT:
	if ((data[0] == '\0') || !nmea_parse_unsigned(data, &count))
	{
	    context->sv_in_view_sentences = 0;

	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    context->sv_in_view_count = count;

	    if (count == 0)
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_GSV_END;
	    }
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_ID:
	svid = 255;

	if ((data[0] == '\0') || nmea_parse_unsigned(data, &svid))
	{
	    if (device->satellites.count < GPS_SATELLITES_COUNT_MAX)
	    {
		device->satellites.info[device->satellites.count].id = svid;
		device->satellites.info[device->satellites.count].elev = 0;
		device->satellites.info[device->satellites.count].azim = 0;
		device->satellites.info[device->satellites.count].snr = 0;
		device->satellites.info[device->satellites.count].state = 0;
	    }
	}
	else
	{
	    context->sv_in_view_sentences = 0;

	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_ELEV:
	elev = 0;

	if ((data[0] == '\0') || nmea_parse_unsigned(data, &elev))
	{
	    if (device->satellites.count < GPS_SATELLITES_COUNT_MAX)
	    {
		device->satellites.info[device->satellites.count].elev = elev;
	    }
	}
	else
	{
	    context->sv_in_view_sentences = 0;

	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_AZIM:
	azim = 0;

	if ((data[0] == '\0') || nmea_parse_unsigned(data, &azim))
	{
	    if (device->satellites.count < GPS_SATELLITES_COUNT_MAX)
	    {
		device->satellites.info[device->satellites.count].azim = azim;
	    }
	}
	else
	{
	    context->sv_in_view_sentences = 0;

	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_SNR:
	if ((data[0] == '\0') || nmea_parse_unsigned(data, &snr))
	{
	    if (device->satellites.count < GPS_SATELLITES_COUNT_MAX)
	    {
		if (data[0] != '\0')
		{
		    device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_TRACKING;
		    device->satellites.info[device->satellites.count].snr = snr;
		}
	    }

	    device->satellites.count++;

	    context->sv_in_view_index++;

	    if ((context->sv_in_view_index == context->sv_in_view_count) || !(context->sv_in_view_index & 3))
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_GSV_END;
	    }
	    else
	    {
		sequence_next = NMEA_FIELD_SEQUENCE_GSV_SV_IN_VIEW_ID;
	    }
	}
	else
	{
	    context->sv_in_view_sentences = 0;

	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;
#endif /* GPS_CONFIG_SATELLITES >= 1 */

    case NMEA_FIELD_SEQUENCE_RMC_STATUS:
	if (data[0] == 'A')
	{
	    context->status = NMEA_STATUS_DATA_VALID;
	}
	else if (data[0] == 'V')
	{
	    context->status = NMEA_STATUS_RECEIVER_WARNING;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_RMC_SPEED:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_SPEED;
	}
	else if (nmea_parse_fixed(data, &speed, 3))
	{
	    /* Conversion factor from knots to m/s is 1852 / 3600.
	     */
	    device->location.speed  = (speed * 1852 +  1800) / 3600;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_RMC_COURSE:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_COURSE;
	}
	else if (nmea_parse_fixed(data, &course, 5))
	{
	    device->location.course = course;
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_RMC_DATE:
	if (data[0] == '\0')
	{
	    context->mask &= ~NMEA_FIELD_MASK_TIME;
	}
	else if (nmea_parse_unsigned(data, &date))
	{
	    device->location.time.day   = date / 10000;
	    device->location.time.month = (date - device->location.time.day * 10000) / 100;
	    device->location.time.year  = (date - device->location.time.day * 10000 - device->location.time.month * 100) + 1900;
	    
	    if (device->location.time.year < 1980)
	    {
		device->location.time.year += 100;
	    }
	}
	else
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	break;

    case NMEA_FIELD_SEQUENCE_RMC_MODE:
	/* SKIP FIELD */
	sequence_next = NMEA_FIELD_SEQUENCE_RMC_END;
	break;

#if (GPS_CONFIG_MEDIATEK >= 1)
    case NMEA_FIELD_SEQUENCE_PMTK001_COMMAND:
	if ((data[0] == '\0') || !nmea_parse_unsigned(data, &command))
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    context->mtk_command = command;
	}
	break;

    case NMEA_FIELD_SEQUENCE_PMTK001_STATUS:
	if ((data[0] == '\0') || !nmea_parse_unsigned(data, &status))
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    context->mtk_status = status;

	    sequence_next = NMEA_FIELD_SEQUENCE_PMTK001_END;
	}
	break;

    case NMEA_FIELD_SEQUENCE_PMTK010_STATUS:
	if ((data[0] == '\0') || !nmea_parse_unsigned(data, &status))
	{
	    sequence_next = NMEA_FIELD_SEQUENCE_SKIP;
	}
	else
	{
	    context->mtk_status = status;

	    sequence_next = NMEA_FIELD_SEQUENCE_PMTK010_END;
	}
	break;

    case NMEA_FIELD_SEQUENCE_PMTK869_MODE:
	/* SKIP */
	break;

    case NMEA_FIELD_SEQUENCE_PMTK869_STATUS:
	/* SKIP */
	sequence_next = NMEA_FIELD_SEQUENCE_PMTK869_END;
	break;
#endif /* GPS_CONFIG_MEDIATEK >= 1 */
    }

    context->sequence = sequence_next;
}

static void nmea_end_sentence(gps_device_t *device)
{
    nmea_context_t *context = &device->nmea;
    uint32_t expected;
#if (GPS_CONFIG_SATELLITES >= 1)
    uint32_t n, svid;
#endif /* GPS_CONFIG_SATELLITES >= 1 */

    switch (context->sequence) {

    case NMEA_FIELD_SEQUENCE_GGA_END:
      if (context->mask & NMEA_FIELD_MASK_POSITION)
	{
	    device->location.mask |= GPS_LOCATION_MASK_POSITION;
	}

	if (context->mask & NMEA_FIELD_MASK_ALTITUDE)
	{
	    device->location.mask |= GPS_LOCATION_MASK_ALTITUDE;
	}

	if (context->mask & NMEA_FIELD_MASK_HDOP)
	{
	    device->location.mask |= GPS_LOCATION_MASK_HDOP;
	}
	
	device->seen |= NMEA_SENTENCE_MASK_GPGGA;
	device->seen &= ~NMEA_SENTENCE_MASK_SOLUTION;
	break;

    case NMEA_FIELD_SEQUENCE_GSA_END:
	if (context->mask & NMEA_FIELD_MASK_VDOP)
	{
	    device->location.mask |= GPS_LOCATION_MASK_VDOP;
	}

	/* If the talkes is "GN", then it's a composite fix, which will consist out
	 * of GNGSA, GNGSA, GPGSV & GLGSV. Otherwise only a GPGSA & GPGSV is to be
	 * expected.
	 */

	if (context->talker & NMEA_TALKER_MASK_GNSS)
	{
	    device->expected |= (NMEA_SENTENCE_MASK_GPGSA | NMEA_SENTENCE_MASK_GPGSV | NMEA_SENTENCE_MASK_GLGSA | NMEA_SENTENCE_MASK_GLGSV);
	    
	    if (!(device->seen & NMEA_SENTENCE_MASK_GPGSA))
	    {
		device->seen |= NMEA_SENTENCE_MASK_GPGSA;
	    }
	    else
	    {
		device->seen |= NMEA_SENTENCE_MASK_GLGSA;
		device->seen &= ~NMEA_SENTENCE_MASK_SOLUTION;
	    }
	}
	else if (context->talker & NMEA_TALKER_MASK_GLONASS)
	{
	    device->expected = (device->expected & ~(NMEA_SENTENCE_MASK_GPGSA | NMEA_SENTENCE_MASK_GPGSV)) | (NMEA_SENTENCE_MASK_GLGSA | NMEA_SENTENCE_MASK_GLGSV);

	    device->seen |= NMEA_SENTENCE_MASK_GLGSA;
	    device->seen &= ~NMEA_SENTENCE_MASK_SOLUTION;
	}
	else
	{
	    device->expected = (device->expected & ~(NMEA_SENTENCE_MASK_GLGSA | NMEA_SENTENCE_MASK_GLGSV)) | (NMEA_SENTENCE_MASK_GPGSA | NMEA_SENTENCE_MASK_GPGSV);

	    device->seen |= NMEA_SENTENCE_MASK_GPGSA;
	    device->seen &= ~NMEA_SENTENCE_MASK_SOLUTION;
	}
	break;

    case NMEA_FIELD_SEQUENCE_GSV_END:
#if (GPS_CONFIG_SATELLITES >= 1)
	if (context->sv_in_view_count == context->sv_in_view_index)
	{
	    context->sv_in_view_sentences = 0;

	    if (context->talker & NMEA_TALKER_MASK_GPS)
	    {
		device->seen |= NMEA_SENTENCE_MASK_GPGSV;
	    }
	    else
	    {
		device->seen |= NMEA_SENTENCE_MASK_GLGSV;
	    }
	}
#endif /* GPS_CONFIG_SATELLITES >= 1 */
	break;

    case NMEA_FIELD_SEQUENCE_RMC_END:
	if (context->mask & NMEA_FIELD_MASK_TIME)
	{
	    device->location.mask |= GPS_LOCATION_MASK_TIME;
	}

	if (context->mask & NMEA_FIELD_MASK_SPEED)
	{
	    device->location.mask |= GPS_LOCATION_MASK_SPEED;
	}

	if (context->mask & NMEA_FIELD_MASK_COURSE)
	{
	    device->location.mask |= GPS_LOCATION_MASK_COURSE;
	}

	device->seen |= NMEA_SENTENCE_MASK_GPRMC;
	device->seen &= ~NMEA_SENTENCE_MASK_SOLUTION;
	break;

#if (GPS_CONFIG_MEDIATEK >= 1)
    case NMEA_FIELD_SEQUENCE_PMTK001_END:
	if (device->command == context->mtk_command)
	{
	    device->command = ~0l;

	    mtk_initialize(device, (context->mtk_status == 3) ? GPS_RESPONSE_ACK : GPS_RESPONSE_NACK, context->mtk_command);
	}
	break;

    case NMEA_FIELD_SEQUENCE_PMTK010_END:
	if (context->mtk_status == 1)
	{
	    mtk_initialize(device, GPS_RESPONSE_STARTUP, 10);
	}
	break;

    case NMEA_FIELD_SEQUENCE_PMTK869_END:
	if (device->command == 869)
	{
	    device->command = ~0l;

	    mtk_initialize(device, GPS_RESPONSE_ACK, 869);
	}
	break;
#endif /* GPS_CONFIG_MEDIATEK >= 1 */

    default:
	break;
    }

    context->sequence = NMEA_FIELD_SEQUENCE_START;

    if (device->init == GPS_INIT_DONE)
    {
	expected = device->expected & (NMEA_SENTENCE_MASK_GPGGA |
				       NMEA_SENTENCE_MASK_GPGSA |
				       NMEA_SENTENCE_MASK_GPRMC |
				       NMEA_SENTENCE_MASK_GLGSA);
	
	if ((device->seen & expected) == expected)
	{
	    if ((context->status == NMEA_STATUS_DATA_VALID) && (context->navigation != NMEA_NAVIGATION_NONE))
	    {
		device->location.type = (context->navigation == NMEA_NAVIGATION_2D) ? GPS_LOCATION_TYPE_2D : GPS_LOCATION_TYPE_3D;
	    }
	    else
	    {
		device->location.type = GPS_LOCATION_TYPE_NONE;

#if (GPS_CONFIG_SATELLITES >= 1)
		context->sv_used_mask[0] = 0;
		context->sv_used_mask[1] = 0;
		context->sv_used_mask[2] = 0;
#endif /* GPS_CONFIG_SATELLITES >= 1 */
	    }

	    gps_location(device);
	    
	    device->seen &= ~(NMEA_SENTENCE_MASK_GPGGA |
			      NMEA_SENTENCE_MASK_GPGSA |
			      NMEA_SENTENCE_MASK_GPRMC |
			      NMEA_SENTENCE_MASK_GLGSA);

	    device->seen |= NMEA_SENTENCE_MASK_SOLUTION;
	}
	
	
#if (GPS_CONFIG_SATELLITES >= 1)
	expected = device->expected & (NMEA_SENTENCE_MASK_GPGSV | NMEA_SENTENCE_MASK_GLGSV);
	
	if ((device->seen & NMEA_SENTENCE_MASK_SOLUTION) && ((device->seen & expected) == expected))
	{
	    for (n = 0; n < device->satellites.count; n++)
	    {
		svid = device->satellites.info[n].id;
		
		if ((svid >= 1) && (svid <= 96) && (context->sv_used_mask[(svid -1) >> 5] & (1ul << ((svid -1) & 31))))
		{
		    device->satellites.info[n].state |= GPS_SATELLITES_STATE_NAVIGATING;
		}
	    }

	    gps_satellites(device);
	    
	    device->seen &= ~(NMEA_SENTENCE_MASK_GPGSV | NMEA_SENTENCE_MASK_GLGSV);
	}
#endif /* GPS_CONFIG_SATELLITES >= 1 */
    }
}

#if (GPS_CONFIG_MEDIATEK >= 1)

static void mtk_send(gps_device_t *device, const char *data)
{
    uint32_t command;

    command = ((data[5] - '0') * 10 + (data[6] - '0')) * 10 + (data[7] - '0');

    gps_send(&gps_device, command, (const uint8_t*)data, strlen(data));
}


static const char *mtk_init_table[] = {
#if (GPS_CONFIG_MEDIATEK == 1)
    "$PMTK397,0*23\r\n",               /* NAV THRESHOLD */
#else /*GPS_CONFIG_MEDIATEK == 1 */
    "$PMTK286,1*23\r\n",               /* AIC           */
    "$PMTK386,0*23\r\n",               /* NAV THRESHOLD */
    "$PMTK351,0*29\r\n",               /* QZSS NMEA     */
#if (GPS_CONFIG_QZSS == 1)
    "$PMTK352,0*2A\r\n",               /* QZSS STOP     */
#else /* GPS_CONFIG_QZSS == 1 */
    "$PMTK352,1*2B\r\n",               /* QZSS STOP     */
#endif /* GPS_CONFIG_QZSS == 1 */
#if (GPS_CONFIG_MEDIATEK == 3)
#if (GPS_CONFIG_GLONASS == 1)
    "$PMTK353,1,1*37\r\n",             /* GLONASS       */
#else /* GPS_CONFIG_GLONASS == 1 */
    "$PMTK353,1,0*36\r\n",             /* GLONASS       */
#endif /* GPS_CONFIG_GLONASS == 1 */
#endif /* GPS_CONFIG_MEDIATEK == 3 */
    "$PMTK185,1*23\r\n",               /* LOCUS         */
    "$PMTK869,0,0*35\r\n",             /* EASY          */
#endif /*GPS_CONFIG_MEDIATEK == 1 */
#if (GPS_CONFIG_SBAS >= 1)
    "$PMTK301,2*2E\r\n",               /* DGPS MODE     */
    "$PMTK313,1*2E\r\n",               /* SBAS ENABLED  */
#else /* GPS_CONFIG_SBAS >= 1 */
    "$PMTK301,0*2C\r\n",               /* DGPS MODE     */
    "$PMTK313,0*2F\r\n",               /* SBAS ENABLED  */
#endif /* GPS_CONFIG_SBAS >= 1 */
#if (GPS_CONFIG_RATE >= 5)
    "$PMTK220,200*2C\r\n",             /* POS FIX       */
    "$PMTK300,200,0,0,0,0*2F\r\n",     /* FIX CTL       */
#else
    "$PMTK220,1000*1F\r\n",            /* POS FIX       */
    "$PMTK300,1000,0,0,0,0*1C\r\n",    /* FIX CTL       */
#endif
#if (GPS_CONFIG_SATELLITES == 2)
    "$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n",
#elif (GPS_CONFIG_SATELLITES == 1)
#if (GPS_CONFIG_RATE >= 5)
    "$PMTK314,0,1,0,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n",
#else
    "$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n",
#endif
#else
    "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n",
#endif
    NULL,
};

static void mtk_initialize(gps_device_t *device, unsigned int response, uint32_t command)
{
    if (response == GPS_RESPONSE_NMEA_SENTENCE)
    {
	if (device->reset == GPS_RESET_COLD_START)
	{
	    mtk_send(device, "$PMTK103*30\r\n");

	    device->init = GPS_INIT_MTK_STARTUP;
	}
	else if (device->reset == GPS_RESET_WARM_START)
	{
	    mtk_send(device, "$PMTK102*31\r\n");

	    device->init = GPS_INIT_MTK_STARTUP;
	}
	else if (device->reset == GPS_RESET_HOT_START)
	{
	    mtk_send(device, "$PMTK101*32\r\n");

	    device->init = GPS_INIT_MTK_STARTUP;
	}
	else
	{
	    mtk_send(device, mtk_init_table[0]);

	    device->init = GPS_INIT_MTK_INIT_TABLE;
	}
    }
    else
    {
	if (device->init == GPS_INIT_MTK_STARTUP)
	{
	    if (response == GPS_RESPONSE_STARTUP)
	    {
		mtk_send(device, mtk_init_table[0]);
	      
		device->init = GPS_INIT_MTK_INIT_TABLE;
	    }
	    else
	    {
		if (response == GPS_RESPONSE_ACK)
		{
		    device->init = GPS_INIT_MTK_STARTUP;
		}
		else
		{
		    device->init = GPS_INIT_MTK_BAUD_RATE;
		}
	    }
	}
	else
	{
	    if (response == GPS_RESPONSE_NACK)
	    {
		if (!((command == 185) || /* LOCUS DISABLE */
		      (command == 397) || /* NAV THRESHOLD */
		      (command == 869)))  /* EASY DISABLE  */
		{
		    device->init = GPS_INIT_MTK_BAUD_RATE;
		}
	    }

	    if (device->init != GPS_INIT_MTK_BAUD_RATE)
	    {
		device->init++;

		if (mtk_init_table[device->init - GPS_INIT_MTK_INIT_TABLE])
		{
		    mtk_send(device, mtk_init_table[device->init - GPS_INIT_MTK_INIT_TABLE]);
		}
		else
		{
#if (GPS_CONFIG_SATELLITES >= 1)
		    if (device->satellites_callback == NULL)
		    {
			mtk_send(device, "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
		    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */

		    device->init = GPS_INIT_DONE;
		    device->seen = 0;
		    device->expected = NMEA_SENTENCE_MASK_GPGGA | NMEA_SENTENCE_MASK_GPGSA | NMEA_SENTENCE_MASK_GPGSV | NMEA_SENTENCE_MASK_GPRMC;

		    device->location.type = 0;
		    device->location.mask = 0;

		    (*device->ini_callback)(1);
		}
	    }
	}
    }
}

#endif /* GPS_CONFIG_MEDIATEK >= 1 */

#else /* GPS_CONFIG_SIRF == 0 && GPS_CONFIG_UBLOX == 0 */

static void nmea_start_sentence(gps_device_t *device)
{
}

static void nmea_parse_sentence(gps_device_t *device, uint8_t *data, unsigned int length)
{
}

static void nmea_end_sentence(gps_device_t *device)
{
}

#endif /* GPS_CONFIG_SIRF == 0 && GPS_CONFIG_UBLOX == 0 */

/************************************************************************************/

#if (GPS_CONFIG_SIRF >= 1)

static inline int8_t srf_data_int8(const uint8_t *data, unsigned int offset)
{
    return (int8_t)data[offset];
}

static inline int16_t srf_data_int16(const uint8_t *data, unsigned int offset)
{
    return (int16_t)(((uint16_t)data[offset+0] << 8) |
		     ((uint16_t)data[offset+1] << 0));
}

static inline int32_t srf_data_int32(const uint8_t *data, unsigned int offset)
{
    return (int32_t)(((uint32_t)data[offset+0] << 24) |
		     ((uint32_t)data[offset+1] << 16) |
		     ((uint32_t)data[offset+2] <<  8) |
		     ((uint32_t)data[offset+3] <<  0));
}

static inline uint8_t srf_data_uint8(const uint8_t *data, unsigned int offset)
{
    return (uint8_t)data[offset];
}

static inline uint16_t srf_data_uint16(const uint8_t *data, unsigned int offset)
{
    return (uint16_t)(((uint16_t)data[offset+0] << 8) |
		      ((uint16_t)data[offset+1] << 0));
}

static inline uint32_t srf_data_uint32(const uint8_t *data, unsigned int offset)
{
    return (uint32_t)(((uint32_t)data[offset+0] << 24) |
		      ((uint32_t)data[offset+1] << 16) |
		      ((uint32_t)data[offset+2] <<  8) |
		      ((uint32_t)data[offset+3] <<  0));
}

static inline int srf_compare_tow(unsigned int tow1, unsigned int tow2)
{
    int diff = 0;

    if (tow1 <= tow2)
    {
	if ((tow2 - tow1) > 20)
	{
	    diff = (tow1 - tow2);
	}
    }
    else
    {
	if ((tow1 - tow2) > 20)
	{
	    diff = (tow1 - tow2);
	}
    }

    return diff;
}

static void srf_start_message(gps_device_t *device, unsigned int message, unsigned int length)
{
#if (GPS_CONFIG_SATELLITES >= 1)
    if (message == 0x04)
    {
	/* SIRF_MSG_SSB_MEASURED_TRACKER */

	device->last = 23;
	device->satellites.count = 0;

	device->seen &= ~SRF_MESSAGE_MASK_MEASURED_TRACKER;
    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */
}

static void srf_parse_message(gps_device_t *device, unsigned int message, const uint8_t *data, unsigned int count)
{
#if (GPS_CONFIG_SATELLITES >= 1)
    srf_context_t *context = &device->srf;
    uint32_t tow, svid, svstate;

    if (message == 0x04)
    {
	/* SIRF_MSG_SSB_MEASURED_TRACKER */

	/* Bits for State:
	 *
	 *  0 acquisition complete
	 *  1 integrated carrier phase valid
	 *  2 bit synchronization complete
	 *  3 subframe synchronization complete
	 *  4 carrier pulling complete, costas lock (carrier lock ?)
	 *  5 code has been locked
	 *  6 (acquisition failed)
	 *  7 ephermis data avialable
	 *
	 * Bits 8-15 are reserved.
	 *
	 * A Channel is fully locked if the value is 0xbf.
	 * 
	 * The "used" bit is in 0x29, sv_used_list.
	 *
	 * The data is converted chunk wise. The svinfo starts at offset 8.
	 */

	svid = srf_data_uint8(data, 8);

	if (svid && (device->satellites.count < GPS_SATELLITES_COUNT_MAX))
	{
	    if ((svid >= 120) && (svid <= 151))
	    {
		svid = svid - 87;
	    }

	    device->satellites.info[device->satellites.count].id = svid;
	    device->satellites.info[device->satellites.count].elev = srf_data_uint8(data, 10) / 2;
	    device->satellites.info[device->satellites.count].azim = (srf_data_uint8(data, 9) * 3) / 2;
	    device->satellites.info[device->satellites.count].snr = srf_data_uint8(data, 13);
	    device->satellites.info[device->satellites.count].state = 0;
	    
	    if ((device->satellites.info[device->satellites.count].elev != 0) &&
		(device->satellites.info[device->satellites.count].azim != 0) &&
		(device->satellites.info[device->satellites.count].snr  != 0))
	    {
		device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_TRACKING;
	    }

	    svstate = srf_data_uint16(data, 11);
	    
	    if (svstate & 0x01)
	    {
		device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_CARRIER_LOCK;
	    }
	    
	    if (svstate & 0x20)
	    {
		device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_CODE_LOCK;
	    }
	    
	    if (svstate & 0x80)
	    {
		device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_EPHEMERIS;
	    }
		
	    device->satellites.count++;
	}

	device->first += 15;
	device->last += 15;
    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */
}

static void srf_end_message(gps_device_t *device, unsigned int message, const uint8_t *data, unsigned int count)
{
    srf_context_t *context = &device->srf;
    uint32_t tow, svid, svstate, expected;
    uint8_t command;
    int offset, limit, n;

    switch (message) {

#if (GPS_CONFIG_SATELLITES >= 1)
    case 0x04:
	/* SIRF_MSG_SSB_MEASURED_TRACKER */

	tow = srf_data_uint32(data, 3) * 10;
	
	if (device->seen & (SRF_MESSAGE_MASK_MEASURED_TRACKER |
			    SRF_MESSAGE_MASK_CLOCK_STATUS |
			    SRF_MESSAGE_MASK_GEODETIC_NAVIGATION |
			    SRF_MESSAGE_MASK_SOLUTION))
	{
	    if (srf_compare_tow(context->tow, tow))
	    {
		device->seen &= ~(SRF_MESSAGE_MASK_MEASURED_TRACKER |
				  SRF_MESSAGE_MASK_CLOCK_STATUS |
				  SRF_MESSAGE_MASK_GEODETIC_NAVIGATION |
				  SRF_MESSAGE_MASK_SOLUTION);
		
		device->location.mask = 0;
	    }
	}

	device->seen |= SRF_MESSAGE_MASK_MEASURED_TRACKER;
	break;
#endif /* (GPS_CONFIG_SATELLITES >= 1) */

    case 0x07:
	/* SRF_MSG_SSB_CLOCK_STATUS */

	tow = srf_data_uint32(data, 3) * 10;

	if (device->seen & (SRF_MESSAGE_MASK_MEASURED_TRACKER |
			    SRF_MESSAGE_MASK_CLOCK_STATUS |
			    SRF_MESSAGE_MASK_GEODETIC_NAVIGATION |
			    SRF_MESSAGE_MASK_SOLUTION))
	{
	    if (srf_compare_tow(context->tow, tow))
	    {
		device->seen &= ~(SRF_MESSAGE_MASK_MEASURED_TRACKER |
				  SRF_MESSAGE_MASK_CLOCK_STATUS |
				  SRF_MESSAGE_MASK_GEODETIC_NAVIGATION |
				  SRF_MESSAGE_MASK_SOLUTION);

		device->location.mask = 0;
	    }
	}

	/* Only update TOW if there hasn't been any more precise value before.
	 */
	if (!(device->seen & SRF_MESSAGE_MASK_GEODETIC_NAVIGATION))
	{
	    context->tow = tow;
	}

	device->location.bias = srf_data_uint32(data, 12);

	/* Hz to ns factor is 1000 / 1575.52 == 0.63475.
	 */
	device->location.drift = (srf_data_uint32(data, 8) * 20799) / 32768;

	device->location.mask |= GPS_LOCATION_MASK_CLOCK;

	device->seen |= SRF_MESSAGE_MASK_CLOCK_STATUS;
	device->seen &= ~SRF_MESSAGE_MASK_SOLUTION;
	break;

    case 0x0b:
	/* SRF_MSG_SSB_ACK */

	command = srf_data_uint8(data, 1);

	if (command == device->command)
	{
	    device->command = ~0l;

	    srf_initialize(device, GPS_RESPONSE_ACK, command);
	}
	break;

    case 0x0c:
	/* SRF_MSG_SSB_ACK */

	command = srf_data_uint8(data, 1);

	if (command == device->command)
	{
	    device->command = ~0l;

	    srf_initialize(device, GPS_RESPONSE_NACK, command);
	}
	break;

    case 0x12:
	/* SRF_MSG_SSB_OK_TO_SEND */
	if (srf_data_uint8(data, 1) == 1)
	{
	    if (device->init == GPS_INIT_SRF_STARTUP) 
	    {
		srf_initialize(device, GPS_RESPONSE_STARTUP, ~0l);
	    }
	}
	break;

    case 0x29:
	/* SIRF_MSG_SSB_GEODETIC_NAVIGATION */

	/*
	 * NavValid (16bit @ offset 1):
	 *
	 *  0    Solution not yet overdetermined (< 5 SV)
	 *  1    -
	 *  2    Using default week number if NavType is "NoFix" (if 0 means week/tow is valid when NavType is "NoFix") (GSD4e)
	 *  3    (SiRFDRive)
	 *  4    (SiRFDRive)
	 *  5    (SiRFDRive)
	 *  6    (SiRFDRive)
	 *  7    (SiRFDRive)
	 *  8    Almanac Based Position (GSD4e)
	 *  9    -
	 * 10    -
	 * 11    Position can only be derived by reverse EE (GSD4e)
	 * 12    -
	 * 13    -
	 * 14    -
	 * 15    (SiRFNav)
	 *
	 *
	 * NavType (16bit @ offset 3):
	 *
	 *  0-2
	 *    0  NoFix
	 *    1  1 SV KF solution
	 *    2  2 SV KF solution
	 *    3  3 SV KF solution
	 *    4  4 or move SV KF solution
	 *    5  2D least-squares solution
	 *    6  3D least-squares solution
	 *    7  DR solution
	 *  3    Trickle Power in use
	 *  4-5
         *    0  No attitude hold applied
         *    1  Holding of altitude from KF
         *    2  Holding of altitude from user input
         *    3  Always hold altitude (from user input)
	 *  6    DOP limits exceeded
	 *  7    DGPS corrections applied
	 *  8    (SIRFDRive)
	 *  9    Navidation solution overdetermined
	 * 10    Velocity DR timeout exceeded
	 * 11    -
	 * 12    Invalid Velocity
	 * 13    Altitude hold disabled
	 * 14    (SIRFDRive)
	 * 15    (SIRFDRive)
	 *
	 *
	 * AdditionalModeInfo (8bit @ offset 90)
	 *
	 *  0    - 
	 *  1    - 
	 *  2    - 
	 *  3    GPS time and week are set (GSD4e)
	 *  4    UTC offset verified (GSD4e)
	 *  5    SBAS ranging used in solution (GSD4e)
	 *  6    (SIRFdrive)
	 *  7    (SIRFdrive)
	 */

	tow = srf_data_uint32(data, 7);

	if (device->seen & (SRF_MESSAGE_MASK_MEASURED_TRACKER |
			    SRF_MESSAGE_MASK_CLOCK_STATUS |
			    SRF_MESSAGE_MASK_GEODETIC_NAVIGATION |
			    SRF_MESSAGE_MASK_SOLUTION))
	{
	    if (srf_compare_tow(context->tow, tow))
	    {
		device->seen &= ~(SRF_MESSAGE_MASK_MEASURED_TRACKER |
				  SRF_MESSAGE_MASK_CLOCK_STATUS |
				  SRF_MESSAGE_MASK_GEODETIC_NAVIGATION |
				  SRF_MESSAGE_MASK_SOLUTION);

		device->location.mask = 0;
	    }
	}

	context->week = srf_data_uint16(data, 5);
	context->tow = tow;

	device->location.time.year = srf_data_uint16(data, 11);
	device->location.time.month = srf_data_uint8(data, 13);
	device->location.time.day = srf_data_uint8(data, 14);
	device->location.time.hour = srf_data_uint8(data, 15);
	device->location.time.min = srf_data_uint8(data, 16);
	device->location.time.sec = srf_data_uint16(data, 17);

	device->location.latitude = srf_data_int32(data, 23);
	device->location.longitude = srf_data_int32(data, 27);
	device->location.altitude = srf_data_int32(data, 31) * 10;
	device->location.speed = srf_data_uint16(data, 40) * 10;
	device->location.course = srf_data_uint16(data, 42) * 1000;
	device->location.climb = srf_data_int16(data, 46) * 10;
	device->location.ehpe = srf_data_uint32(data, 50) * 10;
	device->location.evpe = srf_data_uint32(data, 54) * 10;
	device->location.numsv = srf_data_uint8(data, 88);
	device->location.hdop = srf_data_uint8(data, 89) * 20;

	device->location.type = GPS_LOCATION_TYPE_NONE;
	device->location.quality = GPS_LOCATION_QUALITY_NONE;

	switch (srf_data_uint16(data, 3) & 0x07) {
	case 0x00: /* No Solution               */
	case 0x01: /* 1-SV KF Solution          */
	case 0x02: /* 2-SV KF Solution          */
	case 0x07: /* DR Solution               */
	    break;
	    
	case 0x03: /* 3-SV KF solution          */
	case 0x05: /* 2D least squares solution */
	    device->location.type = GPS_LOCATION_TYPE_2D;
	    break;
	    
	case 0x04: /* 4-SV KF solution          */
	case 0x06: /* 32D least squares solution */
	    device->location.type = GPS_LOCATION_TYPE_3D;
	    break;
	}
	
	if (device->location.type >= GPS_LOCATION_TYPE_2D)
	{
	    if (srf_data_uint16(data, 3) & 0x80)
	    {
		device->location.quality = GPS_LOCATION_QUALITY_DIFFERENTIAL;
	    }
	    else
	    {
		device->location.quality = GPS_LOCATION_QUALITY_AUTONOMOUS;
	    }
	}

	if (context->week && device->location.time.year)
	{
	    device->location.correction = utc_offset_time(&device->location.time, context->week, context->tow);
	    
	    device->location.mask |= (GPS_LOCATION_MASK_TIME | GPS_LOCATION_MASK_CORRECTION);
	}

#if (GPS_CONFIG_SATELLITES >= 1)
	context->sv_used_mask = srf_data_uint32(data, 19);
#endif /* GPS_CONFIG_SATELLITES >= 1 */

	device->location.mask |= (GPS_LOCATION_MASK_POSITION |
				  GPS_LOCATION_MASK_ALTITUDE |
				  GPS_LOCATION_MASK_SPEED |
				  GPS_LOCATION_MASK_COURSE |
				  GPS_LOCATION_MASK_CLIMB |
				  GPS_LOCATION_MASK_EHPE |
				  GPS_LOCATION_MASK_EVPE |
				  GPS_LOCATION_MASK_HDOP);

	device->seen |= SRF_MESSAGE_MASK_GEODETIC_NAVIGATION;
	device->seen &= ~SRF_MESSAGE_MASK_SOLUTION;
	break;

    default:
	break;
    }

    if (device->init == GPS_INIT_DONE)
    {
	expected = device->expected & (SRF_MESSAGE_MASK_CLOCK_STATUS | SRF_MESSAGE_MASK_GEODETIC_NAVIGATION);

	if ((device->seen & expected) == expected)
	{
	    gps_location(device);
	
	    device->seen &= ~(SRF_MESSAGE_MASK_CLOCK_STATUS | SRF_MESSAGE_MASK_GEODETIC_NAVIGATION);
	    device->seen |= SRF_MESSAGE_MASK_SOLUTION;
	}
    
#if (GPS_CONFIG_SATELLITES >= 1)
	expected = device->expected & (SRF_MESSAGE_MASK_MEASURED_TRACKER);
    
	if ((device->seen & SRF_MESSAGE_MASK_SOLUTION) && ((device->seen & expected) == expected))
	{
	    for (n = 0; n < device->satellites.count; n++)
	    {
		svid = device->satellites.info[n].id;
	    
		if ((svid >= 1) && (svid <= 32) && (context->sv_used_mask & (1ul << (svid -1))))
		{
		    device->satellites.info[n].state |= GPS_SATELLITES_STATE_NAVIGATING;
		}
	    }
	
	    gps_satellites(device);
	
	    device->seen &= ~(SRF_MESSAGE_MASK_MEASURED_TRACKER);
	}
#endif /* GPS_CONFIG_SATELLITES >= 1 */
    }
}

static void srf_send(gps_device_t *device, const uint8_t *data)
{
    uint32_t command;
    unsigned int count;

    command = data[4];
    count   = (data[3] | (data[2] << 8)) +8;

    gps_send(&gps_device, command, data, count);
}

static const uint8_t srf_msg_initialize_data_source_hot[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x19,             /* LENGTH_1, LENGTH_2        */
    0x80,
    0x00, 0x00, 0x00, 0x00, /* ECEF X                    */
    0x00, 0x00, 0x00, 0x00, /* ECEF Y                    */
    0x00, 0x00, 0x00, 0x00, /* ECEF Z                    */
    0x00, 0x00, 0x00, 0x00, /* CLOCK DRIFT               */
    0x00, 0x00, 0x00, 0x00, /* TOW                       */
    0x00, 0x00,             /* WEEK                      */
    0x0c,                   /* CHANNELS                  */
    0x00,                   /* RESET                     */
    0x00, 0x8c,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_initialize_data_source_warm[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x19,             /* LENGTH_1, LENGTH_2        */
    0x80,
    0x00, 0x00, 0x00, 0x00, /* ECEF X                    */
    0x00, 0x00, 0x00, 0x00, /* ECEF Y                    */
    0x00, 0x00, 0x00, 0x00, /* ECEF Z                    */
    0x00, 0x00, 0x00, 0x00, /* CLOCK DRIFT               */
    0x00, 0x00, 0x00, 0x00, /* TOW                       */
    0x00, 0x00,             /* WEEK                      */
    0x0c,                   /* CHANNELS                  */
    0x02,                   /* RESET                     */
    0x00, 0x8e,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_initialize_data_source_cold[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x19,             /* LENGTH_1, LENGTH_2        */
    0x80,
    0x00, 0x00, 0x00, 0x00, /* ECEF X                    */
    0x00, 0x00, 0x00, 0x00, /* ECEF Y                    */
    0x00, 0x00, 0x00, 0x00, /* ECEF Z                    */
    0x00, 0x00, 0x00, 0x00, /* CLOCK DRIFT               */
    0x00, 0x00, 0x00, 0x00, /* TOW                       */
    0x00, 0x00,             /* WEEK                      */
    0x0c,                   /* CHANNELS                  */
    0x04,                   /* RESET                     */
    0x00, 0x90,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_default_debug[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x04,                   /* MODE                      */
    0x00,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xaa,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_navigation_debug[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x05,                   /* MODE                      */
    0x00,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xab,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_enable_7[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x07,                   /* MESSAGE                   */
    0x01,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xae,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_enable_41[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x29,                   /* MESSAGE                   */
    0x01,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xd0,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_enable_4_5[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x04,                   /* MESSAGE                   */
    0x05,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xaf,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_enable_4_1[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x04,                   /* MESSAGE                   */
    0x01,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xab,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_4[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x04,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xaa,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_2[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x02,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xa8,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_9[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x09,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xaf,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_13[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x0d,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xb3,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_51[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x33,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xd9,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_56[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x38,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0xde,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_92[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x5c,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x01, 0x02,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_set_message_rate_disable_93[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x08,             /* LENGTH_1, LENGTH_2        */
    0xa6,
    0x00,                   /* MODE                      */
    0x5d,                   /* MESSAGE                   */
    0x00,                   /* RATE                      */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x01, 0x03,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};
    
static const uint8_t srf_msg_mode_control_sbas_5hz[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x0e,             /* LENGTH_1, LENGTH_2        */
    0x88,
    0x00, 0x00,             /*                           */
    0x04,                   /* DEGRADED MODE             */
    0x0c,                   /* POSITION CALC MODE        */
    0x00,                   /*                           */
    0x00, 0x00,             /* ALTITUDE                  */
    0x00,                   /* ALTITUDE HOLD MODE        */
    0x00,                   /* ALTITUDE HOLD SOURCE      */
    0x00,                   /*                           */
    0x00,                   /* DEGRADED TIMEOUT          */
    0x00,                   /* DEAD RECKONING TIMEOUT    */
    0x01,                   /* TRACK SMOOTHING           */
    0x00, 0x99,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_mode_control_sbas_1hz[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x0e,             /* LENGTH_1, LENGTH_2        */
    0x88,
    0x00, 0x00,             /*                           */
    0x04,                   /* DEGRADED MODE             */
    0x08,                   /* POSITION CALC MODE        */
    0x00,                   /*                           */
    0x00, 0x00,             /* ALTITUDE                  */
    0x00,                   /* ALTITUDE HOLD MODE        */
    0x00,                   /* ALTITUDE HOLD SOURCE      */
    0x00,                   /*                           */
    0x00,                   /* DEGRADED TIMEOUT          */
    0x00,                   /* DEAD RECKONING TIMEOUT    */
    0x01,                   /* TRACK SMOOTHING           */
    0x00, 0x95,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};
    
static const uint8_t srf_msg_mode_control_5hz[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x0e,             /* LENGTH_1, LENGTH_2        */
    0x88,
    0x00, 0x00,             /*                           */
    0x04,                   /* DEGRADED MODE             */
    0x04,                   /* POSITION CALC MODE        */
    0x00,                   /*                           */
    0x00, 0x00,             /* ALTITUDE                  */
    0x00,                   /* ALTITUDE HOLD MODE        */
    0x00,                   /* ALTITUDE HOLD SOURCE      */
    0x00,                   /*                           */
    0x00,                   /* DEGRADED TIMEOUT          */
    0x00,                   /* DEAD RECKONING TIMEOUT    */
    0x01,                   /* TRACK SMOOTHING           */
    0x00, 0x91,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_mode_control_1hz[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x0e,             /* LENGTH_1, LENGTH_2        */
    0x88,
    0x00, 0x00,             /*                           */
    0x04,                   /* DEGRADED MODE             */
    0x00,                   /* POSITION CALC MODE        */
    0x00,                   /*                           */
    0x00, 0x00,             /* ALTITUDE                  */
    0x00,                   /* ALTITUDE HOLD MODE        */
    0x00,                   /* ALTITUDE HOLD SOURCE      */
    0x00,                   /*                           */
    0x00,                   /* DEGRADED TIMEOUT          */
    0x00,                   /* DEAD RECKONING TIMEOUT    */
    0x01,                   /* TRACK SMOOTHING           */
    0x00, 0x8d,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};


static const uint8_t srf_msg_static_navigation[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x02,             /* LENGTH_1, LENGTH_2        */
    0x8f,
    0x00,                   /* STATIC NAVIGATION FLAG    */
    0x00, 0x8f,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_dop_mask[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x05,             /* LENGTH_1, LENGTH_2        */
    0x89,
    0x04,                   /* DOP SELECTION             */
    0x32,                   /* GDOP VALUE                */
    0x32,                   /* PDOP VALUE                */
    0x32,                   /* HDOP VALUE                */
    0x01, 0x23,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_elevation_mask_15[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x05,             /* LENGTH_1, LENGTH_2        */
    0x8b,
    0x00, 0x32,             /* TRACKING MASK             */
    0x00, 0x95,             /* NAVIGATION MASK           */
    0x01, 0x52,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_elevation_mask_10[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x05,             /* LENGTH_1, LENGTH_2        */
    0x8b,
    0x00, 0x32,             /* TRACKING MASK             */
    0x00, 0x64,             /* NAVIGATION MASK           */
    0x01, 0x21,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};
static const uint8_t srf_msg_elevation_mask_5[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x05,             /* LENGTH_1, LENGTH_2        */
    0x8b,
    0x00, 0x32,             /* TRACKING MASK             */
    0x00, 0x32,             /* NAVIGATION MASK           */
    0x00, 0xef,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_power_mask[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x03,             /* LENGTH_1, LENGTH_2        */
    0x8c,
    0x00,                   /* TRACKING MASK             */
    0x0c,                   /* NAVIGATION MASK           */
    0x00, 0x98,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_dgps_source_sbas[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x07,             /* LENGTH_1, LENGTH_2        */
    0x85,
    0x01,                   /* DGPS SOURCE               */
    0x00, 0x00, 0x00, 0x00, /* INTERNAL BEACON FREQUENCY */
    0x00,                   /* INTERNAL BEACON BIT RATE  */
    0x00, 0x86,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_dgps_source_none[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x07,             /* LENGTH_1, LENGTH_2        */
    0x85,
    0x00,                   /* DGPS SOURCE               */
    0x00, 0x00, 0x00, 0x00, /* INTERNAL BEACON FREQUENCY */
    0x00,                   /* INTERNAL BEACON BIT RATE  */
    0x00, 0x85,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_dgps_control[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x03,             /* LENGTH_1, LENGTH_2        */
    0x8a,
    0x00,                   /* DGPS SELECTION            */
    0x1e,                   /* DGPS TIMEOUT              */
    0x00, 0xa8,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_sbas_parameters_msas[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x06,             /* LENGTH_1, LENGTH_2        */
    0xaa,
    0x04,                   /* SBAS REGION               */
    0x01,                   /* SBAS MODE                 */
    0x00,                   /* FLAGS                     */
    0x00,                   /*                           */
    0x00,
    0x00, 0xaf,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_sbas_parameters_egnos[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x06,             /* LENGTH_1, LENGTH_2        */
    0xaa,
    0x03,                   /* SBAS REGION               */
    0x01,                   /* SBAS MODE                 */
    0x00,                   /* FLAGS                     */
    0x00,                   /*                           */
    0x00,
    0x00, 0xae,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_sbas_parameters_waas[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x06,             /* LENGTH_1, LENGTH_2        */
    0xaa,
    0x02,                   /* SBAS REGION               */
    0x01,                   /* SBAS MODE                 */
    0x00,                   /* FLAGS                     */
    0x00,                   /*                           */
    0x00,
    0x00, 0xad,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_sbas_parameters_auto[] = {
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x06,             /* LENGTH_1, LENGTH_2        */
    0xaa,
    0x00,                   /* SBAS REGION               */
    0x01,                   /* SBAS MODE                 */
    0x00,                   /* FLAGS                     */
    0x00,                   /*                           */
    0x00,
    0x00, 0xab,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_disable_sirf_aiding[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x04,             /* LENGTH_1, LENGTH_2        */
    0xe8,
    0x20,
    0x01,                   /* DISABLE SGEE              */
    0x01,                   /* DISABLE CGEE              */
    0x01, 0x0a,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t srf_msg_disable_cgee_prediction[] = { 
    0xa0, 0xa2,             /* START_1, START_2          */
    0x00, 0x06,             /* LENGTH_1, LENGTH_2        */
    0xe8,
    0xfe,
    0x00, 0x00, 0x00, 0x00, /* TIME                      */
    0x01, 0xe6,             /* CHECKSUM_1, CHECKSUM_2    */
    0xb0, 0xb3,             /* END_1, END_2              */
};

static const uint8_t *srf_init_table[] = {
    srf_msg_set_message_rate_disable_default_debug,
    srf_msg_set_message_rate_disable_navigation_debug,
    srf_msg_set_message_rate_enable_7,
    srf_msg_set_message_rate_enable_41,
#if (GPS_CONFIG_SATELLITES == 2)
    srf_msg_set_message_rate_enable_4_1,
#elif (GPS_CONFIG_SATELLITES == 1)
#if (GPS_CONFIG_RATE >= 5)
    srf_msg_set_message_rate_enable_4_5,
#else /* GPS_CONFIG_RATE >= 5 */
    srf_msg_set_message_rate_enable_4_1,
#endif /* GPS_CONFIG_RATE >= 5 */
#else
    srf_msg_set_message_rate_disable_4,
#endif
    srf_msg_set_message_rate_disable_2,
    srf_msg_set_message_rate_disable_9,
    srf_msg_set_message_rate_disable_13,
#if (GPS_CONFIG_SIRF >= 2)
    srf_msg_set_message_rate_disable_51,
    srf_msg_set_message_rate_disable_56,
    srf_msg_set_message_rate_disable_92,
    srf_msg_set_message_rate_disable_93,
#endif /*  GPS_CONFIG_SIRF >= 2 */
#if (GPS_CONFIG_SBAS >= 1)
#if (GPS_CONFIG_RATE >= 5)
    srf_msg_mode_control_sbas_5hz,
#else /* GPS_CONFIG_RATE >= 5 */
    srf_msg_mode_control_sbas_1hz,
#endif /* GPS_CONFIG_RATE >= 5 */
#else /* GPS_CONFIG_SBAS >= 1 */
#if (GPS_CONFIG_RATE >= 5)
    srf_msg_mode_control_5hz,
#else /* GPS_CONFIG_RATE >= 5 */
    srf_msg_mode_control_1hz,
#endif /* GPS_CONFIG_RATE >= 5 */
#endif /* GPS_CONFIG_SBAS >= 1 */
    srf_msg_static_navigation,
    srf_msg_dop_mask,
#if (GPS_CONFIG_ELEVATION >= 15)
    srf_msg_elevation_mask_15,
#elif (GPS_CONFIG_ELEVATION >= 10)
    srf_msg_elevation_mask_10,
#else
    srf_msg_elevation_mask_5,
#endif
    srf_msg_power_mask,
#if (GPS_CONFIG_SBAS >= 1)
    srf_msg_dgps_source_sbas,
#else /* GPS_CONFIG_SBAS >= 1 */
    srf_msg_dgps_source_none,
#endif /* GPS_CONFIG_SBAS >= 1 */
    srf_msg_dgps_control,
#if (GPS_CONFIG_SBAS >= 1)
#if (GPS_CONFIG_SBAS == 4)
    srf_msg_sbas_parameters_msas,
#elif (GPS_CONFIG_SBAS == 3)
    srf_msg_sbas_parameters_egnos,
#elif (GPS_CONFIG_SBAS == 2)
    srf_msg_sbas_parameters_waas,
#else
    srf_msg_sbas_parameters_auto,
#endif
#endif /* GPS_CONFIG_SBAS >= 1 */
#if (GPS_CONFIG_SIRF >= 2)
    srf_msg_disable_sirf_aiding,
    srf_msg_disable_cgee_prediction,
#endif /*  GPS_CONFIG_SIRF >= 2 */
    NULL,
};

static void srf_initialize(gps_device_t *device, unsigned int response, uint32_t command)
{    
    if (response == GPS_RESPONSE_SRF_MESSAGE)
    {
	
	if (device->reset == GPS_RESET_HOT_START)
	{
	    srf_send(device, srf_msg_initialize_data_source_hot);

	    device->init = GPS_INIT_SRF_STARTUP;
	}
	else if (device->reset == GPS_RESET_WARM_START)
	{
	    srf_send(device, srf_msg_initialize_data_source_warm);

	    device->init = GPS_INIT_SRF_STARTUP;
	}
	else if (device->reset == GPS_RESET_COLD_START)
	{
	    srf_send(device, srf_msg_initialize_data_source_cold);

	    device->init = GPS_INIT_SRF_STARTUP;
	}
	else
	{
	    srf_send(device, srf_init_table[0]);

	    device->init = GPS_INIT_SRF_INIT_TABLE;
	}
    }
    else
    {
	if (device->init == GPS_INIT_SRF_STARTUP)
	{
	    if (response == GPS_RESPONSE_ACK)
	    {
		srf_send(device, srf_init_table[0]);

		device->init = GPS_INIT_SRF_INIT_TABLE;
	    }
	    else
	    {
		device->init = GPS_INIT_SRF_BAUD_RATE;
	    }
	}
	else
	{
	    if (response == GPS_RESPONSE_ACK)
	    {
		device->init++;

		if (srf_init_table[device->init - GPS_INIT_SRF_INIT_TABLE])
		{
		    srf_send(device, srf_init_table[device->init - GPS_INIT_SRF_INIT_TABLE]);
		}
		else
		{
#if (GPS_CONFIG_SATELLITES >= 1)
		    if (device->satellites_callback == NULL)
		    {
			srf_send(device, srf_msg_set_message_rate_disable_4);
		    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */

		    device->init = GPS_INIT_DONE;
		    device->protocol = GPS_PROTOCOL_MASK_SRF;
		    device->expected = (SRF_MESSAGE_MASK_CLOCK_STATUS | SRF_MESSAGE_MASK_GEODETIC_NAVIGATION | SRF_MESSAGE_MASK_MEASURED_TRACKER);
		    device->seen = 0;
		    device->location.type = 0;
		    device->location.mask = 0;

		    (*device->ini_callback)(1);
		}
	    }
	    else
	    {
		device->init = GPS_INIT_SRF_BAUD_RATE;
	    }
	}
    }
}

#endif /* GPS_CONFIG_SIRF >= 1 */

/************************************************************************************/

#if (GPS_CONFIG_UBLOX >= 1)

static inline int8_t ubx_data_int8(const uint8_t *data, unsigned int offset)
{
    return (int8_t)data[offset];
}

static inline int16_t ubx_data_int16(const uint8_t *data, unsigned int offset)
{
    return (int16_t)(((uint16_t)data[offset+0] << 0) |
		     ((uint16_t)data[offset+1] << 8));
}

static inline int32_t ubx_data_int32(const uint8_t *data, unsigned int offset)
{
    return (int32_t)(((uint32_t)data[offset+0] <<  0) |
		     ((uint32_t)data[offset+1] <<  8) |
		     ((uint32_t)data[offset+2] << 16) |
		     ((uint32_t)data[offset+3] << 24));
}

static inline uint8_t ubx_data_uint8(const uint8_t *data, unsigned int offset)
{
    return (uint8_t)data[offset];
}

static inline uint16_t ubx_data_uint16(const uint8_t *data, unsigned int offset)
{
    return (uint16_t)(((uint16_t)data[offset+0] << 0) |
		      ((uint16_t)data[offset+1] << 8));
}

static inline uint32_t ubx_data_uint32(const uint8_t *data, unsigned int offset)
{
    return (uint32_t)(((uint32_t)data[offset+0] <<  0) |
		      ((uint32_t)data[offset+1] <<  8) |
		      ((uint32_t)data[offset+2] << 16) |
		      ((uint32_t)data[offset+3] << 24));
}

static void ubx_start_message(gps_device_t *device, unsigned int message, unsigned int length)
{
#if (GPS_CONFIG_SATELLITES >= 1)
    if (message == 0x0130)
    {
	/* UBX-NAV-SVINFO/UBX-NAV-SAT */
	    
	device->last = 20;
	device->satellites.count = 0;
	
	device->seen &= ~UBX_MESSAGE_MASK_NAV_SVINFO;
    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */
}

static void ubx_parse_message(gps_device_t *device, unsigned int message, uint8_t *data, unsigned int count)
{
#if (GPS_CONFIG_SATELLITES >= 1)
    unsigned int svid;

    if (message == 0x0130)
    {
	/* UBX-NAV-SVINFO */

	svid = ubx_data_uint8(data, 9);

	if ((svid >= 1) && (svid <= 32))
	{
	    /* GPS */
	}
	else if ((svid >= 33) && (svid <= 64))
	{
	    /* BEIDOU */
	    svid += (201 + 5 - 33);
	}
	else if ((svid >= 65) && (svid <= 96))
	{
	    /* GLONASS */
	}
	else if ((svid >= 120) && (svid <= 151))
	{
	    /* SBAS */
	    svid -= 87;
	}
	else if ((svid >= 152) && (svid <= 158))
	{
	    /* SBAS */
	}
	else if ((svid >= 159) && (svid <= 163))
	{
	    /* BEIDOU */
	    svid += (201 - 159);
	}
	else if ((svid >= 193) && (svid <= 200))
	{
	    /* QZSS */
	}
	else if (svid == 255)
	{
	    /* GLONASS */
	}
	else
	{
	    svid = 0;
	}

	if (svid && (device->satellites.count < GPS_SATELLITES_COUNT_MAX))
	{
	    device->satellites.info[device->satellites.count].id = svid;

	    if (ubx_data_int8(data, 13) > 0)
	    {
		device->satellites.info[device->satellites.count].elev = ubx_data_int8(data, 13);
		device->satellites.info[device->satellites.count].azim = ubx_data_int16(data, 14);
	    }
	    else
	    {
		device->satellites.info[device->satellites.count].elev = 0;
		device->satellites.info[device->satellites.count].azim = 0;
	    }

	    device->satellites.info[device->satellites.count].snr = ubx_data_uint8(data, 12);

	    switch (ubx_data_uint8(data, 11) & 0x0f) {
	    case 0x00: /* NO SIGNAL */
	    case 0x01: /* SEARCHING */ 
		device->satellites.info[device->satellites.count].state = 0;
		break;

	    case 0x02: /* SIGNAL ACQUIRED */
	    case 0x03: /* SIGNAL ACQUIRED, BUT UNUSABLE */
		device->satellites.info[device->satellites.count].state = (GPS_SATELLITES_STATE_TRACKING);
		break;

	    case 0x04: /* SIGNAL ACQUIRED, CODE LOCKED */
		device->satellites.info[device->satellites.count].state = (GPS_SATELLITES_STATE_TRACKING | GPS_SATELLITES_STATE_CODE_LOCK);
		break;

	    case 0x05: /* SIGNAL ACQUIRED, CODE LOCKED, CARRIER LOCKED */
	    case 0x06: /* SIGNAL ACQUIRED, CODE LOCKED, CARRIER LOCKED */
	    case 0x07: /* SIGNAL ACQUIRED, CODE LOCKED, CARRIER LOCKED */
		device->satellites.info[device->satellites.count].state = (GPS_SATELLITES_STATE_TRACKING | GPS_SATELLITES_STATE_CODE_LOCK | GPS_SATELLITES_STATE_CARRIER_LOCK);
		break;

	    default:
		device->satellites.info[device->satellites.count].state = 0;
		break;
	    }

	    if (device->satellites.info[device->satellites.count].state & GPS_SATELLITES_STATE_TRACKING)
	    {
		if (ubx_data_uint8(data, 10) & 0x01)
		{
		    device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_NAVIGATING;
		}

		if (ubx_data_uint8(data, 10) & 0x02)
		{
		    device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_CORRECTION;
		}

		if ((ubx_data_uint8(data, 10) & 0x0c) == 0x0c)
		{
		    device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_EPHEMERIS;
		}

		if (ubx_data_uint8(data, 10) & 0x10)
		{
		    device->satellites.info[device->satellites.count].state |= GPS_SATELLITES_STATE_UNHEALTHY;
		}
	    }

	    device->satellites.count++;
	}
	
	device->first += 12;
	device->last += 12;
    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */
}

static void ubx_end_message(gps_device_t *device, unsigned int message, uint8_t *data, unsigned int count)
{
    ubx_context_t *context = &device->ubx;
    unsigned int expected;
    uint16_t week, command;
    int32_t tow;

    if ((message >> 8) == 0x01)
    {
	if (device->seen & (UBX_MESSAGE_MASK_NAV_CLOCK |
			    UBX_MESSAGE_MASK_NAV_DOP |
			    UBX_MESSAGE_MASK_NAV_PVT |
			    UBX_MESSAGE_MASK_NAV_POSLLH |
			    UBX_MESSAGE_MASK_NAV_SOL |
			    UBX_MESSAGE_MASK_NAV_SVINFO |
			    UBX_MESSAGE_MASK_NAV_VELNED |
			    UBX_MESSAGE_MASK_NAV_TIMEGPS |
			    UBX_MESSAGE_MASK_NAV_TIMEUTC |
			    UBX_MESSAGE_MASK_SOLUTION))
	{
	    if (context->itow != ubx_data_uint32(data, 0))
	    {
		device->seen = 0;
		device->location.type = 0;
		device->location.mask = 0;
	    }
	}
	
	context->itow = ubx_data_uint32(data, 0);

	switch (message & 0xff) {

#if (GPS_CONFIG_UBLOX == 1)
	case 0x02:
	    /* UBX-NAV-POSLLH */

	    device->location.latitude = ubx_data_int32(data, 8);
	    device->location.longitude = ubx_data_int32(data, 4);
	    device->location.altitude = ubx_data_int32(data, 12);
	    device->location.ehpe = ubx_data_uint32(data, 20);
	    device->location.evpe = ubx_data_uint32(data, 24);

	    device->location.mask |= (GPS_LOCATION_MASK_POSITION |
				      GPS_LOCATION_MASK_ALTITUDE |
				      GPS_LOCATION_MASK_EHPE |;
				      GPS_LOCATION_MASK_EVPE);

	    device->seen |= UBX_MESSAGE_MASK_NAV_POSLLH;
	    device->seen &= ~UBX_MESSAGE_MASK_SOLUTION;
	    break;
#endif /* GPS_CONFIG_UBLOX == 1 */

	case 0x04:
	    /* UBX-NAV-DOP */

	    device->location.hdop = ubx_data_uint16(data, 12);
	    device->location.vdop = ubx_data_uint16(data, 10);
	    device->location.tdop = ubx_data_uint16(data, 8);

	    device->location.mask |= (GPS_LOCATION_MASK_HDOP |
				      GPS_LOCATION_MASK_VDOP |
				      GPS_LOCATION_MASK_TDOP);

	    device->seen |= UBX_MESSAGE_MASK_NAV_DOP;
	    break;

#if (GPS_CONFIG_UBLOX == 1)
	case 0x06:
	    /* UBX-NAV-SOL */

	    if ((ubx_data_uint8(data, 11) & 0x0c) == 0x0c)
	    {
		tow  = ubx_data_uint32(data, 0) + (ubx_data_int32(data, 4) + 500000) / 1000000;
		week = ubx_data_uint16(data, 8);

		if (tow < 0)
		{
		    tow += 604800000;
		    week -= 1;
		}
		
		if (tow >= 604800000)
		{
		    tow -= 604800000;
		    week += 1;
		}
		
		context->week = week;
		context->tow = tow;
	    }
	    else
	    {
		context->week = 0;
		context->tow = 0;
	    }

	    switch (ubx_data_uint8(data, 10)) {
	    case 0x00:
		device->location.type = GPS_LOCATION_TYPE_NONE;
		device->location.quality = GPS_LOCATION_QUALITY_NONE;
		break;

	    case 0x01:
		device->location.type = GPS_LOCATION_TYPE_NONE;
		device->location.quality = GPS_LOCATION_QUALITY_ESTIMATED;
		break;

	    case 0x02:
		device->location.type = GPS_LOCATION_TYPE_2D;

		if (ubx_data_uint8(data, 11) & 0x01)
		{
		    device->location.quality = ((ubx_data_uint8(data, 11) & 0x02) ? GPS_LOCATION_QUALITY_DIFFERENTIAL : GPS_LOCATION_QUALITY_AUTONOMOUS);
		}
		else
		{
		    device->location.quality = GPS_LOCATION_QUALITY_NONE;
		}
		break;

	    case 0x03:
		device->location.type = GPS_LOCATION_TYPE_3D;

		if (ubx_data_uint8(data, 11) & 0x01)
		{
		    device->location.quality = ((ubx_data_uint8(data, 11) & 0x02) ? GPS_LOCATION_QUALITY_DIFFERENTIAL : GPS_LOCATION_QUALITY_AUTONOMOUS);
		}
		else
		{
		    device->location.quality = GPS_LOCATION_QUALITY_NONE;
		}
		break;

	    case 0x04:
		device->location.type = GPS_LOCATION_TYPE_2D;
		device->location.quality = GPS_LOCATION_QUALITY_ESTIMATED;
		break;

	    case 0x05:
		device->location.type = GPS_LOCATION_TYPE_TIME;
		device->location.quality = GPS_LOCATION_QUALITY_NONE;
		break;
	    }

	    device->location.numsv = ubx_data_uint8(data, 47);

	    device->seen |= UBX_MESSAGE_MASK_NAV_SOL;
	    device->seen &= ~UBX_MESSAGE_MASK_SOLUTION;
	    break;
#endif /* GPS_CONFIG_UBLOX == 1 */

#if (GPS_CONFIG_UBLOX >= 2)
	case 0x07:
	    /* UBX-NAV-PVT */

	    if ((ubx_data_uint8(data, 11) & 0x03) == 0x03)
	    {
		device->location.time.year = ubx_data_uint16(data, 4);
		device->location.time.month = ubx_data_uint8(data, 6);
		device->location.time.day = ubx_data_uint8(data, 7);
		device->location.time.hour = ubx_data_uint8(data, 8);
		device->location.time.min = ubx_data_uint8(data, 9);
		device->location.time.sec = ubx_data_uint8(data, 10) * 1000;
		
		if (ubx_data_int32(data, 16) > 0)
		{
		    device->location.time.sec += ((ubx_data_int32(data, 16) + 500000) / 1000000);
		}
	    }
	    else
	    {
		device->location.time.year = 0;
		device->location.time.month = 0;
		device->location.time.day = 0;
		device->location.time.hour = 0;
		device->location.time.min = 0;
		device->location.time.sec = 0;
	    }

	    device->location.latitude = ubx_data_int32(data, 28);
	    device->location.longitude = ubx_data_int32(data, 24);
	    device->location.altitude = ubx_data_int32(data, 32);
	    device->location.speed = ubx_data_int32(data, 60);
	    device->location.course = ubx_data_int32(data, 64);
	    device->location.climb = - ubx_data_int32(data, 56);
	    device->location.ehpe = ubx_data_uint32(data, 40);
	    device->location.evpe = ubx_data_uint32(data, 44);
	    device->location.esve = ubx_data_uint32(data, 68);
	    device->location.ecve = ubx_data_uint32(data, 72);

	    switch (ubx_data_uint8(data, 20)) {
	    case 0x00:
		device->location.type = GPS_LOCATION_TYPE_NONE;
		device->location.quality = GPS_LOCATION_QUALITY_NONE;
		break;

	    case 0x01:
		device->location.type = GPS_LOCATION_TYPE_NONE;
		device->location.quality = GPS_LOCATION_QUALITY_ESTIMATED;
		break;

	    case 0x02:
		device->location.type = GPS_LOCATION_TYPE_2D;

		if (ubx_data_uint8(data, 21) & 0x01)
		{
		    device->location.quality = ((ubx_data_uint8(data, 21) & 0x02) ? GPS_LOCATION_QUALITY_DIFFERENTIAL : GPS_LOCATION_QUALITY_AUTONOMOUS);
		}
		else
		{
		    device->location.quality = GPS_LOCATION_QUALITY_NONE;
		}
		break;

	    case 0x03:
		device->location.type = GPS_LOCATION_TYPE_3D;

		if (ubx_data_uint8(data, 21) & 0x01)
		{
		    device->location.quality = ((ubx_data_uint8(data, 21) & 0x02) ? GPS_LOCATION_QUALITY_DIFFERENTIAL : GPS_LOCATION_QUALITY_AUTONOMOUS);
		}
		else
		{
		    device->location.quality = GPS_LOCATION_QUALITY_NONE;
		}
		break;

	    case 0x04:
		device->location.type = GPS_LOCATION_TYPE_2D;
		device->location.quality = GPS_LOCATION_QUALITY_ESTIMATED;
		break;

	    case 0x05:
		device->location.type = GPS_LOCATION_TYPE_TIME;
		device->location.quality = GPS_LOCATION_QUALITY_NONE;
		break;
	    }

	    device->location.numsv = ubx_data_uint8(data, 23);

	    device->location.mask |= (GPS_LOCATION_MASK_POSITION |
				      GPS_LOCATION_MASK_ALTITUDE |
				      GPS_LOCATION_MASK_SPEED |
				      GPS_LOCATION_MASK_COURSE |
				      GPS_LOCATION_MASK_CLIMB |
				      GPS_LOCATION_MASK_EHPE |
				      GPS_LOCATION_MASK_EVPE |
				      GPS_LOCATION_MASK_ESVE |
				      GPS_LOCATION_MASK_ECVE);

	    device->seen |= UBX_MESSAGE_MASK_NAV_PVT;
	    device->seen &= ~UBX_MESSAGE_MASK_SOLUTION;
	    break;
#endif /* GPS_CONFIG_UBLOX >= 2 */

#if (GPS_CONFIG_UBLOX == 1)
	case 0x12:
	    /* UBX-NAV-VELNED */

	    device->location.speed = ubx_data_int32(data, 20) * 10;
	    device->location.course = ubx_data_int32(data, 24);
	    device->location.climb = - ubx_data_int32(data, 12) * 10;
	    device->location.esve = ubx_data_int32(data, 28) * 10;
	    device->location.ecve = ubx_data_int32(data, 32);

	    device->location.mask |= (GPS_LOCATION_MASK_SPEED |
				      GPS_LOCATION_MASK_COURSE |
				      GPS_LOCATION_MASK_CLIMB |
				      GPS_LOCATION_MASK_ESVE |
				      GPS_LOCATION_MASK_ECVE);

	    device->seen |= UBX_MESSAGE_MASK_NAV_VELNED;
	    device->seen &= ~UBX_MESSAGE_MASK_SOLUTION;
	    break;
#endif /* GPS_CONFIG_UBLOX == 1 */

#if (GPS_CONFIG_UBLOX >= 2)
	case 0x20:
	    /* UBX-NAV-TIMEGPS */

	    if ((ubx_data_uint8(data, 11) & 0x03) == 0x03)
	    {
		tow  = ubx_data_uint32(data, 0) + (ubx_data_int32(data, 4) + 500000) / 1000000;
		week = ubx_data_uint16(data, 8);

		if (tow < 0)
		{
		    tow += 604800000;
		    week -= 1;
		}
		
		if (tow >= 604800000)
		{
		    tow -= 604800000;
		    week += 1;
		}

		context->week = week;
		context->tow = tow;
		
		device->location.correction = ubx_data_uint8(data, 10);
	    }
	    else
	    {
		context->week = 0;
		context->tow = 0;

		device->location.correction = 0;
	    }

	    device->seen |= UBX_MESSAGE_MASK_NAV_TIMEGPS;
	    device->seen &= ~UBX_MESSAGE_MASK_SOLUTION;
	    break;
#endif /* GPS_CONFIG_UBLOX >= 2 */

#if (GPS_CONFIG_UBLOX == 1)
	case 0x21:
	    /* UBX-NAV-TIMEUTC */

	    if ((ubx_data_uint8(data, 19) & 0x03) == 0x03)
	    {
		device->location.time.year = ubx_data_uint16(data, 12);
		device->location.time.month = ubx_data_uint8(data, 14);
		device->location.time.day = ubx_data_uint8(data, 15);
		device->location.time.hour = ubx_data_uint8(data, 16);
		device->location.time.min = ubx_data_uint8(data, 17);
		device->location.time.sec = ubx_data_uint8(data, 18) * 1000;
		
		if (ubx_data_int32(data, 8) > 0)
		{
		    device->location.time.sec += ((ubx_data_int32(data, 8) + 500000) / 1000000);
		}
	    }
	    else
	    {
		device->location.time.year = 0;
		device->location.time.month = 0;
		device->location.time.day = 0;
		device->location.time.hour = 0;
		device->location.time.min = 0;
		device->location.time.sec = 0;
	    }

	    device->seen |= UBX_MESSAGE_MASK_NAV_TIMEUTC;
	    device->seen &= ~UBX_MESSAGE_MASK_SOLUTION;
	    break;
#endif /* GPS_CONFIG_UBLOX == 1 */

	case 0x22:
	    /* UBX-NAV-CLOCK */

	    device->location.bias = ubx_data_int32(data, 4);
	    device->location.drift = ubx_data_int32(data, 8);

	    device->location.mask |= GPS_LOCATION_MASK_CLOCK;

	    device->seen |= UBX_MESSAGE_MASK_NAV_CLOCK;
	    device->seen &= ~UBX_MESSAGE_MASK_SOLUTION;
	    break;

#if (GPS_CONFIG_SATELLITES >= 1)
	case 0x30:
	    /* UBX-NAV-SVINFO */

	    device->seen |= UBX_MESSAGE_MASK_NAV_SVINFO;
	    break;
#endif /* GPS_CONFIG_SATELLITES >= 1 */

	default:
	    break;
	}
    }

    else if (message == 0x0500)
    {
	/* UBX-ACK-NACK */
	command = ((ubx_data_uint8(data, 0) << 8) | ubx_data_uint8(data, 1));

	if (command == device->command)
	{
	    device->command = ~0l;
	    
	    ubx_initialize(device, GPS_RESPONSE_NACK, command);
	}
    }

    else if (message == 0x0501)
    {
	/* UBX-ACK-ACK */
	command = ((ubx_data_uint8(data, 0) << 8) | ubx_data_uint8(data, 1));

	if (command == device->command)
	{
	    device->command = ~0l;
	    
	    ubx_initialize(device, GPS_RESPONSE_ACK, command);
	}
    }

    if (device->init == GPS_INIT_DONE)
    {
	expected = device->expected & (UBX_MESSAGE_MASK_NAV_CLOCK |
				       UBX_MESSAGE_MASK_NAV_DOP |
				       UBX_MESSAGE_MASK_NAV_POSLLH |
				       UBX_MESSAGE_MASK_NAV_PVT |
				       UBX_MESSAGE_MASK_NAV_SOL |
				       UBX_MESSAGE_MASK_NAV_TIMEGPS |
				       UBX_MESSAGE_MASK_NAV_TIMEUTC |
				       UBX_MESSAGE_MASK_NAV_VELNED);
	
	if ((device->seen & expected) == expected)
	{
	    if (context->week && device->location.time.year)
	    {
		if (!(device->seen & UBX_MESSAGE_MASK_NAV_TIMEGPS))
		{
		    device->location.correction = utc_offset_time(&device->location.time, context->week, context->tow);
		}

		device->location.mask |= (GPS_LOCATION_MASK_TIME | GPS_LOCATION_MASK_CORRECTION);
	    }
	    
	    gps_location(device);
	    
	    device->seen &= ~(UBX_MESSAGE_MASK_NAV_CLOCK |
			      UBX_MESSAGE_MASK_NAV_DOP |
			      UBX_MESSAGE_MASK_NAV_POSLLH |
			      UBX_MESSAGE_MASK_NAV_PVT |
			      UBX_MESSAGE_MASK_NAV_SOL |
			      UBX_MESSAGE_MASK_NAV_TIMEGPS |
			      UBX_MESSAGE_MASK_NAV_TIMEUTC |
			      UBX_MESSAGE_MASK_NAV_VELNED);
	    
	    device->seen |= UBX_MESSAGE_MASK_SOLUTION;
	}
	
#if (GPS_CONFIG_SATELLITES >= 1)
	expected = device->expected & UBX_MESSAGE_MASK_NAV_SVINFO;
	
	if ((device->seen & UBX_MESSAGE_MASK_SOLUTION) && ((device->seen & expected) == expected))
	{
	    gps_satellites(device);
	    
	    device->seen &= ~UBX_MESSAGE_MASK_NAV_SVINFO;
	}
#endif /* GPS_CONFIG_SATELLITES >= 1 */
    }
}

static void ubx_send(gps_device_t *device, const uint8_t *data)
{
    uint32_t command;
    unsigned int count;

    command = (data[2] << 8) | data[3];
    count  = (data[4] | (data[5] << 8)) +8;

    gps_send(&gps_device, command, data, count);
}

static const uint8_t ubx_cfg_rst_hot_start[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x04,             /* CLASS, ID                 */
    0x04, 0x00,             /* LENGTH                    */
    0x00, 0x00,             /* MASK                      */
    0x02,                   /* MODE                      */
    0x00,                   /*                           */
    0x10, 0x68,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_rst_warm_start[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x04,             /* CLASS, ID                 */
    0x04, 0x00,             /* LENGTH                    */
    0x01, 0x00,             /* MASK                      */
    0x02,                   /* MODE                      */
    0x00,                   /*                           */
    0x11, 0x6c,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_rst_cold_start[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x04,             /* CLASS, ID                 */
    0x04, 0x00,             /* LENGTH                    */
    0xff, 0xff,             /* MASK                      */
    0x02,                   /* MODE                      */
    0x00,                   /*                           */
    0x0e, 0x61,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_pvt[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x07,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x18, 0xe1,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_sol[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x06,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x17, 0xda,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_posllh[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x02,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x13, 0xbe,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_velned[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x12,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x23, 0x2e,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_timeutc[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x21,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x32, 0x97,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_timegps[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x20,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x31, 0x90,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_clock[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x22,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x33, 0x9e,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_dop[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x04,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x15, 0xcc,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_svinfo_disable[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x30,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x00,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x40, 0xfb,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_svinfo_10[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x30,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x0a,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x4a, 0x2d,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_svinfo_5[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x30,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x05,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x45, 0x14,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nav_svinfo_1[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* CLASS                     */
    0x30,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x01,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x41, 0x00,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nmea_gga[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0xf0,                   /* CLASS                     */
    0x00,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x00,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0xff, 0x23,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nmea_gll[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0xf0,                   /* CLASS                     */
    0x01,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x00,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x00, 0x2a,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nmea_gsa[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0xf0,                   /* CLASS                     */
    0x02,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x00,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x01, 0x31,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nmea_gsv[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0xf0,                   /* CLASS                     */
    0x03,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x00,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x02, 0x38,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nmea_rmc[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0xf0,                   /* CLASS                     */
    0x04,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x00,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x03, 0x3f,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_msg_nmea_vtg[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x01,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0xf0,                   /* CLASS                     */
    0x05,                   /* ID                        */
    0x00,                   /* RATE DDC                  */
    0x00,                   /* RATE UART1                */
    0x00,                   /* RATE UART2                */
    0x00,                   /* RATE USB                  */
    0x00,                   /* RATE SPI                  */
    0x00,                   /*                           */
    0x04, 0x46,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_rate_10HZ[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x08,             /* CLASS, ID                 */
    0x06, 0x00,             /* LENGTH                    */
    0x64, 0x00,             /* MEASUREMENT RATE          */
    0x01, 0x00,             /* NAVIGATION  RATE          */
    0x00, 0x00,             /* TIME REFERENCE            */
    0x79, 0x10,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_rate_5HZ[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x08,             /* CLASS, ID                 */
    0x06, 0x00,             /* LENGTH                    */
    0xc8, 0x00,             /* MEASUREMENT RATE          */
    0x01, 0x00,             /* NAVIGATION  RATE          */
    0x00, 0x00,             /* TIME REFERENCE            */
    0xdd, 0x68,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_rate_4HZ[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x08,             /* CLASS, ID                 */
    0x06, 0x00,             /* LENGTH                    */
    0xfa, 0x00,             /* MEASUREMENT RATE          */
    0x01, 0x00,             /* NAVIGATION  RATE          */
    0x00, 0x00,             /* TIME REFERENCE            */
    0x0f, 0x94,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_rate_1HZ[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x08,             /* CLASS, ID                 */
    0x06, 0x00,             /* LENGTH                    */
    0xe8, 0x03,             /* MEASUREMENT RATE          */
    0x01, 0x00,             /* NAVIGATION  RATE          */
    0x00, 0x00,             /* TIME REFERENCE            */
    0x00, 0x37,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_sbas_msas[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x16,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* MODE                      */
    0x03,                   /* USAGE                     */
    0x03,                   /* MAX SBAS                  */
    0x00,                   /* SCANMODE2                 */
    0x00, 0x02, 0x02, 0x00, /* SCANMODE1 (129, 137)      */
    0x2f, 0xc3,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_sbas_egnos[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x16,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* MODE                      */
    0x03,                   /* USAGE                     */
    0x03,                   /* MAX SBAS                  */
    0x00,                   /* SCANMODE2                 */
    0x51, 0x00, 0x00, 0x00, /* SCANMODE1 (120, 124, 126) */
    0x7c, 0xfd,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_sbas_waas[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x16,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* MODE                      */
    0x03,                   /* USAGE                     */
    0x03,                   /* MAX SBAS                  */
    0x00,                   /* SCANMODE2                 */
    0x00, 0xa0, 0x04, 0x00, /* SCANMODE1 (133, 135, 138) */
    0xcf, 0xa1,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_sbas_auto[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x16,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x01,                   /* MODE                      */
    0x03,                   /* USAGE                     */
    0x03,                   /* MAX SBAS                  */
    0x00,                   /* SCANMODE2                 */
    0x51, 0xa0, 0x04, 0x00, /* SCANMODE1                 */
    0x20, 0xe5,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_sbas_none[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x16,             /* CLASS, ID                 */
    0x08, 0x00,             /* LENGTH                    */
    0x00,                   /* MODE                      */
    0x00,                   /* USAGE                     */
    0x00,                   /* MAX SBAS                  */
    0x00,                   /* SCANMODE2                 */
    0x00, 0x00, 0x00, 0x00, /* SCANMODE1 (133, 135, 138) */
    0x24, 0x8a,             /* CK_A, CK_B                */
};

/* UBLOX7 */

static const uint8_t ubx_cfg_gnss_22ch[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x24, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x16,                                           /* NUM TRACKING CHANNELS HW  */
    0x10,                                           /* NUM TRACKING CHANNELS SW  */
    0x04,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* SBAS                      */
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xb7, 0xec,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_22ch_sbas[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x24, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x16,                                           /* NUM TRACKING CHANNELS HW  */
    0x13,                                           /* NUM TRACKING CHANNELS SW  */
    0x04,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* SBAS                      */
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xbf, 0xbf,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_22ch_qzss[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x24, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x16,                                           /* NUM TRACKING CHANNELS HW  */
    0x13,                                           /* NUM TRACKING CHANNELS SW  */
    0x04,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* SBAS                      */
    0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xbe, 0x88,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_22ch_sbas_qzss[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x24, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x16,                                           /* NUM TRACKING CHANNELS HW  */
    0x16,                                           /* NUM TRACKING CHANNELS SW  */
    0x04,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* SBAS                      */
    0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xc6, 0x5b,                                     /* CK_A, CK_B                */
};

/* UBLOX8 */

static const uint8_t ubx_cfg_gnss_32ch[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x10,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xcd, 0xdb,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_32ch_sbas[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x13,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xd5, 0xee,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_32ch_qzss[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x13,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xd4, 0x8f,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_32ch_sbas_qzss[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x16,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xdc, 0xa2,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_32ch_glonass[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x10,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xe4, 0x6b,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_32ch_sbas_glonass[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x13,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xec, 0x7e,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_32ch_qzss_glonass[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x13,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xeb, 0x1f,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_gnss_32ch_sbas_qzss_glonass[] = {
    0xb5, 0x62,                                     /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x3e,                                     /* CLASS, ID                 */
    0x2c, 0x00,                                     /* LENGTH                    */
    0x00,                                           /* VERSION                   */
    0x20,                                           /* NUM TRACKING CHANNELS HW  */
    0x16,                                           /* NUM TRACKING CHANNELS SW  */
    0x05,                                           /* NUM CONFIG BLOCKS         */
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, /* GPS                       */
    0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* SBAS                      */
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BEIDOU                    */
    0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, /* QZSS                      */
    0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00, /* GLONASS                   */
    0xf3, 0x32,                                     /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_15_airborne[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x08,                   /* AIRBORNE <= 4G            */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0f,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x29, 0x85,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_15_balloon[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x06,                   /* AIRBORNE <= 1G            */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0f,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x27, 0x41,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_15_marine[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x05,                   /* SEA                       */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0f,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x26, 0x1f,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_15_car[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x04,                   /* AUTOMOTIVE                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0f,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x25, 0xfd,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_15_pedestrian[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x03,                   /* PEDESTRIAN                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0f,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x24, 0xdb,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_15_stationary[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x02,                   /* STATIONARY                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0f,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x23, 0xb9,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_10_airborne[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x08,                   /* AIRBORNE <= 4G            */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0a,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x24, 0x0d,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_10_balloon[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x06,                   /* AIRBORNE <= 1G            */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0a,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x22, 0xc9,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_10_marine[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x05,                   /* SEA                       */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0a,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x21, 0xa7,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_10_car[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x04,                   /* AUTOMOTIVE                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0a,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x20, 0x85,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_10_pedestrian[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x03,                   /* PEDESTRIAN                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0a,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x1f, 0x63,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_10_stationary[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x02,                   /* STATIONARY                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x0a,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x1e, 0x41,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_5_airborne[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x08,                   /* AIRBORNE <= 4G            */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x05,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x1f, 0x95,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_5_balloon[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x06,                   /* AIRBORNE <= 1G            */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x05,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x1d, 0x51,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_5_marine[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x05,                   /* SEA                       */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x05,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x1c, 0x2f,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_5_car[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x04,                   /* AUTOMOTIVE                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x05,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x1b, 0x0d,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_5_pedestrian[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x03,                   /* PEDESTRIAN                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x05,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x1a, 0xeb,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_nav5_5_stationary[] = {
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x24,             /* CLASS, ID                 */
    0x24, 0x00,             /* LENGTH                    */
    0xff, 0x01,
    0x02,                   /* STATIONARY                */
    0x03,                   /* AUTO 2D/3D                */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE            */
    0x00, 0x00, 0x00, 0x00, /* FIXED ALTITUDE VARIANCE   */
    0x05,                   /* MIN ELEVATION             */
    0x00,                   /*                           */
    0xfa, 0x00,             /* PDOP                      */
    0xfa, 0x00,             /* TDOP                      */
    0x64, 0x00,             /* EPE                       */
    0x2c, 0x01,             /* ETE                       */
    0x00,                   /* STATIC HOLD THRESHOLD     */
    0x3c,                   /* DGPS TIMEOUT              */
    0x00,                   /* C/NO THRESHOLD NUMSV (7+) */
    0x00,                   /* C/NO THRESHOLD       (7+) */
    0x00, 0x00,             /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x00, 0x00, 0x00, 0x00, /*                           */
    0x19, 0xc9,             /* CK_A, CK_B                */
};

static const uint8_t ubx_cfg_tp5[] = { 
    0xb5, 0x62,             /* SYNC_CHAR_1, SYNC_CHAR_2  */
    0x06, 0x31,             /* CLASS, ID                 */
    0x20, 0x00,             /* LENGTH                    */
    0x00,                   /* TIMEPULSE                 */
    0x00, 0x00, 0x00,       /*                           */
    0x32, 0x00,             /* ANTENNA CABLE DELAY       */
    0x00, 0x00,             /* RF GROUP DELAY            */
    0x40, 0x42, 0x0f, 0x00, /* PERIOD                    */
    0x40, 0x42, 0x0f, 0x00, /* PERIOD LOCKED             */
    0x00, 0x00, 0x00, 0x00, /* PULSE LENGTH              */
    0x20, 0xa1, 0x07, 0x00, /* PULSE LENGTH LOCKED       */
    0x00, 0x00, 0x00, 0x00, /* USER DELAY                */
    0x77, 0x00, 0x00, 0x00, /* FLAGS (UTC, RAISING)      */
    0xea, 0xfc,             /* CK_A, CK_B                */
};

static const uint8_t *ubx_init_table[] = {
#if (GPS_CONFIG_UBLOX == 1)
    ubx_cfg_msg_nav_sol,
    ubx_cfg_msg_nav_posllh,
    ubx_cfg_msg_nav_velned,
    ubx_cfg_msg_nav_timeutc,
#else /* GPS_CONFIG_UBLOX == 1 */
    ubx_cfg_msg_nav_pvt,
    ubx_cfg_msg_nav_timegps,
#endif /* GPS_CONFIG_UBLOX == 1 */
    ubx_cfg_msg_nav_clock,
    ubx_cfg_msg_nav_dop,
#if (GPS_CONFIG_SATELLITES == 2)
    ubx_cfg_msg_nav_svinfo_1,
#elif (GPS_CONFIG_SATELLITES == 1)
#if (GPS_CONFIG_RATE >= 10)
    ubx_cfg_msg_nav_svinfo_10,
#elif (GPS_CONFIG_RATE >= 5)
    ubx_cfg_msg_nav_svinfo_5,
#else
    ubx_cfg_msg_nav_svinfo_1,
#endif
#else
    ubx_cfg_msg_nav_svinfo_disable,
#endif
    ubx_cfg_msg_nmea_gga,
    ubx_cfg_msg_nmea_gll,
    ubx_cfg_msg_nmea_gsa,
    ubx_cfg_msg_nmea_gsv,
    ubx_cfg_msg_nmea_rmc,
    ubx_cfg_msg_nmea_vtg,
#if (GPS_CONFIG_RATE >= 10)
    ubx_cfg_rate_10HZ,
#elif (GPS_CONFIG_RATE >= 5)
    ubx_cfg_rate_5HZ,
#else
    ubx_cfg_rate_1HZ,
#endif
#if (GPS_CONFIG_SBAS == 1)
    ubx_cfg_sbas_auto,
#elif (GPS_CONFIG_SBAS == 2)
    ubx_cfg_sbas_waas,
#elif (GPS_CONFIG_SBAS == 3)
    ubx_cfg_sbas_egnos,
#elif (GPS_CONFIG_SBAS == 4)
    ubx_cfg_sbas_msas,
#else
    ubx_cfg_sbas_disable,
#endif
#if (GPS_CONFIG_UBLOX == 2)
#if (GPS_CONFIG_QZSS == 1)    
#if (GPS_CONFIG_SBAS >= 1)    
    ubx_cfg_gnss_22ch_sbas_qzss,
#else /* GPS_CONFIG_SBAS >= 1 */    
    ubx_cfg_gnss_22ch_qzss,
#endif /* GPS_CONFIG_SBAS >= 1 */    
#else /* GPS_CONFIG_QZSS == 1 */    
#if (GPS_CONFIG_SBAS >= 1)    
    ubx_cfg_gnss_22ch_sbas,
#else /* GPS_CONFIG_SBAS >= 1 */    
    ubx_cfg_gnss_22ch,
#endif /* GPS_CONFIG_SBAS >= 1 */    
#endif /* GPS_CONFIG_QZSS == 1 */    
#endif /* GPS_CONFIG_UBLOX == 2 */
#if (GPS_CONFIG_UBLOX == 3)
#if (GPS_CONFIG_GLONASS == 1)    
#if (GPS_CONFIG_QZSS == 1)    
#if (GPS_CONFIG_SBAS >= 1)    
    ubx_cfg_gnss_32ch_sbas_qzss_glonass,
#else /* GPS_CONFIG_SBAS >= 1 */    
    ubx_cfg_gnss_32ch_qzss_glonass,
#endif /* GPS_CONFIG_SBAS >= 1 */    
#else /* GPS_CONFIG_QZSS == 1 */    
#if (GPS_CONFIG_SBAS >= 1)    
    ubx_cfg_gnss_32ch_sbas_glonass,
#else /* GPS_CONFIG_SBAS >= 1 */    
    ubx_cfg_gnss_32ch_glonass,
#endif /* GPS_CONFIG_SBAS >= 1 */    
#endif /* GPS_CONFIG_QZSS == 1 */    
#else /*GPS_CONFIG_GLONASS == 1 */   
#if (GPS_CONFIG_QZSS == 1)    
#if (GPS_CONFIG_SBAS >= 1)    
    ubx_cfg_gnss_32ch_sbas_qzss,
#else /* GPS_CONFIG_SBAS >= 1 */    
    ubx_cfg_gnss_32ch_qzss,
#endif /* GPS_CONFIG_SBAS >= 1 */    
#else /* GPS_CONFIG_QZSS == 1 */    
#if (GPS_CONFIG_SBAS >= 1)    
    ubx_cfg_gnss_32ch_sbas,
#else /* GPS_CONFIG_SBAS >= 1 */    
    ubx_cfg_gnss_32ch,
#endif /* GPS_CONFIG_SBAS >= 1 */    
#endif /* GPS_CONFIG_QZSS == 1 */    
#endif /*GPS_CONFIG_GLONASS == 1 */   
#endif /* GPS_CONFIG_UBLOX == 2 */
#if (GPS_CONFIG_ELEVATION >= 15)
#if (GPS_CONFIG_PLATFORM == 5)
    ubx_cfg_nav5_15_airborne,
#elif (GPS_CONFIG_PLATFORM == 4)
    ubx_cfg_nav5_15_balloon,
#elif (GPS_CONFIG_PLATFORM == 3)
    ubx_cfg_nav5_15_marine,
#elif (GPS_CONFIG_PLATFORM == 2)
    ubx_cfg_nav5_15_car,
#elif (GPS_CONFIG_PLATFORM == 1)
    ubx_cfg_nav5_15_pedestrian,
#else
    ubx_cfg_nav5_15_stationary,
#endif
#elif (GPS_CONFIG_ELEVATION >= 10)
#if (GPS_CONFIG_PLATFORM == 5)
    ubx_cfg_nav5_10_airborne,
#elif (GPS_CONFIG_PLATFORM == 4)
    ubx_cfg_nav5_10_balloon,
#elif (GPS_CONFIG_PLATFORM == 3)
    ubx_cfg_nav5_10_marine,
#elif (GPS_CONFIG_PLATFORM == 2)
    ubx_cfg_nav5_10_car,
#elif (GPS_CONFIG_PLATFORM == 1)
    ubx_cfg_nav5_10_pedestrian,
#else
    ubx_cfg_nav5_10_stationary,
#endif
#else
#if (GPS_CONFIG_PLATFORM == 5)
    ubx_cfg_nav5_5_airborne,
#elif (GPS_CONFIG_PLATFORM == 4)
    ubx_cfg_nav5_5_balloon,
#elif (GPS_CONFIG_PLATFORM == 3)
    ubx_cfg_nav5_5_marine,
#elif (GPS_CONFIG_PLATFORM == 2)
    ubx_cfg_nav5_5_car,
#elif (GPS_CONFIG_PLATFORM == 1)
    ubx_cfg_nav5_5_pedestrian,
#else
    ubx_cfg_nav5_5_stationary,
#endif
#endif
    ubx_cfg_tp5,
    NULL,
};

static void ubx_initialize(gps_device_t *device, unsigned int response, uint32_t command)
{
    if ((response == GPS_RESPONSE_NMEA_SENTENCE) || (response == GPS_RESPONSE_UBX_MESSAGE))
    {
	if (device->reset == GPS_RESET_HOT_START)
	{
	    ubx_send(device, ubx_cfg_rst_hot_start);

	    device->init = GPS_INIT_UBX_STARTUP;
	}
	else if (device->reset == GPS_RESET_WARM_START)
	{
	    ubx_send(device, ubx_cfg_rst_warm_start);

	    device->init = GPS_INIT_UBX_STARTUP;
	}
	else if (device->reset == GPS_RESET_COLD_START)
	{
	    ubx_send(device, ubx_cfg_rst_cold_start);

	    device->init = GPS_INIT_UBX_STARTUP;
	}
	else
	{
	    ubx_send(device, ubx_init_table[0]);

	    device->init = GPS_INIT_UBX_INIT_TABLE;
	}
    }
    else
    {
	if (device->init == GPS_INIT_UBX_STARTUP)
	{
	    if (response == GPS_RESPONSE_ACK)
	    {
		ubx_send(device, ubx_init_table[0]);

		device->init = GPS_INIT_UBX_INIT_TABLE;
	    }
	    else
	    {
		device->init = GPS_INIT_UBX_BAUD_RATE;
	    }
	}
	else
	{
	    if (response == GPS_RESPONSE_ACK)
	    {
		device->init++;

		if (ubx_init_table[device->init - GPS_INIT_UBX_INIT_TABLE])
		{
		    ubx_send(device, ubx_init_table[device->init - GPS_INIT_UBX_INIT_TABLE]);
		}
		else
		{
#if (GPS_CONFIG_SATELLITES >= 1)
		    if (device->satellites_callback == NULL)
		    {
			ubx_send(device, ubx_cfg_msg_nav_svinfo_disable);
		    }
#endif /* GPS_CONFIG_SATELLITES >= 1 */

		    device->init = GPS_INIT_DONE;
		    device->protocol = GPS_PROTOCOL_MASK_UBX;
#if (GPS_CONFIG_UBLOX == 1)
		    /* UBLOX6 */
		    device->expected = (UBX_MESSAGE_MASK_NAV_CLOCK |
					UBX_MESSAGE_MASK_NAV_DOP |
					UBX_MESSAGE_MASK_NAV_SOL |
					UBX_MESSAGE_MASK_NAV_POSLLH |
					UBX_MESSAGE_MASK_NAV_SVINFO |
					UBX_MESSAGE_MASK_NAV_TIMEUTC |
					UBX_MESSAGE_MASK_NAV_VELNED);
#else /* GPS_CONFIG_UBLOX == 1 */
		    /* UBLOX7, UBLOX8 */
		    device->expected = (UBX_MESSAGE_MASK_NAV_CLOCK |
					UBX_MESSAGE_MASK_NAV_DOP |
					UBX_MESSAGE_MASK_NAV_PVT |
					UBX_MESSAGE_MASK_NAV_SVINFO |
					UBX_MESSAGE_MASK_NAV_TIMEGPS);
#endif /* GPS_CONFIG_UBLOX == 1 */
		    device->seen = 0;
		    device->location.type = 0;
		    device->location.mask = 0;

		    (*device->ini_callback)(1);
		}
	    }
	    else
	    {
		device->init = GPS_INIT_UBX_BAUD_RATE;
	    }
	}
    }
}

#endif /* GPS_CONFIG_UBLOX >= 1 */

/************************************************************************************/

void gps_receive(uint8_t c)
{
    gps_device_t *device = &gps_device;

    if ((device->protocol & GPS_PROTOCOL_MASK_NMEA) && 
	(device->state <= GPS_STATE_NMEA_END_LF) &&
	(c == '$'))
    {
	/* Whenever we see a '$', it's the start of a new sentence,
	 * which can discard a partially read one.
	 */
	    
	device->state = GPS_STATE_NMEA_PAYLOAD;
	device->checksum = 0x00;
	device->count = 0;

	nmea_start_sentence(device);
    }
    else
    {
	switch (device->state) {
	case GPS_STATE_START:
#if (GPS_CONFIG_SIRF >= 1)
	    if ((device->protocol & GPS_PROTOCOL_MASK_SRF) && (c == 0xa0))
	    {
		device->state = GPS_STATE_SRF_START_2;
	    }
#endif /* GPS_CONFIG_SIRF >= 1 */

#if (GPS_CONFIG_UBLOX >= 1)
	    if ((device->protocol & GPS_PROTOCOL_MASK_UBX) && (c == 0xb5))
	    {
		device->state = GPS_STATE_UBX_SYNC_2;
	    }
#endif /* GPS_CONFIG_UBLOX >= 1 */
	    break;

	case GPS_STATE_NMEA_PAYLOAD:
	    if (c == '*')
	    {
		device->data[device->count] = '\0';

		nmea_parse_sentence(device, &device->data[0], device->count);

		device->state = GPS_STATE_NMEA_CHECKSUM_1;
	    }
	    else if ((c >= 0x20) && (c <= 0x7f))
	    {
		if (device->count >= GPS_DATA_SIZE)
		{
		    /* Reject a too long sentence.
		     */
		    device->state = GPS_STATE_START;
		}
		else
		{
		    device->checksum ^= c;

		    if (c == ',')
		    {
			device->data[device->count] = '\0';
			    
			nmea_parse_sentence(device, &device->data[0], device->count);

			device->count = 0;
		    }
		    else
		    {
			device->data[device->count++] = c;
		    }
		}
	    }
	    else
	    {
		/* If there is an illegal char, then scan again for a new start.
		 */
		device->state = GPS_STATE_START;
	    }
	    break;

	case GPS_STATE_NMEA_CHECKSUM_1:
	    if (c == nmea_hex_ascii[device->checksum >> 4])
	    {
		device->state = GPS_STATE_NMEA_CHECKSUM_2;
	    }
	    else
	    {
		/* If there is a checksum error, then scan again for a new start.
		 */

		device->state = GPS_STATE_START;
	    }
	    break;

	case GPS_STATE_NMEA_CHECKSUM_2:
	    if (c == nmea_hex_ascii[device->checksum & 0x0f])
	    {
		device->state = GPS_STATE_NMEA_END_CR;
	    }
	    else
	    {
		/* If there is a checksum error, then scan again for a new start.
		 */

		device->state = GPS_STATE_START;
	    }
	    break;

	case GPS_STATE_NMEA_END_CR:
	    if (c == '\r')
	    {
		device->state = GPS_STATE_NMEA_END_LF;
	    }
	    else
	    {
		/* If there is an illegal char, then scan again for a new start.
		 */

		device->state = GPS_STATE_START;
	    }
	    break;

	case GPS_STATE_NMEA_END_LF:
	    if (c == '\n')
	    {
		if (device->init != GPS_INIT_DONE)
		{
#if (GPS_CONFIG_MEDIATEK >= 1)
		    if (device->init == GPS_INIT_MTK_BAUD_RATE)
		    {
			mtk_initialize(device, GPS_RESPONSE_NMEA_SENTENCE, ~0l);
		    }
#endif /* GPS_CONFIG_MEDIATEK >= 1 */

#if (GPS_CONFIG_SIRF >= 1)
		    if (device->init == GPS_INIT_SRF_BAUD_RATE)
		    {
			srf_initialize(device, GPS_RESPONSE_NMEA_SENTENCE, ~0l);
		    }
#endif /* GPS_CONFIG_SIRF >= 1 */

#if (GPS_CONFIG_UBLOX >= 1)
		    if (device->init == GPS_INIT_UBX_BAUD_RATE)
		    {
			ubx_initialize(device, GPS_RESPONSE_NMEA_SENTENCE, ~0l);
		    }
#endif /* GPS_CONFIG_UBLOX >= 1 */
		}

		nmea_end_sentence(device);
	    }
		 
	    device->state = GPS_STATE_START;
	    break;

#if (GPS_CONFIG_SIRF >= 1)
	case GPS_STATE_SRF_START_2:
	    if (c != 0xa2)
	    {
		device->state = GPS_STATE_START;
	    }
	    else
	    {
		device->state = GPS_STATE_SRF_LENGTH_1;
	    }
	    break;
	    
	case GPS_STATE_SRF_LENGTH_1:
	    if (c > 0x7f)
	    {
		device->state = GPS_STATE_START;
	    }
	    else
	    {
		device->length = c << 8;
		device->state = GPS_STATE_SRF_LENGTH_2;
	    }
	    break;
	    
	case GPS_STATE_SRF_LENGTH_2:
	    device->length |= c;
	    device->checksum = 0;
	    device->count = 0;
	    device->first = 0;
	    device->last = ~0l;
	    device->state = GPS_STATE_SRF_MESSAGE;
	    break;
	    
	case GPS_STATE_SRF_MESSAGE:
	    device->checksum = (device->checksum + c) & 0x7fff;
	    device->message = c;
	    
	    device->data[device->count] = c;
	    device->count++;
	    
	    srf_start_message(device, device->message, device->length);
	    
	    if (device->count == device->length)
	    {
		device->state = GPS_STATE_SRF_CHECKSUM_1;
	    }
	    else
	    {
		device->state = GPS_STATE_SRF_PAYLOAD;
	    }
	    break;
	    
	case GPS_STATE_SRF_PAYLOAD:
	    device->checksum = (device->checksum + c) & 0x7fff;
	    
	    if ((device->count - device->first) < GPS_DATA_SIZE)
	    {
		device->data[device->count - device->first] = c;
	    }
	    
	    device->count++;
	    
	    if (device->count == device->last)
	    {
		srf_parse_message(device, device->message, &device->data[0], device->count);
	    }
	    
	    if (device->count == device->length)
	    {
		device->state = GPS_STATE_SRF_CHECKSUM_1;
	    }
	    break;
	    
	    
	case GPS_STATE_SRF_CHECKSUM_1:
	    if (c > 0x7f)
	    {
		device->state = GPS_STATE_START;
	    }
	    else
	    {
		device->checksum ^= (c << 8);
		device->state = GPS_STATE_SRF_CHECKSUM_2;
	    }
	    break;
	    
	case GPS_STATE_SRF_CHECKSUM_2:
	    device->checksum ^= c;
	    device->state = GPS_STATE_SRF_END_1;
	    break;
	    
	case GPS_STATE_SRF_END_1:
	    if (c != 0xb0)
	    {
		device->state = GPS_STATE_START;
	    }
	    else
	    {
		device->state = GPS_STATE_SRF_END_2;
	    }
	    break;
	    
	case GPS_STATE_SRF_END_2:
	    if ((c == 0xb3) && (device->checksum == 0x0000))
	    {
		if (device->init != GPS_INIT_DONE)
		{
		    if (device->init == GPS_INIT_SRF_BAUD_RATE)
		    {
			srf_initialize(device, GPS_RESPONSE_SRF_MESSAGE, ~0l);
		    }
		}
		
		if ((device->count - device->first) <= GPS_DATA_SIZE)
		{
		    srf_end_message(device, device->message, &device->data[0], device->count);
		}
	    }
	    
	    device->state = GPS_STATE_START;
	    break;
#endif /* GPS_CONFIG_SIRF >= 1 */

#if (GPS_CONFIG_UBLOX >= 1)
	case GPS_STATE_UBX_SYNC_2:
	    if (c != 0x62)
	    {
		device->state = GPS_STATE_START;
	    }
	    else
	    {
		device->state = GPS_STATE_UBX_MESSAGE_1;
	    }
	    break;
		
	case GPS_STATE_UBX_MESSAGE_1:
	    device->ck_a = c;
	    device->ck_b = c;
	    device->message = (c << 8);
	    device->state = GPS_STATE_UBX_MESSAGE_2;
	    break;
		
	case GPS_STATE_UBX_MESSAGE_2:
	    device->ck_a += c;
	    device->ck_b += device->ck_a;
	    device->message |= c;
	    device->state = GPS_STATE_UBX_LENGTH_1;
	    break;

	case GPS_STATE_UBX_LENGTH_1:
	    device->ck_a += c;
	    device->ck_b += device->ck_a;
	    device->length = c;
	    device->state = GPS_STATE_UBX_LENGTH_2;
	    break;
		
	case GPS_STATE_UBX_LENGTH_2:
	    device->ck_a += c;
	    device->ck_b += device->ck_a;
	    device->length |= (c << 8);
	    device->count = 0;
	    device->first = 0;
	    device->last = ~0l;

	    ubx_start_message(device, device->message, device->length);

	    if (device->count == device->length)
	    {
		device->state = GPS_STATE_UBX_CK_A;
	    }
	    else
	    {
		device->state = GPS_STATE_UBX_PAYLOAD;
	    }
	    break;
	    
	case GPS_STATE_UBX_PAYLOAD:
	    device->ck_a += c;
	    device->ck_b += device->ck_a;

	    if ((device->count - device->first) < GPS_DATA_SIZE)
	    {
		device->data[device->count - device->first] = c;
	    }
		
	    device->count++;
		
	    if (device->count == device->last)
	    {
		ubx_parse_message(device, device->message, &device->data[0], device->count);
	    }

	    if (device->count == device->length)
	    {
		device->state = GPS_STATE_UBX_CK_A;
	    }
	    break;

	case GPS_STATE_UBX_CK_A:
	    device->ck_a ^= c;
	    device->state = GPS_STATE_UBX_CK_B;
	    break;
		
	case GPS_STATE_UBX_CK_B:
	    device->ck_b ^= c;

	    if ((device->ck_a == 0x00) &&  (device->ck_b == 0x00))
	    {
		if (device->init != GPS_INIT_DONE)
		{
		    if (device->init == GPS_INIT_UBX_BAUD_RATE)
		    {
			ubx_initialize(device, GPS_RESPONSE_UBX_MESSAGE, ~0l);
		    }
		}

		if ((device->count - device->first) <= GPS_DATA_SIZE)
		{
		    ubx_end_message(device, device->message, &device->data[0], device->count);
		}
	    }
		
	    device->state = GPS_STATE_START;
	    break;
#endif /* GPS_CONFIG_UBLOX >= 1 */

	default:
	    break;
	}
    }
}

void gps_pps_callback(uint64_t tick, uint32_t period)
{
    gps_device_t *device = &gps_device;

    device->pps_pending = 1;
    device->pps_reference = tick;
    device->pps_period = period;
}

void gps_initialize(int reset, gps_ini_callback_t ini_callback, gps_location_callback_t location_callback, gps_satellites_callback_t satellites_callback)
{
    gps_device_t *device = &gps_device;

    device->reset = reset;
    device->ini_callback = ini_callback;
    device->location_callback = location_callback;
    device->satellites_callback = satellites_callback;

    device->state = GPS_STATE_START;
    device->command = -1;

    memset(&device->pps_time, 0, sizeof(device->pps_time));
    device->pps_correction = 0;

    memset(&device->location, 0, sizeof(device->location));
#if (GPS_CONFIG_SATELLITES >= 1)
    memset(&device->satellites, 0, sizeof(device->satellites));
#endif /* GPS_CONFIG_SATELLITES >= 1 */

#if (GPS_CONFIG_SIRF >= 1)

    device->protocol = GPS_PROTOCOL_MASK_SRF;
    device->init = GPS_INIT_SRF_BAUD_RATE;

    gps_configure(device, 4800);

#if (GPS_CONFIG_SPEED >= 115200)
    nmea_send(device, "$PSRF100,0,115200,8,1,0*04\r\n");
#elif (GPS_CONFIG_SPEED >= 57600)
    nmea_send(device, "$PSRF100,0,57600,8,1,0*37\r\n");
#elif (GPS_CONFIG_SPEED >= 38400)
    nmea_send(device, "$PSRF100,0,38400,8,1,0*3C\r\n");
#elif (GPS_CONFIG_SPEED >= 19200)
    nmea_send(device, "$PSRF100,0,19200,8,1,0*39\r\n");
#elif (GPS_CONFIG_SPEED >= 9600)
    nmea_send(device, "$PSRF100,0,9600,8,1,0*0C\r\n");
#else
    nmea_send(device, "$PSRF100,0,4800,8,1,0*0F\r\n");
#endif

#endif /* GPS_CONFIG_SIRF >= 1 */

#if (GPS_CONFIG_UBLOX >= 1)

    device->protocol = GPS_PROTOCOL_MASK_NMEA | GPS_PROTOCOL_MASK_UBX;
    device->init = GPS_INIT_UBX_BAUD_RATE;

    gps_configure(device, 9600);

#if (GPS_CONFIG_SPEED >= 115200)
    nmea_send(device, "$PUBX,41,1,0007,0003,115200,0*18\r\n");
#elif (GPS_CONFIG_SPEED >= 57600)
    nmea_send(device, "$PUBX,41,1,0007,0003,57600,0*2B\r\n");
#elif (GPS_CONFIG_SPEED >= 38400)
    nmea_send(device, "$PUBX,41,1,0007,0003,38400,0*20\r\n");
#elif (GPS_CONFIG_SPEED >= 19200)
    nmea_send(device, "$PUBX,41,1,0007,0003,19200,0*25\r\n");
#elif (GPS_CONFIG_SPEED >= 9600)
    nmea_send(device, "$PUBX,41,1,0007,0003,9600,0*10\r\n");
#else
    nmea_send(device, "$PUBX,41,1,0007,0003,4800,0*13\r\n");
#endif
    
#endif /* GPS_CONFIG_UBLOX >= 1 */

#if (GPS_CONFIG_SIRF == 0) && (GPS_CONFIG_UBLOX == 0)
#if (GPS_CONFIG_MEDIATEK >= 1)

    device->protocol = GPS_PROTOCOL_MASK_NMEA;
    device->init = GPS_INIT_MTK_BAUD_RATE;

    gps_configure(device, 9600);

#if (GPS_CONFIG_SPEED >= 115200)
    nmea_send(device, "$PMTK251,115200*1F\r\n");
#elif (GPS_CONFIG_SPEED >= 57600)
    nmea_send(device, "$PMTK251,57600*2C\r\n");
#elif (GPS_CONFIG_SPEED >= 38400)
    nmea_send(device, "$PMTK251,38400*27\r\n");
#elif (GPS_CONFIG_SPEED >= 19200)
    nmea_send(device, "$PMTK251,19200*22\r\n");
#elif (GPS_CONFIG_SPEED >= 9600)
    nmea_send(device, "$PMTK251,9600*17\r\n");
#else
    nmea_send(device, "$PMTK251,4800*14\r\n");
#endif

#else /* GPS_CONFIG_MEDIATEK >= 1 */

    device->protocol = GPS_PROTOCOL_MASK_NMEA;
    device->init = GPS_INIT_DONE;
    device->expected = NMEA_SENTENCE_MASK_GPGGA | NMEA_SENTENCE_MASK_GPGSA | NMEA_SENTENCE_MASK_GPGSV | NMEA_SENTENCE_MASK_GPRMC;

    (*device->ini_callback)(1);

#endif /* GPS_CONFIG_MEDIATEK >= 1 */
#endif /* GPS_CONFIG_SIRF == 0 && GPS_CONFIG_UBLOX == 0 */

    gps_configure(device, GPS_CONFIG_SPEED);
}
