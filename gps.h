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

#if !defined(_GPS_H)
#define _GPS_H

#if (KITTY_CONFIG_ALTERNATIVE == 0)
#define GPS_CONFIG_UBLOX                      2      /* 0, 1 (UBLOX6), 2 (UBLOX7), 3 (UBLOX8)                                         */
#define GPS_CONFIG_GLONASS                    0      /* 0, 1                                                                          */
#else /* KITTY_CONFIG_ALTERNATIVE == 0 */
#define GPS_CONFIG_UBLOX                      3      /* 0, 1 (UBLOX6), 2 (UBLOX7), 3 (UBLOX8)                                         */
#define GPS_CONFIG_GLONASS                    1      /* 0, 1                                                                          */
#endif /* KITTY_CONFIG_ALTERNATIVE == 0 */

#define GPS_CONFIG_MEDIATEK                   0      /* 0, 1 (MT3329), 2 (MT3339), 3 (MT3333)                                         */
#define GPS_CONFIG_SIRF                       0      /* 0, 1 (SIRF3), 2 (SIRF4)                                                       */
#define GPS_CONFIG_SPEED                      115200 /* 4800, 9600, 19200, 38400, 57600, 115200                                       */
#define GPS_CONFIG_RATE                       10     /* 1 (1Hz), 5 (5Hz), 10 (10Hz)                                                   */
#define GPS_CONFIG_SBAS                       2      /* 0, 1 (AUTO), 2 (WASS), 3 (EGNOS), 4 (MSAS)                                    */
#define GPS_CONFIG_QZSS                       0      /* 0, 1                                                                          */
#define GPS_CONFIG_ELEVATION                  10     /* 5, 10, 15                                                                     */
#define GPS_CONFIG_PLATFORM                   2      /* 0 (STATIONARY), 1 (PEDESTRIAN), 2 (CAR), 3 (MARINE), 4 (BALLON), 5 (AIRBORNE) */
#define GPS_CONFIG_SATELLITES                 1      /* 0 (DISABLE), 1 (1Hz), 2 (@ LOCATION RATE)                                     */

#define GPS_RESET_NONE                        0
#define GPS_RESET_HOT_START                   1
#define GPS_RESET_WARM_START                  2
#define GPS_RESET_COLD_START                  3

#define GPS_LOCATION_TYPE_NONE                0
#define GPS_LOCATION_TYPE_TIME                1
#define GPS_LOCATION_TYPE_2D                  2
#define GPS_LOCATION_TYPE_3D                  3

#define GPS_LOCATION_QUALITY_NONE             0
#define GPS_LOCATION_QUALITY_AUTONOMOUS       1
#define GPS_LOCATION_QUALITY_DIFFERENTIAL     2
#define GPS_LOCATION_QUALITY_PRECISE          3
#define GPS_LOCATION_QUALITY_RTK_INTEGER      4
#define GPS_LOCATION_QUALITY_RTK_FLOAT        5
#define GPS_LOCATION_QUALITY_ESTIMATED        6
#define GPS_LOCATION_QUALITY_MANUAL           7
#define GPS_LOCATION_QUALITY_SIMULATION       8

typedef struct _utc_time_t {
    uint16_t       year;             /* 1 .. 9999               */
    uint8_t        month;            /* 1 .. 12                 */
    uint8_t        day;              /* 1 .. 31                 */
    uint8_t        hour;             /* 0 .. 23                 */
    uint8_t        min;              /* 0 .. 59                 */
    uint16_t       sec;              /* 0 .. 60, 1e3            */ 
} utc_time_t;

#define GPS_LOCATION_MASK_TIME           0x0001
#define GPS_LOCATION_MASK_CORRECTION     0x0002
#define GPS_LOCATION_MASK_PPS            0x0004
#define GPS_LOCATION_MASK_POSITION       0x0008
#define GPS_LOCATION_MASK_ALTITUDE       0x0010
#define GPS_LOCATION_MASK_SPEED          0x0020
#define GPS_LOCATION_MASK_COURSE         0x0040
#define GPS_LOCATION_MASK_CLIMB          0x0080
#define GPS_LOCATION_MASK_CLOCK          0x0100
#define GPS_LOCATION_MASK_EHPE           0x0200
#define GPS_LOCATION_MASK_EVPE           0x0400
#define GPS_LOCATION_MASK_HDOP           0x0800
#define GPS_LOCATION_MASK_VDOP           0x1000
#define GPS_LOCATION_MASK_TDOP           0x2000
#define GPS_LOCATION_MASK_ESVE           0x4000
#define GPS_LOCATION_MASK_ECVE           0x8000

typedef struct _gps_location_t {
    utc_time_t     time;             /* UTC date/time           */
    uint16_t       mask;             /*                         */
    uint8_t        correction;       /* GPS/UTC offset          */
    uint8_t        type;             /* fix type                */
    int32_t        latitude;         /* (WGS84) degrees, 1e7    */
    int32_t        longitude;        /* (WGS84) degrees, 1e7    */
    int32_t        altitude;         /* (WGS84) m, 1e3          */
    uint32_t       speed;            /* m/s, 1e3                */
    uint32_t       course;           /* degrees, 1e5            */
    int32_t        climb;            /* m/s, 1e3                */
    int32_t        bias;             /* s, 1e9                  */
    int32_t        drift;            /* s/s, 1e9                */
    uint32_t       ehpe;             /* m, 1e3                  */
    uint32_t       evpe;             /* m, 1e3                  */
    uint8_t        quality;          /* fix quality             */
    uint8_t        numsv;            /* fix numsv               */
    uint16_t       hdop;             /* 1e2                     */
    uint16_t       vdop;             /* 1e2                     */
    uint16_t       tdop;             /* 1e2                     */
    uint32_t       esve;             /* m/s, 1e3                */
    uint32_t       ecve;             /* degress/s, 1e3          */
} gps_location_t;

#define GPS_SATELLITES_STATE_CODE_LOCK          0x01
#define GPS_SATELLITES_STATE_CARRIER_LOCK       0x02
#define GPS_SATELLITES_STATE_UNHEALTHY          0x04
#define GPS_SATELLITES_STATE_CORRECTION         0x08
#define GPS_SATELLITES_STATE_ALMANAC            0x10
#define GPS_SATELLITES_STATE_EPHEMERIS          0x20
#define GPS_SATELLITES_STATE_TRACKING           0x40
#define GPS_SATELLITES_STATE_NAVIGATING         0x80

#if 0
#if (GPS_CONFIG_GLONASS == 1)
#define GPS_SATELLITES_COUNT_MAX          32
#else
#define GPS_SATELLITES_COUNT_MAX          22
#endif
#endif

#define GPS_SATELLITES_COUNT_MAX          22

/* PRN ranges:
 *
 * 0       none
 * 1-32    GPS
 * 33-64   SBAS (+87)
 * 65-96   GLONASS (-64)
 * 193-200 QZSS
 * 201-237 BEIDOU (-200)
 * 255     GLONASS
 */

typedef struct _gps_satellites_t {
    utc_time_t     time;
    uint16_t       mask;
    uint8_t        correction;
    uint8_t        count;
    struct {
	uint8_t    id;
        uint8_t    elev;
        uint16_t   azim;
        uint8_t    snr;
        uint8_t    state;
    }              info[GPS_SATELLITES_COUNT_MAX];
} gps_satellites_t;

typedef void (*gps_ini_callback_t)(int success);
typedef void (*gps_location_callback_t)(uint64_t tick, const gps_location_t *location);
typedef void (*gps_satellites_callback_t)(uint64_t tick, const gps_satellites_t *satellites);

void gps_initialize(int reset, gps_ini_callback_t ini_callback, gps_location_callback_t location_callback, gps_satellites_callback_t satellites_callback);
void gps_receive(uint8_t c);
void gps_pps_callback(uint64_t tick, uint32_t period);

#endif /* _GPS_H */
