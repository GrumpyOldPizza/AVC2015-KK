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

#if !defined(_NAVIGATION_H)
#define _NAVIGATION_H

typedef struct _navigation_sensors_t {
    uint64_t         rpm_tick;
    float            rpm_delta;
    uint64_t         gps_tick;
    gps_location_t   gps_location;
    uint64_t         accel_tick;
    float            accel_data[3];
    uint64_t         gyro_tick;
    float            gyro_data[3];
    uint64_t         mag_tick;
    float            mag_data[3];
    float            mag_heading;
} navigation_sensors_t;

extern volatile navigation_sensors_t navigation_sensors;

typedef struct _navigation_location_t {
    uint64_t         tick;
    float            x;          /* m      */
    float            y;          /* m      */
    float            speed;      /* m/s    */
    float            course;     /* rad    */
    float            rpm_scale;  /* m/s    */
    float            gyro_scale; /* rad    */
    float            gyro_bias;  /* rad    */
} navigation_location_t;

typedef struct _navigation_record_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    navigation_location_t location;
} navigation_record_entry_t;

typedef struct _gps_fifo_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    gps_location_t    location;
} gps_location_entry_t;

typedef struct _gps_satellites_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    gps_satellites_t  satellites;
} gps_satellites_entry_t;

extern void gps_location_callback(uint64_t tick, const gps_location_t *location);
extern void gps_satellites_callback(uint64_t tick, const gps_satellites_t *satellites);

extern void navigation_pps_notify(uint64_t tick, uint32_t period);
extern void navigation_rpm_notify(uint64_t tick);
extern void navigation_gps_notify(uint64_t tick, const gps_location_t *location);
extern void navigation_accel_notify(uint64_t tick, float ax, float ay, float az);
extern void navigation_gyro_notify(uint64_t tick, float gx, float gy, float gz);
extern void navigation_mag_notify(uint64_t tick, float mx, float my, float mz);

extern int navigation_active(void);
extern int navigation_location(uint64_t tick, float *x, float *y, float *speed, float *course);

extern void navigation_initialize(void);


typedef struct _lla_home_t {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    float   S[3];
} lla_home_t;

extern void LLA2HOME(int32_t latitude, int32_t longitude, int32_t altitude, lla_home_t *home );
extern void LLA2NED(int32_t latitude, int32_t longitude, int32_t altitude, const lla_home_t *home, float ned[3] );

#endif /* _NAVIGATION_H */

