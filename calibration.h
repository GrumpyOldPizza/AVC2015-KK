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

#if !defined(_CALIBRATION_H)
#define _CALIBRATION_H

typedef struct _calibration_t {
    float rpm_scale;
    float gyro_scale;
    float gyro_bias;
    float accel_bias[3];
    float accel_scale[3];
    float mag_bias[3];
    float mag_matrix[3][3];
    float steering_center;        /* 1456           */
    float steering_range_min;     /* 200            */
    float steering_range_max;     /* 400            */
    float steering_speed_min;     /* 4.0            */
    float steering_speed_max;     /* 8.0            */
    float steering_gain;          /* 600            */
    float throttle_min;           /* 1525           */
    float throttle_max;           /* 1975           */
    float throttle_limit;         /* integral clamp */
    float throttle_slope;         /* 2, max delta   */
    float throttle_constants[3];  /* Kp, Ki, Kd     */
} calibration_t;

extern calibration_t calibration;

#endif /* _CALIBRATION_H */

