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

#if !defined(_EKF_CORE_H)
#define _EKF_CORE_H

typedef struct _ekf_state_t {
    float         x;          /* m      */
    float         y;          /* m      */
    float         speed;      /* m/s    */
    float         course;     /* rad    */
    float         rpm_scale;  /* m/s    */
    float         gyro_scale; /* rad    */
    float         gyro_bias;  /* rad    */
} ekf_state_t;

#define EKF_EVENT_INITIALIZE  0
#define EKF_EVENT_CORRECT     1
#define EKF_EVENT_PREDICT     2

typedef struct _ekf_record_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    ekf_state_t       state;
} ekf_record_entry_t;

extern void ekf_initialize(float x, float y, float speed, float course, float rpm_scale, float gyro_scale, float gyro_bias, ekf_state_t *state);
extern void ekf_predict(float rpm_speed, float gyro_rate, ekf_state_t *state);
extern void ekf_correct(float x, float y, float speed, float course, ekf_state_t *state);

#endif /* _EKF_CORE_H */

