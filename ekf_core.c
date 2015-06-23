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

#if defined(SIMULATION)

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "constants.h"
#include "ekf_core.h"
#include "ekf_math.h"

#else

#include "kitty.h"

#endif

#if defined(SIMULATION)
ekf_state_t ekf_state;
#endif

static int ekf_ready = 0;
static float ekf_gyro_scale = 1.0;

#define SIG_RPM_SPEED      2.0                /* m/sec         */
#define SIG_GYRO_RATE      (DEG2RAD * 0.06)   /* deg/sec       */
#define SIG_GYRO_BIAS      (DEG2RAD * 0.005)  /* deg/sec / Hz  */

#define SIG_GPS_POSITION   3.0                /* m             */
#define SIG_GPS_SPEED      0.3                /* m/sec         */
#define SIG_GPS_COURSE     (DEG2RAD * 5.0)    /* degree        */

#define	P_INIT_POSITION    10.0
#define	P_INIT_SPEED       1.0
#define P_INIT_COURSE      (DEG2RAD * 180.0)
#define	P_INIT_RPM_SCALE   1.0
#define	P_INIT_GYRO_BIAS   (DEG2RAD * 5.0)

#define X_STATE_X          MAT_ELEMENT(X,0,0)
#define X_STATE_Y          MAT_ELEMENT(X,1,0)
#define X_STATE_SPEED      MAT_ELEMENT(X,2,0)
#define X_STATE_COURSE     MAT_ELEMENT(X,3,0)
#define X_STATE_RPM_SCALE  MAT_ELEMENT(X,4,0)
#define X_STATE_GYRO_BIAS  MAT_ELEMENT(X,5,0)

MAT_CREATE_STATIC(X, 6, 1);		// States
MAT_CREATE_STATIC(F, 6, 6);		// State Matrix
MAT_CREATE_STATIC(P, 6, 6);		// Covariance Matrix
MAT_CREATE_STATIC(G, 6, 3);
MAT_CREATE_STATIC(Q, 3, 3);

MAT_CREATE_STATIC(K, 6, 4);
MAT_CREATE_STATIC(H, 4, 6);

MAT_CREATE_STATIC(R, 4, 4);

MAT_CREATE_STATIC(Y, 4, 1);

MAT_CREATE_STATIC(I6, 6, 6);

MAT_CREATE_STATIC(temp1_6x6, 6, 6);
MAT_CREATE_STATIC(temp2_6x6, 6, 6);
MAT_CREATE_STATIC(temp3_6x3, 6, 3);

MAT_CREATE_STATIC(temp4_6x4, 6, 4);
MAT_CREATE_STATIC(temp5_4x4, 4, 4);
MAT_CREATE_STATIC(temp6_4x4, 4, 4);

MAT_CREATE_STATIC(temp7_6x1, 6, 1);
MAT_CREATE_STATIC(temp8_6x1, 6, 1);

void ekf_initialize(float x, float y, float speed, float course, float rpm_scale, float gyro_scale, float gyro_bias, ekf_state_t *state)
{
    ekf_state_t payload;

    ekf_ready = 1;
    ekf_gyro_scale = gyro_scale;

    X_STATE_X          = x;
    X_STATE_Y          = y;
    X_STATE_SPEED      = speed;
    X_STATE_COURSE     = course;
    X_STATE_RPM_SCALE  = rpm_scale;
    X_STATE_GYRO_BIAS  = gyro_bias;

    mat_init_zero(P);
    MAT_ELEMENT(P,0,0) = P_INIT_POSITION;
    MAT_ELEMENT(P,1,1) = P_INIT_POSITION;
    MAT_ELEMENT(P,2,2) = P_INIT_SPEED * P_INIT_SPEED;
    MAT_ELEMENT(P,3,3) = P_INIT_COURSE * P_INIT_COURSE;
    MAT_ELEMENT(P,4,4) = P_INIT_RPM_SCALE * P_INIT_RPM_SCALE;
    MAT_ELEMENT(P,5,5) = P_INIT_GYRO_BIAS * P_INIT_GYRO_BIAS;

    mat_init_zero(Q);
    MAT_ELEMENT(Q,0,0) = SIG_RPM_SPEED * SIG_RPM_SPEED;
    MAT_ELEMENT(Q,1,1) = SIG_GYRO_RATE * SIG_GYRO_RATE;
    MAT_ELEMENT(Q,2,2) = SIG_GYRO_BIAS * SIG_GYRO_BIAS;

    mat_init_zero(R);
    MAT_ELEMENT(R,0,0) = SIG_GPS_POSITION * SIG_GPS_POSITION;
    MAT_ELEMENT(R,1,1) = SIG_GPS_POSITION * SIG_GPS_POSITION;
    MAT_ELEMENT(R,2,2) = SIG_GPS_SPEED * SIG_GPS_SPEED;
    MAT_ELEMENT(R,3,3) = SIG_GPS_COURSE * SIG_GPS_COURSE;

    mat_init_zero(H);
    MAT_ELEMENT(H,0,0) = 1.0;
    MAT_ELEMENT(H,1,1) = 1.0;
    MAT_ELEMENT(H,2,2) = 1.0;
    MAT_ELEMENT(H,3,3) = 1.0;

    mat_init_identity(I6);

#if defined(SIMULATION)
    {
	extern int doSimulation;
      
	if (doSimulation)
	{
	    printf("EKF_INITIALIZE(x=%f, y=%f, speed=%f, course=%f, rpm_scale=%f, gyro_scale=%f, gyro_bias=%f)\n", x, y, speed, course, rpm_scale, gyro_scale, gyro_bias);
	}
    }
#endif

    if (!state)
    {
	state = &payload;
    }

    state->x          = X_STATE_X;
    state->y          = X_STATE_Y;
    state->speed      = X_STATE_SPEED;
    state->course     = X_STATE_COURSE;
    state->rpm_scale  = X_STATE_RPM_SCALE;
    state->gyro_scale = ekf_gyro_scale;
    state->gyro_bias  = X_STATE_GYRO_BIAS;

#if !defined(SIMULATION)
    record_enter_extended(RECORD_TYPE_EKF, EKF_EVENT_INITIALIZE, tm4c123_capture_clock(), state, sizeof(ekf_state_t));
#endif

#if defined(SIMULATION)
    ekf_state.x          = X_STATE_X;
    ekf_state.y          = X_STATE_Y;
    ekf_state.speed      = X_STATE_SPEED;
    ekf_state.course     = X_STATE_COURSE;
    ekf_state.rpm_scale  = X_STATE_RPM_SCALE;
    ekf_state.gyro_scale = ekf_gyro_scale;
    ekf_state.gyro_bias  = X_STATE_GYRO_BIAS;
#endif
}

void ekf_predict(float rpm_speed, float gyro_rate, ekf_state_t *state)
{
    ekf_state_t payload;
    float dT, S0, S1;

    if (!ekf_ready)
    {
	return;
    }

    dT = 0.01;

    S0 = X_STATE_RPM_SCALE * rpm_speed * dT;
    S1 = X_STATE_COURSE + 0.5 * ekf_gyro_scale * (gyro_rate - X_STATE_GYRO_BIAS) * dT;

    MAT_ELEMENT(F,0,0) = 1.0;
    MAT_ELEMENT(F,0,1) = 0.0;
    MAT_ELEMENT(F,0,2) = 0.0;
    MAT_ELEMENT(F,0,3) = - S0 * sinf(S1);
    MAT_ELEMENT(F,0,4) = rpm_speed * dT * cosf(S1);
    MAT_ELEMENT(F,0,5) = S0 * sinf(S1) * (0.5 * dT);

    MAT_ELEMENT(F,1,0) = 0.0;
    MAT_ELEMENT(F,1,1) = 1.0;
    MAT_ELEMENT(F,1,2) = 0.0;
    MAT_ELEMENT(F,1,3) = S0 * cosf(S1);
    MAT_ELEMENT(F,1,4) = rpm_speed * dT * sinf(S1);
    MAT_ELEMENT(F,1,5) = - S0 * cosf(S1) * (0.5 * dT);

    MAT_ELEMENT(F,2,0) = 0.0;
    MAT_ELEMENT(F,2,1) = 0.0;
    MAT_ELEMENT(F,2,2) = 0.0;
    MAT_ELEMENT(F,2,3) = 0.0;
    MAT_ELEMENT(F,2,4) = rpm_speed;
    MAT_ELEMENT(F,2,5) = 0.0;

    MAT_ELEMENT(F,3,0) = 0.0;
    MAT_ELEMENT(F,3,1) = 0.0;
    MAT_ELEMENT(F,3,2) = 0.0;
    MAT_ELEMENT(F,3,3) = 1.0;
    MAT_ELEMENT(F,3,4) = 0.0;
    MAT_ELEMENT(F,3,5) = - dT;

    MAT_ELEMENT(F,4,0) = 0.0;
    MAT_ELEMENT(F,4,1) = 0.0;
    MAT_ELEMENT(F,4,2) = 0.0;
    MAT_ELEMENT(F,4,3) = 0.0;
    MAT_ELEMENT(F,4,4) = 1.0;
    MAT_ELEMENT(F,4,5) = 0.0;

    MAT_ELEMENT(F,5,0) = 0.0;
    MAT_ELEMENT(F,5,1) = 0.0;
    MAT_ELEMENT(F,5,2) = 0.0;
    MAT_ELEMENT(F,5,3) = 0.0;
    MAT_ELEMENT(F,5,4) = 0.0;
    MAT_ELEMENT(F,5,5) = 1.0;

    MAT_ELEMENT(G,0,0) = X_STATE_RPM_SCALE * dT * cosf(S1);
    MAT_ELEMENT(G,0,1) = - S0 * sinf(S1) * (0.5 * dT);
    MAT_ELEMENT(G,0,2) = 0.0;

    MAT_ELEMENT(G,1,0) = X_STATE_RPM_SCALE * dT * sinf(S1);
    MAT_ELEMENT(G,1,1) = S0 * cosf(S1) * (0.5 * dT);
    MAT_ELEMENT(G,1,2) = 0.0;

    MAT_ELEMENT(G,2,0) = X_STATE_RPM_SCALE;
    MAT_ELEMENT(G,2,1) = 0.0;
    MAT_ELEMENT(G,2,2) = 0.0;

    MAT_ELEMENT(G,3,0) = 0.0;
    MAT_ELEMENT(G,3,1) = dT;
    MAT_ELEMENT(G,3,2) = 0.0;
    
    MAT_ELEMENT(G,4,0) = 0.0;
    MAT_ELEMENT(G,4,1) = 0.0;
    MAT_ELEMENT(G,4,2) = 0.0;

    MAT_ELEMENT(G,5,0) = 0.0;
    MAT_ELEMENT(G,5,1) = 0.0;
    MAT_ELEMENT(G,5,2) = 1.0;

    X_STATE_X      = X_STATE_X + S0 * cosf(S1);
    X_STATE_Y      = X_STATE_Y + S0 * sinf(S1);
    X_STATE_SPEED  = X_STATE_RPM_SCALE  * rpm_speed;
    X_STATE_COURSE = X_STATE_COURSE + ekf_gyro_scale * (gyro_rate - X_STATE_GYRO_BIAS) * dT;

    X_STATE_COURSE = angle_normalize(X_STATE_COURSE);

    mat_mul(F, P, temp1_6x6);                     /* F*P             */
    mat_mul_transpose(temp1_6x6, F, temp2_6x6);   /* F*P*F'          */
    mat_mul(G, Q, temp3_6x3);                     /* G*Q             */
    mat_mul_transpose(temp3_6x3, G, temp1_6x6);   /* G*Q*G'          */
    mat_add(temp2_6x6, temp1_6x6, P);             /* F*P*F' + G*Q*G' */

    mat_copy_transpose(P, temp1_6x6);
    mat_add(P, temp1_6x6, temp2_6x6);
    mat_scale(temp2_6x6, 0.5, P);

#if defined(SIMULATION)
    {
	extern int doSimulation;
      
	if (doSimulation)
	{
	    printf("EKF_PREDICT(rpm_speed=%f, gyro_rate=%f) = (x=%f, y=%f, speed=%f, course=%f, rpm_scale=%f, gyro_scale=%f, gyro_bias=%f)\n", 
		   rpm_speed, gyro_rate, X_STATE_X, X_STATE_Y, X_STATE_SPEED, (X_STATE_COURSE * RAD2DEG), X_STATE_RPM_SCALE, ekf_gyro_scale, X_STATE_GYRO_BIAS);
	}
    }
#endif

    if (!state)
    {
	state = &payload;
    }

    state->x          = X_STATE_X;
    state->y          = X_STATE_Y;
    state->speed      = X_STATE_SPEED;
    state->course     = X_STATE_COURSE;
    state->rpm_scale  = X_STATE_RPM_SCALE;
    state->gyro_scale = ekf_gyro_scale;
    state->gyro_bias  = X_STATE_GYRO_BIAS;

#if !defined(SIMULATION)
    record_enter_extended(RECORD_TYPE_EKF, EKF_EVENT_PREDICT, tm4c123_capture_clock(), state, sizeof(ekf_state_t));
#endif

#if defined(SIMULATION)
    ekf_state.x          = X_STATE_X;
    ekf_state.y          = X_STATE_Y;
    ekf_state.speed      = X_STATE_SPEED;
    ekf_state.course     = X_STATE_COURSE;
    ekf_state.rpm_scale  = X_STATE_RPM_SCALE;
    ekf_state.gyro_scale = ekf_gyro_scale;
    ekf_state.gyro_bias  = X_STATE_GYRO_BIAS;
#endif
}

void ekf_correct(float x, float y, float speed, float course, ekf_state_t *state)
{
    ekf_state_t payload;
    float angle;

    if (!ekf_ready)
    {
	return;
    }

    mat_mul_transpose(P, H, temp4_6x4);      /* P*H'               */
    mat_mul(H, temp4_6x4, temp5_4x4);        /* H*P*H'             */
    mat_add(temp5_4x4, R, temp6_4x4);        /* H*P*H'+R           */
    mat_inverse(temp6_4x4, temp5_4x4);       /* (H*P*H'+R)^-1      */
    mat_mul(temp4_6x4, temp5_4x4, K);        /* P*H'*(H*P*H'+R)^-1 */

    angle = angle_difference(course, X_STATE_COURSE);

    MAT_ELEMENT(Y,0,0) = x - X_STATE_X;
    MAT_ELEMENT(Y,1,0) = y - X_STATE_Y;
    MAT_ELEMENT(Y,2,0) = speed - X_STATE_SPEED;
    MAT_ELEMENT(Y,3,0) = angle;

    mat_mul(K, Y, temp7_6x1);                /* K*Y       */
    mat_add(X, temp7_6x1, temp8_6x1);        /* X+K*Y     */
    mat_copy(temp8_6x1, X);

    X_STATE_COURSE = angle_normalize(X_STATE_COURSE);

    mat_mul(K, H, temp1_6x6);              /* K*H       */
    mat_sub(I6, temp1_6x6, temp2_6x6);     /* I-K*H     */
    mat_mul(temp2_6x6, P, temp1_6x6);      /* (I-K*H)*P */
    mat_copy(temp1_6x6, P);

#if defined(SIMULATION)
    {
	extern int doSimulation;
      
	if (doSimulation)
	{
	    float distance;

	    static float d_accum = 0;
	    static int d_count = 0;

	    distance = sqrtf((x - X_STATE_X) * (x - X_STATE_X) + (y - X_STATE_Y) * (y - X_STATE_Y));

	    d_accum += (distance * distance);
	    d_count++;

	    printf("EKF_CORRECT(x=%f, y=%f, speed=%f, course=%f) = (distance=%f (%f), angle=%f, x=%f, y=%f, speed=%f, course=%f, rpm_scale=%f, gyro_scale=%f, gyro_bias=%f)\n",
		   x, y, speed, course,
		   distance, sqrtf(d_accum / d_count), (angle * RAD2DEG),
		   X_STATE_X, X_STATE_Y, X_STATE_SPEED, (X_STATE_COURSE * RAD2DEG), X_STATE_RPM_SCALE, ekf_gyro_scale, X_STATE_GYRO_BIAS);
	}
    }
#endif

    if (!state)
    {
	state = &payload;
    }

    state->x          = X_STATE_X;
    state->y          = X_STATE_Y;
    state->speed      = X_STATE_SPEED;
    state->course     = X_STATE_COURSE;
    state->rpm_scale  = X_STATE_RPM_SCALE;
    state->gyro_scale = ekf_gyro_scale;
    state->gyro_bias  = X_STATE_GYRO_BIAS;

#if !defined(SIMULATION)
    record_enter_extended(RECORD_TYPE_EKF, EKF_EVENT_CORRECT, tm4c123_capture_clock(), state, sizeof(ekf_state_t));
#endif

#if defined(SIMULATION)
    ekf_state.x          = X_STATE_X;
    ekf_state.y          = X_STATE_Y;
    ekf_state.speed      = X_STATE_SPEED;
    ekf_state.course     = X_STATE_COURSE;
    ekf_state.rpm_scale  = X_STATE_RPM_SCALE;
    ekf_state.gyro_scale = ekf_gyro_scale;
    ekf_state.gyro_bias  = X_STATE_GYRO_BIAS;
#endif
}
