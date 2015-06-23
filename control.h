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

#if !defined(_CONTROL_H)
#define _CONTROL_H

typedef struct _control_state_t {
    uint32_t          status;
    float             x;
    float             y;
    float             speed;
    float             course;
    float             target_speed;
    float             target_course;
    float             steering_reserved_1;
    float             steering_reserved_2;
    float             steering_reserved_3;
    float             steering_servo;
    float             throttle_integral;
    float             throttle_differential;
    float             throttle_delta;
    float             throttle_servo;
    uint16_t          servo_steering;
    uint16_t          servo_throttle;
} control_state_t;

typedef struct _control_record_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    control_state_t   state;
} control_record_entry_t;

typedef struct _profile_entry_t {
    uint8_t           type;
    uint8_t           flags;
    uint16_t          utime;
    uint32_t          ltime;
    uint32_t          stack[2];
    uint64_t          cycles[16];    
} profile_entry_t;

typedef struct _profile_entry_payload_t {
    uint32_t          stack[2];
    uint64_t          cycles[16];    
} profile_entry_payload_t;

#define CONTROL_STATUS_IMU_READY        0x00000001
#define CONTROL_STATUS_IMU_FAULT        0x00000002
#define CONTROL_STATUS_GPS_READY        0x00000010
#define CONTROL_STATUS_GPS_FAULT        0x00000020
#define CONTROL_STATUS_RCV_READY        0x00000100
#define CONTROL_STATUS_RCV_FAULT        0x00000200
#define CONTROL_STATUS_RCV_ACTIVE       0x00000400
#define CONTROL_STATUS_SDC_READY        0x00001000
#define CONTROL_STATUS_SDC_FAULT        0x00002000
#define CONTROL_STATUS_CRASH            0x00010000
#define CONTROL_STATUS_MODE_PASSTHROU   0x04000000
#define CONTROL_STATUS_MODE_FAILSAFE    0x08000000
#define CONTROL_STATUS_STATE_READY      0x10000000
#define CONTROL_STATUS_STATE_SET        0x20000000
#define CONTROL_STATUS_STATE_GO         0x40000000
#define CONTROL_STATUS_STATE_HALT       0x80000000

extern void control_imu_fault(void);
extern void control_imu_initialize(int success);

extern void control_rcv_fault(void);
extern void control_rcv_active(void);
extern void control_rcv_initialize(int success);

extern void control_gps_ini_callback(int success);
extern void control_sdc_ini_callback(int success);

extern void control_crash(void);

extern void control_routine(unsigned int mode, uint16_t *p_steering, uint16_t *p_throttle);
extern uint32_t control_status(void);
extern void control_initialize(void);

#endif /* _CONTROL_H */

