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

#include "kitty.h"

typedef struct _control_device_t {
    volatile uint32_t status;
    uint32_t          count;
    float             steering_integral;
    float             steering_differential;
    float             steering_delta;
    float             steering_servo;         /* last output    */
    float             throttle_integral;
    float             throttle_differential;
    float             throttle_delta;
    float             throttle_servo;         /* last output    */
    int               crash;
} control_device_t;

#define CONTROL_CRASH_NONE           0
#define CONTROL_CRASH_STOP_1         1
#define CONTROL_CRASH_BACKUP_LEFT    2
#define CONTROL_CRASH_STOP_2         3
#define CONTROL_CRASH_FORWARD_RIGHT  4

static control_device_t control_device;

static void control_crash_timeout(void)
{
    switch (control_device.crash) {
    case CONTROL_CRASH_NONE:
	break;

    case CONTROL_CRASH_STOP_1:
	control_device.crash = CONTROL_CRASH_BACKUP_LEFT;
	break;

    case CONTROL_CRASH_BACKUP_LEFT:
	control_device.crash = CONTROL_CRASH_STOP_2;
	break;

    case CONTROL_CRASH_STOP_2:
	control_device.crash = CONTROL_CRASH_FORWARD_RIGHT;
	break;

    case CONTROL_CRASH_FORWARD_RIGHT:
	control_device.crash = CONTROL_CRASH_NONE;

	armv7m_atomic_and(&control_device.status, ~CONTROL_STATUS_CRASH);
	break;
    }

    if (control_device.crash == CONTROL_CRASH_NONE)
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_CONTROL, 0, NULL);
    }
    else
    {
	armv7m_systick_timeout(TIMEOUT_SLOT_CONTROL, 2000, control_crash_timeout);
	
    }
}

void control_imu_fault(void)
{
    armv7m_atomic_or(&control_device.status, CONTROL_STATUS_IMU_FAULT);
}

void control_imu_initialize(int success)
{
    armv7m_atomic_or(&control_device.status, (success ? CONTROL_STATUS_IMU_READY : CONTROL_STATUS_IMU_FAULT));
}

void control_rcv_fault(void)
{
    armv7m_atomic_or(&control_device.status, CONTROL_STATUS_RCV_FAULT);
}

void control_rcv_active(void)
{
    armv7m_atomic_or(&control_device.status, CONTROL_STATUS_RCV_ACTIVE);
}

void control_rcv_initialize(int success)
{
    armv7m_atomic_or(&control_device.status, (success ? CONTROL_STATUS_RCV_READY : CONTROL_STATUS_RCV_FAULT));
}

void control_gps_ini_callback(int success)
{
    armv7m_atomic_or(&control_device.status, (success ? CONTROL_STATUS_GPS_READY : CONTROL_STATUS_GPS_FAULT));
}

void control_sdc_ini_callback(int success)
{
    armv7m_atomic_or(&control_device.status, (success ? CONTROL_STATUS_SDC_READY : CONTROL_STATUS_SDC_FAULT));
}

void control_crash(void)
{
    armv7m_atomic_or(&control_device.status, CONTROL_STATUS_CRASH);
}

static uint32_t control_steering(float steering_target, float steering_feedback, float speed)
{
    float steering_range, steering_servo;

    if (speed < calibration.steering_speed_min)
    {
	steering_range = calibration.steering_range_max;
    }
    else if (speed > calibration.steering_speed_max)
    {
	steering_range = calibration.steering_range_min;
    }
    else
    {
	steering_range = ((calibration.steering_range_max - calibration.steering_range_min) *
			  ((speed - calibration.steering_speed_min) * (calibration.steering_speed_max - calibration.steering_speed_min)) +
			  calibration.steering_range_min);
    }

    steering_servo = angle_difference(steering_target, steering_feedback) * calibration.steering_gain;
    
    if (steering_servo < -steering_range)
    {
	steering_servo = -steering_range;
    }

    if (steering_servo > steering_range)
    {
	steering_servo = steering_range;
    }

    steering_servo += calibration.steering_center;

    control_device.steering_servo = steering_servo;

    return control_device.steering_servo;
}

static uint32_t control_throttle(float throttle_target, float throttle_feedback)
{
  float throttle_delta, throttle_error, throttle_servo;

  throttle_error = throttle_target - throttle_feedback;

  control_device.throttle_integral += throttle_error;
  
  if (control_device.throttle_integral > calibration.throttle_limit)
  {
      control_device.throttle_integral = calibration.throttle_limit;
  }
  
  if (control_device.throttle_integral < - calibration.throttle_limit)
  {
      control_device.throttle_integral = - calibration.throttle_limit;
  }

  throttle_delta = ((calibration.throttle_constants[0] * throttle_error) +
		    (calibration.throttle_constants[1] * control_device.throttle_integral) +
		    (calibration.throttle_constants[2] * (throttle_error - control_device.throttle_differential)));
  
  control_device.throttle_differential = throttle_error;
  control_device.throttle_delta = throttle_delta;

  if (throttle_delta > calibration.throttle_slope)
  {
      throttle_delta = calibration.throttle_slope;
  }
  
  if (throttle_delta < - calibration.throttle_slope)
  {
      throttle_delta = - calibration.throttle_slope;
  }

  throttle_servo = control_device.throttle_servo + throttle_delta;
  
  if (throttle_servo > calibration.throttle_max)
  {
      throttle_servo = calibration.throttle_max;
  }
  
  if (throttle_servo < calibration.throttle_min)
  {
      throttle_servo = calibration.throttle_min;
  }
  
  control_device.throttle_servo = throttle_servo;

  return control_device.throttle_servo;
}

void control_routine(unsigned int mode, uint16_t *p_steering, uint16_t *p_throttle)
{
    int event, status, status_previous;
    uint32_t steering, throttle;
    control_state_t state;
    float x, y, speed, course, target_speed, target_course;

    ARMV7M_PROFILE_TAG_PUSH(CONTROL);
    
    if (!tm4c123_receiver_heartbeat())
    {
	armv7m_atomic_or(&control_device.status, CONTROL_STATUS_RCV_FAULT);
    }

    status = status_previous = control_device.status;

    switch (mode) {
    case TM4C123_SERVO_MODE_AUTONOMOUS:
	break;

    case TM4C123_SERVO_MODE_PASSTHROU:
	if (!(status & CONTROL_STATUS_MODE_PASSTHROU))
	{
	    status = armv7m_atomic_or(&control_device.status, CONTROL_STATUS_MODE_PASSTHROU);
	}
	break;

    case TM4C123_SERVO_MODE_FAILSAFE:
	if (!(status & CONTROL_STATUS_MODE_FAILSAFE))
	{
	    status = armv7m_atomic_or(&control_device.status, (CONTROL_STATUS_MODE_FAILSAFE | CONTROL_STATUS_STATE_HALT));
	}
	break;
	
    default:
	break;
    }

    do
    {
	event = tm4c123_switch_event();
	    
	if (event != TM4C123_SWITCH_EVENT_NONE)
	{
	    if (!(status & CONTROL_STATUS_MODE_FAILSAFE))
	    {
		if (event == TM4C123_SWITCH_EVENT_SWITCH_ON)
		{
		    if (status & CONTROL_STATUS_STATE_HALT)
		    {
		    }
		    else
		    {
			if (status & CONTROL_STATUS_STATE_GO)
			{
			    /* STOP the rover via remote control.
			     */
			    status = armv7m_atomic_or(&control_device.status, CONTROL_STATUS_STATE_HALT);
			}
			else
			{
			    if (status & CONTROL_STATUS_STATE_SET)
			    {
				/* START the rover via remote control.
				 */
				if (navigation_active())
				{
				    status = armv7m_atomic_or(&control_device.status, CONTROL_STATUS_STATE_GO);
				}
			    }
			    else
			    {
			    }
			}
		    }
		}
	    }
	}
	else
	{
	    event = tm4c123_button_event();

	    if (event != TM4C123_BUTTON_EVENT_NONE)
	    {
		if (!(status & CONTROL_STATUS_MODE_FAILSAFE))
		{
		    if (event == TM4C123_BUTTON_EVENT_BUTTON_1_PRESS)
		    {
			if (status & CONTROL_STATUS_STATE_HALT)
			{
			}
			else
			{
			    if (status & CONTROL_STATUS_STATE_GO)
			    {
				status = armv7m_atomic_or(&control_device.status, CONTROL_STATUS_STATE_HALT);
			    }
			    else
			    {
				if (status & CONTROL_STATUS_STATE_SET)
				{
				    if (navigation_active())
				    {
					status = armv7m_atomic_or(&control_device.status, CONTROL_STATUS_STATE_GO);
				    }
				}
				else
				{
				    if ((status &
					 (CONTROL_STATUS_IMU_READY | CONTROL_STATUS_GPS_READY | CONTROL_STATUS_RCV_READY)) ==
					(CONTROL_STATUS_IMU_READY | CONTROL_STATUS_GPS_READY | CONTROL_STATUS_RCV_READY))
				    {
					status = armv7m_atomic_or(&control_device.status, CONTROL_STATUS_STATE_SET);
				    }
				}
			    }
			}
		    }

		    if (event == TM4C123_BUTTON_EVENT_BUTTON_2_PRESS)
		    {
			/* Cancel override mode.
			 */

			status = armv7m_atomic_and(&control_device.status, ~(CONTROL_STATUS_RCV_READY |
									     CONTROL_STATUS_RCV_FAULT |
									     CONTROL_STATUS_MODE_PASSTHROU |
									     CONTROL_STATUS_MODE_FAILSAFE |
									     CONTROL_STATUS_STATE_SET |
									     CONTROL_STATUS_STATE_GO |
									     CONTROL_STATUS_STATE_HALT));
			tm4c123_receiver_reset();
		    }
		}
	    }
	}
    }
    while (event != 0);

    x             = 0.0;
    y             = 0.0;
    speed         = 0.0;
    course        = 0.0;
    target_speed  = 0.0;
    target_course = 0.0;

    if (status & (CONTROL_STATUS_MODE_FAILSAFE | CONTROL_STATUS_STATE_HALT))
    {
	steering = 0;
	throttle = 1250;
    }
    else
    {
	if (status & CONTROL_STATUS_STATE_GO)
	{
	    navigation_location(tm4c123_capture_clock(), &x, &y, &speed, &course);

	    guidance_target(x, y, &target_speed, &target_course);

	    if (target_speed > 0.0)
	    {
		if (!(control_device.status & CONTROL_STATUS_CRASH))
		{
		    steering = control_steering(target_course, course, speed);

		    if (target_speed >= 2.0)
		    {
			throttle = control_throttle(target_speed, speed);
		    }
		    else
		    {
			throttle = 1450;
		    }
		}
		else
		{
		  throttle = 1500;
		  steering = 1456;

		    switch (control_device.crash) {
		    case CONTROL_CRASH_NONE:
			armv7m_systick_timeout(TIMEOUT_SLOT_CONTROL, 2000, control_crash_timeout);
			
			control_device.crash = CONTROL_CRASH_STOP_1;
			
		    case CONTROL_CRASH_STOP_1:
		    case CONTROL_CRASH_STOP_2:
			break;
			
		    case CONTROL_CRASH_BACKUP_LEFT:
			throttle = 1500 - 60;
			steering = 1456 - 60;
			break;
			
		    case CONTROL_CRASH_FORWARD_RIGHT:
			throttle = 1500 + 60;
			steering = 1456 + 60;
			break;
		    }
		}
	    }
	    else
	    {
		armv7m_atomic_or(&control_device.status, CONTROL_STATUS_STATE_HALT);
		
		steering = 0;
		throttle = 1500;
	    }
	}
	else
	{
	    steering = 1500;
	    throttle = 1500;
	}
    }

    if ((status & CONTROL_STATUS_STATE_SET) && !(status_previous & CONTROL_STATUS_STATE_SET))
    {
	record_start();
    }

    state.status                = control_device.status;
    state.x                     = x;
    state.y                     = y;
    state.speed                 = speed;
    state.course                = course;
    state.target_speed          = target_speed;
    state.target_course         = target_course;
    state.steering_servo        = control_device.steering_servo;
    state.throttle_integral     = control_device.throttle_integral;
    state.throttle_differential = control_device.throttle_differential;
    state.throttle_delta        = control_device.throttle_delta;
    state.throttle_servo        = control_device.throttle_servo;
    state.servo_steering        = steering;
    state.servo_throttle        = throttle;

    record_enter_extended(RECORD_TYPE_CONTROL, 0, tm4c123_capture_clock(), &state, sizeof(state));

    if ((!(status & CONTROL_STATUS_STATE_SET) && (status_previous & CONTROL_STATUS_STATE_SET)) ||
	((status & CONTROL_STATUS_STATE_HALT) && !(status_previous & CONTROL_STATUS_STATE_HALT)))
    {
	record_stop();
    }
 
    *p_steering = steering;
    *p_throttle = throttle;

    control_device.count++;

    if (!(control_device.count & 63))
    {
	profile_entry_payload_t profile_payload;

	profile_payload.stack[0] = (uint32_t)&__StackLimit[0];
	profile_payload.stack[1] = (uint32_t)__StackLevel;

	armv7m_profile_cycles(&profile_payload.cycles[0]);

	record_enter_extended(RECORD_TYPE_PROFILE, 0, tm4c123_capture_clock(), &profile_payload, sizeof(profile_payload));
    }

    ARMV7M_PROFILE_TAG_POP();
}

uint32_t control_status(void)
{
    return control_device.status;
}

void control_initialize(void)
{
    control_device.steering_integral      = 0.0;
    control_device.steering_differential  = 0.0;
    control_device.steering_servo         = calibration.steering_center;

    control_device.throttle_integral      = 0.0;
    control_device.throttle_differential  = 0.0;
    control_device.throttle_servo         = 1565;

    armv7m_atomic_or(&control_device.status, CONTROL_STATUS_STATE_READY);
}
