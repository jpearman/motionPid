/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     motionPid.c                                                  */
/*    Author:     James Pearman                                                */
/*    Created:    10 July 2014                                                 */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00     10 July 2014 - Initial release                     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*	  This program is free software: you can redistribute it and/or modify     */
/*	  it under the terms of the GNU General Public License as published by     */
/*	  the Free Software Foundation, either version 3 of the License, or        */
/*	  (at your option) any later version.                                      */
/*                                                                             */
/*	  This program is distributed in the hope that it will be useful,          */
/*	  but WITHOUT ANY WARRANTY; without even the implied warranty of           */
/*	  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            */
/*    GNU General Public License for more details.                             */
/*                                                                             */
/*	  You should have received a copy of the GNU General Public License        */
/*	  along with this program.  If not, see <http://www.gnu.org/licenses/>.    */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @file    motionPid.c
  * @brief   Advanced PID with motion profiles
*//*---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <math.h>

#include "ch.h"  					// needed for all ChibiOS programs
#include "hal.h" 					// hardware abstraction layer header
#include "vex.h"					// vex library header
#include "smartmotor.h"
#include "fastmath.c"
#include "motionPid.h"


// There is no sign function in the standard library
static inline float
fsgn(float x)
{
    if( x == 0 ) return 0;
    if( x > 0 ) return 1; else return (-1);
}
static inline int16_t
sgn(int16_t x)
{
    if( x == 0 ) return 0;
    if( x > 0 ) return 1; else return (-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Create a power based lut                                       */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/
/*  @details
 *   VEX motors do not increase in speed proportionally to their control value.
 *   This is the same LUT that I used in the old PID library.
 */
void
MpMakeRemapLut( mp_controller *mp )
{
    int16_t   i;
    float     x;

    for(i=0;i<REMAP_LUT_SIZE;i++)
        {
        // check for valid power base
        if( REMAP_LUT_FACTOR > 1 )
            {
            x = fastpow( REMAP_LUT_FACTOR, (float)i / (float)(REMAP_LUT_SIZE-1) );

            if(i >= (REMAP_LUT_OFFSET/2))
            	mp->motor_remap[i] = (((x - 1.0) / (REMAP_LUT_FACTOR - 1.0)) * (REMAP_LUT_SIZE-1-REMAP_LUT_OFFSET)) + REMAP_LUT_OFFSET;
            else
            	mp->motor_remap[i] = i * 2;
            }
        else
            {
            // Linear
            mp->motor_remap[i] = i;
            }
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Initialize the Motion Profile/PID controller                   */
/** @param[in]  mp pointer to motion controller structure                      */
/** @param[in]  m1 first motor controlled by motion controller                 */
/** @param[in]  m2 second motor controlled by motion controller                */
/** @param[in]  m3 third motor controlled by motion controller                 */
/** @param[in]  m4 fourth motor controlled by motion controller                */
/** @param[in]  ul upper limit used by motion controller                       */
/** @param[in]  ll upper limit used by motion controller                       */
/*-----------------------------------------------------------------------------*/

void
MpInit( mp_controller *mp, tVexMotor m1, tVexMotor m2, tVexMotor m3, tVexMotor m4, int32_t ul, int32_t ll )
{
	// Save the motors on this lift
    mp->motors[0] = m1;
    mp->motors[1] = m2;
    mp->motors[2] = m3;
    mp->motors[3] = m4;

    // debug counter
    mp->counter   = 0;

    // limits for stop position calculation
    mp->upper_limit = ul;
    mp->lower_limit = ll;
    mp->limitCallback = NULL;

    // Clear encoder
    vexMotorPositionSet( mp->motors[0], 0 );

    if( vexMotorTypeGet( mp->motors[0] ) == kVexMotor393T )
    	{
    	mp->ticks_per_rev = SMLIB_TPR_393T;
        // setup for current calculation
        mp->i_free              = SMLIB_I_FREE_393;
        mp->i_stall             = SMLIB_I_STALL_393;
        mp->r_motor             = SMLIB_R_393;
        mp->l_motor             = SMLIB_L_393;
        mp->ke_motor            = SMLIB_Ke_393;
        mp->rpm_free            = SMLIB_RPM_FREE_393;
        // maximum theoretical v_bemf
        mp->v_bemf_max          = mp->ke_motor * mp->rpm_free;
    	}

    if( vexMotorTypeGet( mp->motors[0] ) == kVexMotor393S )
    	{
    	mp->ticks_per_rev = SMLIB_TPR_393S;
        // setup for current calculation
        mp->i_free              = SMLIB_I_FREE_393;
        mp->i_stall             = SMLIB_I_STALL_393;
        mp->r_motor             = SMLIB_R_393;
        mp->l_motor             = SMLIB_L_393;
        mp->ke_motor            = SMLIB_Ke_393 / 1.6;
        mp->rpm_free            = SMLIB_RPM_FREE_393 * 1.6;
        // maximum theoretical v_bemf
        mp->v_bemf_max          = mp->ke_motor * mp->rpm_free;
    	}

    // clear target
	mp->mp_target           = 0;

	// motion profile constants
	mp->mp_deltaT           = (PID_LOOP_SPEED / 1000.0);
	// maximum velocity, this will be changed by control
	mp->mp_max_velocity     = (MP_MAX_RPM / 60.0 * mp->ticks_per_rev);
	// accelerate to maximum over 1/3 second
    mp->mp_max_acceleration = (mp->mp_max_velocity * (PID_LOOP_SPEED / 1000.0)) / MP_DEFAULT_ACCELERATION_TIME;
    // pre-calculate distance to stop constant
    mp->mp_stop_constant    = mp->mp_deltaT / (2 * mp->mp_max_acceleration);
    // motion profile error threshold
    mp->mp_error_threshold  = 1;

	// pre-calculate the constant used to calculate motor velocity
	mp->v_speed_constant    =  60000.0/(mp->ticks_per_rev*VF_SAMPLES);

	// All these will be overwritten by the calling process
	// in "normal" operation.  These are just placeholders

    // Init position PID variables
	mp->p_pid.Kp              = 0.001;
	mp->p_pid.Ki              = 0.0;
	mp->p_pid.Kd              = 0.0;
	mp->p_pid.Kvff            = 0.0;
	mp->p_pid.Kbias           = 0.0;

	// zero out working variables
    mp->p_pid.error           = 0;
    mp->p_pid.last_error      = 0;
    mp->p_pid.integral        = 0;
    mp->p_pid.derivative      = 0;
    mp->p_pid.error_threshold = 6;

    // Zero drive
    mp->p_pid.drive           = 0.0;

    // Calculate the integral limit
    if(mp->p_pid.Ki != 0)
       mp->p_pid.integral_limit = (MP_INTEGRAL_DRIVE_MAX / mp->p_pid.Ki);
    else
       mp->p_pid.integral_limit = 0;

    // Init velocity PID variables
	mp->v_pid.Kp              = 0.001;
	mp->v_pid.Ki              = 0.0;
	mp->v_pid.Kd              = 0.0;
	mp->v_pid.Kvff            = 0.0;
	mp->v_pid.Kbias           = 0.0;

	// zero out working variables
    mp->v_pid.error           = 0;
    mp->v_pid.last_error      = 0;
    mp->v_pid.integral        = 0;
    mp->v_pid.derivative      = 0;
    mp->v_pid.error_threshold = 2;

    // create a LUT to help with the non-linear behavior of the VEX motors
    MpMakeRemapLut( mp );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set the callback for handling limits                           */
/** @param[in]  mp pointer to motion controller structure                      */
/** @param[in]  cb pointer to callback function                                */
/*-----------------------------------------------------------------------------*/
/*  @details
 *   To keep implementation specific details out of this code we use a callback
 *   to limit motor power if, for example, we hit a limit switch.
 */
void
MpSetLimitCallback( mp_controller *mp, void *cb )
{
	mp->limitCallback = cb;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set the controller position                                    */
/** @param[in]  mp pointer to motion controller structure                      */
/** @param[in]  position desired position                                      */
/** @param[in]  v_max maximum speed when moving to position                    */
/*-----------------------------------------------------------------------------*/

void
MpSetPosition( mp_controller *mp, int position, int v_max )
{
	mp->mp_target       = position;
	// convert v_max (in rpm) to ticks per second
	mp->mp_max_velocity = (float)v_max / 60.0 * mp->ticks_per_rev;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Reset the motion controller encoder                            */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/
/*  @details
 *   If we reset the encoder to 0 without also reseting the other variables
 *   associated with the motion controller, then we will create a large error
 *   that the motion controller will try and reduce.
 */
void
MpResetEncoder( mp_controller *mp )
{
    vexMotorPositionSet( mp->motors[0], 0 );
	mp->mp_target        = 0;
	mp->mp_position      = 0;
	mp->p_pid.error      = 0;
    mp->p_pid.last_error = 0;
	mp->p_pid.integral   = 0;
	mp->v_pid.error      = 0;
    mp->v_pid.last_error = 0;
	mp->v_pid.integral   = 0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Calculate the lift stop position                               */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/
/** @details
 *   Calculate the position the controller will come to a stop based on current
 *   velocity if it were to be decelerated from this point.
 */

int32_t
MpCalculateStopPosition( mp_controller *mp )
{
	float   dist_to_stop;
	int32_t	stop_position;

	// do we need to decelerate to a stop?
    dist_to_stop =  (mp->mp_velocity * mp->mp_velocity * mp->mp_stop_constant) + abs(mp->mp_velocity * mp->mp_deltaT);

    // calculate stop position
	if( mp->mp_velocity > 0 )
		stop_position = (int32_t)(mp->mp_position + dist_to_stop);
	else
		stop_position = (int32_t)(mp->mp_position - dist_to_stop);

	// clip stop position to limits
	if( stop_position > mp->upper_limit )
		stop_position = mp->upper_limit;
	if( stop_position < mp->lower_limit )
		stop_position = mp->lower_limit;

	return( stop_position );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Update the motion profile variables                            */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/
/*
 * @details
 *  This function calculates the motion profile position, acceleration and
 *  velocity for the next time step.
 *  The motion profile is trapezoidal in this version of the code.
 */
void
MpControlUpdateMotionProfile( mp_controller *mp )
{
	float   dist_to_stop;

	// Calculate position error
	mp->mp_error = mp->mp_target - mp->mp_position;

	// force error to 0 if below threshold
	if( abs(mp->mp_error) < mp->mp_error_threshold )
		{
		mp->mp_error = 0;
		mp->mp_velocity = 0;
		mp->mp_acceleration = 0;
		mp->mp_position = mp->mp_target;
		mp->mp_disp_velocity = 0;
		return;
		}

	// do we need to decelerate to a stop yet?
    dist_to_stop =  (mp->mp_velocity * mp->mp_velocity * mp->mp_stop_constant) + abs(mp->mp_velocity * mp->mp_deltaT);

	// calculate acceleration
	if( mp->mp_error > 0 )
		{
		// Going forwards
		if( mp->mp_error <= dist_to_stop && mp->mp_velocity >= 0 )
			mp->mp_acceleration = -mp->mp_max_acceleration;
		else
			{
			if(mp->mp_acceleration < 0 )
				mp->mp_acceleration = 0;
			else
				mp->mp_acceleration = mp->mp_max_acceleration;
			}
		}
	else
		{
		// Going backwards
		if( -mp->mp_error <= dist_to_stop && mp->mp_velocity <= 0)
			mp->mp_acceleration = mp->mp_max_acceleration;
		else
			{
			if( mp->mp_acceleration > 0 )
				mp->mp_acceleration = 0;
			else
				mp->mp_acceleration = -mp->mp_max_acceleration;
			}
		}

	// new position
	mp->mp_position += ((mp->mp_velocity + mp->mp_acceleration/2) * mp->mp_deltaT);

	// See if max velocity has moved lower than current velocity
	if( abs(mp->mp_velocity) > mp->mp_max_velocity )
		{
		// It has, we much decelerate
		if( mp->mp_velocity > 0 )
			mp->mp_acceleration = -mp->mp_max_acceleration;
		else
			mp->mp_acceleration = mp->mp_max_acceleration;
		}

	// new velocity
	mp->mp_velocity += mp->mp_acceleration;

	// clip velocity if we were accelerating
	if( abs(mp->mp_velocity) > mp->mp_max_velocity )
		{
		if( mp->mp_velocity > 0 && mp->mp_acceleration > 0 )
			mp->mp_velocity = mp->mp_max_velocity;

		if( mp->mp_velocity < 0 && mp->mp_acceleration < 0 )
			mp->mp_velocity = -mp->mp_max_velocity;

		mp->mp_acceleration = 0; // mostly for debug
		}

	// for debug
	mp->mp_disp_velocity = mp->mp_velocity * 60 / mp->ticks_per_rev;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Update the position pid controller variables                   */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/
/*
 * @details
 *  This function calculates motor drive based on current and requested
 *  positions.  Motor drive is calculated in the range of (-1) to (+1).
 */

void
MpControlUpdatePositionPid( mp_controller *mp )
{
	// calculate position error
	mp->p_pid.error = mp->p_pid.target - mp->p_pid.current;

	// force error to 0 if below threshold
    if( fabs(mp->p_pid.error) < mp->p_pid.error_threshold )
    	mp->p_pid.error = 0;

    // integral accumulation
    if( mp->p_pid.Ki != 0 )
        {
        mp->p_pid.integral += mp->p_pid.error;

        // limit to avoid windup
        if( fabs( mp->p_pid.integral ) > mp->p_pid.integral_limit )
            mp->p_pid.integral = fsgn(mp->p_pid.integral) * mp->p_pid.integral_limit;
        }
    else
        mp->p_pid.integral = 0;

    // derivative
    mp->p_pid.derivative = mp->p_pid.error - mp->p_pid.last_error;
    mp->p_pid.last_error = mp->p_pid.error;

    // calculate drive - no delta T in this version
    mp->p_pid.drive = (mp->p_pid.Kp * mp->p_pid.error) + (mp->p_pid.Ki * mp->p_pid.integral) + (mp->p_pid.Kd * mp->p_pid.derivative) + (mp->p_pid.Kvff * mp->mp_velocity) + mp->p_pid.Kbias;

    // drive should be in the range +/- 1.0
    if( fabs( mp->p_pid.drive ) > 1.0 )
    	mp->p_pid.drive = fsgn(mp->p_pid.drive);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Update the velocity pid controller variables                   */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/

void
MpControlUpdateVelocityPid( mp_controller *mp )
{
    // calculate error in velocity
    mp->v_pid.error = mp->v_pid.target - mp->v_pid.current;

	// force error to 0 if below threshold
    if( fabs(mp->v_pid.error) < mp->v_pid.error_threshold )
        mp->v_pid.error = 0;

    // integrate velocity error
    if( mp->v_pid.Ki != 0 )
        {
        mp->v_pid.integral += mp->v_pid.error;

        // limit to avoid windup
        if( fabs( mp->v_pid.integral ) > mp->v_pid.integral_limit )
            mp->v_pid.integral = fsgn(mp->v_pid.integral) * mp->v_pid.integral_limit;
        }
    else
        mp->v_pid.integral = 0;

    // change in velocity error
    mp->v_pid.derivative = mp->v_pid.error - mp->v_pid.last_error;
    mp->v_pid.last_error = mp->v_pid.error;


    // The velocity PID calculation
    if( abs(mp->v_pid.target) > 2 )
    	{
    	mp->v_pid.drive =  mp->v_pid.drive + ((mp->v_pid.error * mp->v_pid.Kp) + (mp->v_pid.derivative * mp->v_pid.Kd) + (mp->v_pid.integral * mp->v_pid.Ki));
		// make sure we don't create "negative" drive
		// keep motor out of dead band

		if( mp->v_pid.target >= 0 && mp->v_pid.drive < mp->v_pid.Kbias )
			{
			mp->v_pid.drive = mp->v_pid.Kbias;
			}
		//else
		if( mp->v_pid.target < 0 && mp->v_pid.drive > 0 )
			{
			mp->v_pid.drive = 0;
			}
		}
    else
    	{
    	mp->v_pid.drive = mp->v_pid.Kbias; // holding power
    	mp->v_pid.integral = 0;
    	}
}

/*-----------------------------------------------------------------------------*/
/** @brief      Calculate lift motor speed                                     */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/

void
MpCalculateSpeed( mp_controller *mp )
{
	int		    i;
	float	    tmp_delta;
	int32_t		delta_ms;

    // calculate the speed of the motor in rpm

	// This is just used so we don't need to know how often we are called
    // how many mS since we were last here
    delta_ms  = chTimeNow() - mp->v_time;
    mp->v_time = chTimeNow();

    // rotate old deltas
    for(i=VF_SAMPLES-1;i>0;i--)
    	mp->v_deltas[i] = mp->v_deltas[i-1];

    // Save new delta, p_pid.last is set in main control loop
    mp->v_deltas[0] = (mp->p_pid.current - mp->p_pid.last);

    // Calculate average of last VF_SAMPLES samples
	// Final divide by number of samples is folded in to the speed constant
    for(i=0, tmp_delta=0.0;i<VF_SAMPLES;i++)
    	tmp_delta += mp->v_deltas[i];

	// Calculate velocity
	mp->v_current = (mp->v_speed_constant * tmp_delta) / delta_ms;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Update lift motion profile and PID variables                   */
/** @param[in]  mp pointer to motion controller structure                      */
/** @param[in]  kill if true then kills all motor power                        */
/*-----------------------------------------------------------------------------*/

void
MpControlUpdate( mp_controller *mp, bool_t kill )
{
	int	i;

    // motor 0 must have IME
	mp->p_pid.current = vexMotorPositionGet( mp->motors[0] );

	// Calculate motor speed
	// output is contained in v_current
	MpCalculateSpeed( mp );

	// calculate the new target position and velocity
	MpControlUpdateMotionProfile( mp );
	// debug
	//mp->mp_position = mp->mp_target;

	// output of motion profile becomes position target
	mp->p_pid.target    = mp->mp_position;

	// Do the position PID calculations
	// Final output from this is p_drive
	MpControlUpdatePositionPid( mp );

	// save last position
	mp->p_pid.last = mp->p_pid.current;

#ifdef	VELOCITY_PID
	// drive is +- MP_MAX_POWER rpm
    mp->v_pid.target  = mp->p_pid.drive * MP_MAX_RPM;
    mp->v_pid.current = mp->v_current;
	// Do the velocity PID calculations
    MpControlUpdateVelocityPid( mp) ;
    // motor drive is output of speed control
    mp->motor_drive  = mp->v_pid.drive + 0.5;
#else
	// now calculate drive for motor
	// drive is +- MP_MAX_POWER rpm
	mp->motor_drive  = mp->p_pid.drive * MP_MAX_POWER + 0.5;
#endif

	// Limit
	if( mp->motor_drive >  127 ) mp->motor_drive =  127;
	if( mp->motor_drive < -127 ) mp->motor_drive = -127;

#ifndef	VELOCITY_PID
    // Re-map motor value through LUT
    mp->motor_drive = sgn( mp->motor_drive) * mp->motor_remap[ abs(mp->motor_drive) ];
#endif

    // external limits - switches etc.
	if( mp->limitCallback != NULL )
		mp->limitCallback( mp );

	// Finally, kill motor power if necessary
	// for example, to allow PTC reset
    if(kill)
    	mp->motor_drive = 0;

    // and finally set the motor control value
    for(i=0;i<MP_MAX_MOTORS;i++) {
    	if( mp->motors[i] != kVexMotor_None )
    	    vexMotorSet( mp->motors[i], mp->motor_drive );
    	}
}

/*-----------------------------------------------------------------------------*/
/** @brief      Estimate smart motor current                                   */
/** @param[in]  mp pointer to motion controller structure                      */
/** @param[in]  v_battery The battery voltage in volts                         */
/** @returns    The calculated current                                         */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Estimate current in Vex motor using vamfun's algorithm.\n
 *  subroutine written by Vamfun...Mentor Vex 1508, 599.\n
 *  7.13.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com\n
 *
 *  Modified by James Pearman 7.28.2012.\n
 *  Modified by James Pearman 10.1.2012 - more generalized code.\n
 *
 *  If cmd is positive then rpm must also be positive for this to work.
 *
 *  This version is dedicated to the motion controller code
 */

float
MpMotorCurrent( mp_controller *mp, float v_battery  )
{
    float   v_bemf;
    float   c1, c2;
    float   lamda;

    float   duty_on, duty_off;

    float   i_max, i_bar, i_0;
    float   i_ss_on, i_ss_off;

    int     dir;

    // get current cmd
    int     cmd = mp->motor_drive;

    // rescale control value
    // ports 2 through 9 behave a little differently
    if( mp->motors[0] > kVexMotor_1 && mp->motors[0] < kVexMotor_10 )
        cmd = (cmd * 128) / 90;

    // clip control value to +/- 127
    if( abs(cmd) > 127 )
        cmd = sgn(cmd) * 127;

    // which way are we turning ?
    // modified to use rpm near command value of 0 to reduce transients
    if( abs(cmd) > 10 )
        dir = sgn(cmd);
    else
        dir = sgn(mp->v_current);


    duty_on = abs(cmd)/127.0;

    // constants for this pwm cycle
    lamda = mp->r_motor/((float)SMLIB_PWM_FREQ * mp->l_motor);
    c1    = fastexp( -lamda *    duty_on  );
    c2    = fastexp( -lamda * (1-duty_on) );

    // Calculate back emf voltage
    v_bemf  = mp->ke_motor * mp->v_current;

    // new - clip v_bemf, stops issues if motor runs faster than rpm_free
    if( fabs(v_bemf) > mp->v_bemf_max )
        v_bemf = sgn(v_bemf) * mp->v_bemf_max;

    // Calculate steady state current for on and off pwm phases
    i_ss_on  =  ( v_battery * dir - v_bemf ) / (mp->r_motor + SMLIB_R_SYS);
    i_ss_off = -( SMLIB_V_DIODE   * dir + v_bemf ) /  mp->r_motor;

    // compute trial i_0
    i_0 = (i_ss_on*(1-c1)*c2 + i_ss_off*(1-c2))/(1-c1*c2);

    //check to see if i_0 crosses 0 during off phase if diode were not in circuit
    if(i_0*dir < 0)
        {
        // waveform reaches zero during off phase of pwm cycle hence
        // ON phase will start at 0
        // once waveform reaches 0, diode clamps the current to zero.
        i_0 = 0;

        // peak current
        i_max = i_ss_on*(1-c1);

        //where does the zero crossing occur
        duty_off = -fastlog(-i_ss_off/(i_max-i_ss_off))/lamda ;
        }
    else
        {
        // peak current
        i_max = i_0*c1 + i_ss_on*(1-c1);

        // i_0 is non zero so final value of waveform must occur at end of cycle
        duty_off = 1 - duty_on;
        }

    // Average current for cycle
    i_bar = i_ss_on*duty_on + i_ss_off*duty_off;

    // Save current
    mp->current = i_bar;

    // simple iir filter to remove transients
    mp->filtered_current = (mp->filtered_current * 0.8) + (i_bar * 0.2);

    // peak current - probably not useful
    if( fabs(mp->current) > mp->peak_current )
        mp->peak_current = fabs(mp->current);

    return i_bar;
}




