/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     liftControl.c                                                */
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
/** @file    liftControl.c
  * @brief   Advanced PID for lift control
*//*---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <math.h>

#include "ch.h"  					// needed for all ChibiOS programs
#include "hal.h" 					// hardware abstraction layer header
#include "vex.h"					// vex library header
#include "robotc_glue.h"
#include "smartmotor.h"
#include "presets.h"
#include "fastmath.c"
#include "motionPid.h"

// Values specific to my lift
#define	LIFT_ABSOLUTE_UPPER_LIMIT	3200
#define	LIFT_UPPER_LIMIT			3136
#define	LIFT_LOWER_LIMIT			0
#define	LIFT_ABSOLUTE_LOWER_LIMIT  -50
#define	LIFT_MAX_RPM				100

// Encoder counts per revolution - 393 on Torque with IME
#define LIFT_COUNTS_PER_REV         627.2
// specific to my lift - 12 tooth sprocket drives the chain
// 12 links of chain = 12 * 0.386
//#define LIFT_INCH_PER_REV			4.628
#define LIFT_INCH_PER_REV			2.4
// Therefore the number of encoder counts per inch is
#define	LIFT_COUNTS_PER_INCH		(LIFT_COUNTS_PER_REV / LIFT_INCH_PER_REV)

// Holding power to overcome gravity, reduce this as much as possible
//#define	LIFT_HOLD_POWER				12
#define	LIFT_HOLD_POWER				0

#define	LIFT_MAX_POWER				127

#define LCD_DISP    			    VEX_LCD_DISPLAY_2

#define	PID_LOOP_SPEED			    5
#define VF_SAMPLES					4

// Assumptions
// motors[0] must be encoded with an IME
// 4.6285 inches per rev with 12 tooth sprocket
// Demo code uses 2.4 inches per rev with 60 tooth gear assuming 12 inch for one rotation

#define	MP_MAX_MOTORS				4

// Structure to gather all the motion profile data
typedef struct _lift_controller {
	// Motion profile controller
	mp_controller	mp;

	// Presets
	vexPreset		vp;						///< preset positions
    } lift_controller;

// An instance of the lift
static	lift_controller	lift;

// flag for debugging PID
static  int pidDebug = 0;

// There is no sgn function in the standard library
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
/** @brief      Limit motor power callback                                     */
/** @param[in]  mp pointer to motion controller structure                      */
/*-----------------------------------------------------------------------------*/

void
LiftLimitCallback( mp_controller *mp )
{
	static	int16_t	low_velocity = 0;

    // Check lower limit switch on digital input 10
    if( mp->motor_drive < 0 && vexDigitalPinGet( kVexDigital_10 ) == 0 )
    	{
    	MpResetEncoder( mp );
        mp->motor_drive = LIFT_HOLD_POWER;
    	}

    // Final safety limit for the top position
    if( mp->motor_drive > 0 && mp->p_pid.current > LIFT_ABSOLUTE_UPPER_LIMIT )
    	mp->motor_drive = 0;

    // We don't need this as we have a limit switch at the lower end
    // Final safety limit for lower position
    // if( mp->motor_drive < 0 && mp->p_current < LIFT_ABSOLUTE_LOWER_LIMIT )
    // 	mp->motor_drive = 0;

}

/*-----------------------------------------------------------------------------*/
/** @brief      Initialize the Lift controller                                 */
/** @param[in]  mp pointer to motion controller structure                      */
/** @param[in]  m1 first motor controlled by motion controller                 */
/** @param[in]  m2 second motor controlled by motion controller                */
/** @param[in]  m3 third motor controlled by motion controller                 */
/** @param[in]  m4 fourth motor controlled by motion controller                */
/*-----------------------------------------------------------------------------*/

void
LiftControlInit( mp_controller *mp, tVexMotor m1, tVexMotor m2, tVexMotor m3, tVexMotor m4 )
{
	MpInit( mp, m1, m2, m3, m4, LIFT_UPPER_LIMIT, LIFT_LOWER_LIMIT );
    MpSetLimitCallback( mp, LiftLimitCallback );

	// Init position PID variables
	mp->p_pid.Kp                = 0.02;
	mp->p_pid.Ki                = 0.0005;
	mp->p_pid.Kd                = 0.010;
	mp->p_pid.Kvff              = 0.0005;
	mp->p_pid.Kbias             = (float)LIFT_HOLD_POWER / (float)MP_MAX_POWER;
	mp->p_pid.Kbias             = 0;

	// Init velocity PID variables
	mp->v_pid.Kp                = 0.002;
	mp->v_pid.Ki                = 0.0;
	mp->v_pid.Kd                = 0.005;
	mp->v_pid.Kvff              = 0.0;
	mp->v_pid.Kbias             = (float)LIFT_HOLD_POWER;
}



/*-----------------------------------------------------------------------------*/
/*  Arm control task                                                           */
/*-----------------------------------------------------------------------------*/

void
SetPidDebug( int value )
{
	pidDebug = value;
}

task
LiftTask(void *arg)
{
	int16_t		speed;
	int16_t		preset_control = 0;
	int16_t		preset_action = 0;
	int16_t		preset_running = 0;
	int16_t		manual_button = 0;
	int16_t		debug_ctr = 0;
	bool_t		kill = false;
	int16_t		target = 0;
	int16_t		last_target = 0;
	int16_t		last_speed;

    (void) arg;

    // Must call this
    vexTaskRegister("lift task");

    // Init
    LiftControlInit( &lift.mp, kVexMotor_1, kVexMotor_10, kVexMotor_None, kVexMotor_None );

    // Add presets at 0, 3, 6, 9 inches and max height
    PresetInit( &lift.vp );
    PresetAdd( &lift.vp, LIFT_LOWER_LIMIT );
    PresetAdd( &lift.vp, LIFT_COUNTS_PER_INCH * 3 );
    PresetAdd( &lift.vp, LIFT_COUNTS_PER_INCH * 6 );
    PresetAdd( &lift.vp, LIFT_COUNTS_PER_INCH * 9 );
    PresetAdd( &lift.vp, LIFT_UPPER_LIMIT );

    while(!chThdShouldTerminate())
        {
		// presets

		// Next, prev or release preset button
		if( vexControllerGet(Btn8U) )
			preset_control = 1;
		else
		if( vexControllerGet(Btn8D) )
			preset_control = -1;
		else
			preset_control = 0;

		// Next preset
		if( ( preset_control > 0 ) && (!preset_action) )
			{
			// detect button push on first loop
			preset_action = 1;
			// next preset
			if( PresetGetNextPosition( &lift.vp, lift.mp.mp_target, 1 ) != (-1) )
				{
				target = PresetCurrentPosition( &lift.vp );
				MpSetPosition( &lift.mp, target, LIFT_MAX_RPM );
				preset_running = 1;
				}
			}
		else
		// Previous preset
		if( ( preset_control < 0 ) && (!preset_action) )
			{
			// detect button push on first loop
			preset_action = 1;
			// previous preset
			if( PresetGetNextPosition( &lift.vp, lift.mp.mp_target, 0 ) != (-1) )
				{
				target = PresetCurrentPosition( &lift.vp );
				MpSetPosition( &lift.mp, target, LIFT_MAX_RPM );
				preset_running = 1;
				}
			}
		else
		// wait for button release
		if( preset_control == 0 )
			{
			// use manual
			preset_action = 0;

			// manual control
			// Slow down
			if( vexControllerGet( Btn6D ) )
				{
				// Recal - we will trip limit switch
				MpSetPosition( &lift.mp, -2000, 20 );
				manual_button = 1;
				}
			else
			// Slow Up
			if( vexControllerGet( Btn6U ) )
				{
				MpSetPosition( &lift.mp, LIFT_UPPER_LIMIT, 20 );
				manual_button = 1;
				}
			else
			// Joystick control
				{
				speed = vexControllerGet( Ch2 );
				if( abs(speed) > 10 )
					{
					preset_running = 0;
					if( speed >= 0 )
						target = LIFT_UPPER_LIMIT;
					else
						target = LIFT_LOWER_LIMIT;

					// Use non-linear joystick to speed control
					speed = (speed*speed)/128 * LIFT_MAX_RPM / 128;

					MpSetPosition( &lift.mp, target, abs(speed) );
					}
				else
					{
					if(!preset_running || manual_button)
						{
						// Decelerate to stop using these parameters
						last_target = MpCalculateStopPosition(&lift.mp);
						last_speed  = lift.mp.mp_disp_velocity;

						preset_running = 1;
						manual_button  = 0;

						MpSetPosition( &lift.mp, last_target, abs(last_speed) );
						}
					}
				}
			}

		// 5U is a kill switch - remove all power from PID controlled motors
		if( vexControllerGet( Btn5U ) == 1 )
			kill = true;
		else
			kill = false;

		// Debug time spent in the PID calculation
		vexDigitalPinSet( kVexDigital_2, 1);

		// control the lift
    	MpControlUpdate( &lift.mp, kill );

    	// current calculation
    	MpMotorCurrent( &lift.mp, vexSpiGetMainBattery()/1000.0 );

    	// Debug time spent in the PID calculation
		vexDigitalPinSet( kVexDigital_2, 0);

		// Every 5 loops print some debug information
    	if(++debug_ctr == 5)
    		{
    		debug_ctr = 0;

    		vexLcdPrintf( LCD_DISP, VEX_LCD_LINE_T, "%4d %4d  %5.2f", lift.mp.p_pid.target,  lift.mp.p_pid.current, vexSpiGetMainBattery()/1000.0 );
    		vexLcdPrintf( LCD_DISP, VEX_LCD_LINE_B, "%2d %3d %3d %5.2f", lift.mp.mp_disp_velocity, lift.mp.v_current, lift.mp.motor_drive, lift.mp.filtered_current);

        	if(pidDebug)
        	    vex_printf("PD %4d, %5d, %5d, %5d, %5d, %5d, %5d %7.2f\r\n", (int)lift.mp.p_pid.error,  (int)lift.mp.p_pid.current,  (int)lift.mp.mp_position, (int)lift.mp.mp_velocity, (int)lift.mp.mp_acceleration*8, (int)lift.mp.motor_drive, (int)lift.mp.v_current, (float)lift.mp.counter * (PID_LOOP_SPEED/1000.0) );
    		}

    	//
        wait1Msec(PID_LOOP_SPEED);
        lift.mp.counter++;
        }

    return (msg_t)0;
}




