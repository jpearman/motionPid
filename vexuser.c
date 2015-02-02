/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     vexuser.c                                                    */
/*    Author:     James Pearman                                                */
/*    Created:    7 May 2013                                                   */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  XX XXX 2013 - Initial release                         */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <stdlib.h>

#include "ch.h"  		// needs for all ChibiOS programs
#include "hal.h" 		// hardware abstraction layer header
#include "vex.h"		// vex library header

#include "smartmotor.h"
#include "robotc_glue.h"
#include "pidlib.h"

// Digi IO configuration
static  vexDigiCfg  dConfig[kVexDigital_Num] = {
        { kVexDigital_1,    kVexSensorDigitalOutput, kVexConfigOutput,      0 },
        { kVexDigital_2,    kVexSensorDigitalOutput, kVexConfigOutput,      0 },
        { kVexDigital_3,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_4,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_5,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_6,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_7,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_8,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_9,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_10,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigInput,       0 }
};

static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotor393T,          kVexMotorNormal,       kVexSensorIME,        kImeChannel_1 }, // lift
        { kVexMotor_2,      kVexMotor393T,      	kVexMotorReversed,     kVexSensorNone,        0 },
        { kVexMotor_3,      kVexMotor393T,     		kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_4,      kVexMotorUndefined,     kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_5,      kVexMotorUndefined,     kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_6,      kVexMotorUndefined,     kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_7,      kVexMotorUndefined,     kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_8,      kVexMotor393T,      	kVexMotorReversed,     kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotor393T,      	kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_10,     kVexMotor393T,          kVexMotorNormal,       kVexSensorNone,        0 }
};

#define     MotorLift1          kVexMotor_1
#define     MotorLift2          kVexMotor_10

#define     MotorLF             kVexMotor_2
#define     MotorRF             kVexMotor_3

#define     MotorLB             kVexMotor_8
#define     MotorRB             kVexMotor_9

task	LiftTask(void *arg);

/*-----------------------------------------------------------------------------*/
/*  Drive control task                                                         */
/*                                                                             */
/*  Uses joystick Ch3, Ch4 and Btn8L, Btn8R                                    */
/*-----------------------------------------------------------------------------*/

void
DriveSystemMecanumDrive(void)
{
    short   forward, turn, right;

    long drive_l_front;
    long drive_l_back;
    long drive_r_front;
    long drive_r_back;

    // Get controller
    if( abs( vexControllerGet( Ch3) ) > 10 )
        forward = vexControllerGet( Ch3 );
    else
        forward = 0;

    if( abs( vexControllerGet( Ch4 ) ) > 10 )
        right   = vexControllerGet( Ch4 );
    else
        right = 0;

    if( vexControllerGet( Btn8R ) == 1 )
        turn = 64;
    else
    if( vexControllerGet( Btn8L ) == 1 )
        turn = -64;
    else
        turn = 0;

    // Set drive
    drive_l_front = forward + turn + right;
    drive_l_back  = forward + turn - right;

    drive_r_front = forward - turn - right;
    drive_r_back  = forward - turn + right;

    // normalize drive so max is 127 if any drive is over 127
    int max = abs(drive_l_front);
    if (abs(drive_l_back)  > max)
        max = abs(drive_l_back);
    if (abs(drive_r_back)  > max)
        max = abs(drive_r_back);
    if (abs(drive_r_front) > max)
        max = abs(drive_r_front);
    if (max>127) {
        drive_l_front = 127 * drive_l_front / max;
        drive_l_back  = 127 * drive_l_back  / max;
        drive_r_back  = 127 * drive_r_back  / max;
        drive_r_front = 127 * drive_r_front / max;
    }

    // Send to motors
    // left drive
    SetMotor( MotorLF, drive_l_front);
    SetMotor( MotorLB, drive_l_back);

    // right drive
    SetMotor( MotorRF, drive_r_front);
    SetMotor( MotorRB, drive_r_back);
}

/*-----------------------------------------------------------------------------*/
/*  Drive control task                                                         */
/*-----------------------------------------------------------------------------*/

task
DriveTask(void *arg)
{
    (void) arg;

    // Must call this
    vexTaskRegister("drive task");

    while(!chThdShouldTerminate())
        {
        DriveSystemMecanumDrive();
        wait1Msec(25);
        }
    return (msg_t)0;
}


/*-----------------------------------------------------------------------------*/
/** @brief      User setup                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  The digital and motor ports can (should) be configured here.
 */
void
vexUserSetup()
{
	vexDigitalConfigure( dConfig, DIG_CONFIG_SIZE( dConfig ) );
	vexMotorConfigure( mConfig, MOT_CONFIG_SIZE( mConfig ) );
}

/*-----------------------------------------------------------------------------*/
/** @brief      User initialize                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function is called after all setup is complete and communication has
 *  been established with the master processor.
 *  Start other tasks and initialize user variables here
 */
void
vexUserInit()
{
    SmartMotorsInit();
    //SmartMotorCurrentMonitorEnable();

    //SmartMotorRun();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the autonomous period is started
 */
msg_t
vexAutonomous( void *arg )
{
    (void)arg;

    // Must call this
    vexTaskRegister("auton");

    while(1)
        {
        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control                                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the driver control period is started
 */
msg_t
vexOperator( void *arg )
{
	int16_t		blink = 0;

	(void)arg;

	// Must call this
	vexTaskRegister("operator");

    // control tasks
    //StartTask( DriveTask );
	StartTask( LiftTask );


	// Run until asked to terminate
	while(!chThdShouldTerminate())
		{
		// flash led/digi out
		vexDigitalPinSet( kVexDigital_1, (blink++ >> 3) & 1);

		// Don't hog cpu
		vexSleep( 25 );
		}

	return (msg_t)0;
}



