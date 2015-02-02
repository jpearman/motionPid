/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     motionPid.h                                                  */
/*    Author:     James Pearman                                                */
/*    Created:    10 July 2014                                                 */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00     10 July 2014 - Initial release                     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    This program is free software: you can redistribute it and/or modify     */
/*    it under the terms of the GNU General Public License as published by     */
/*    the Free Software Foundation, either version 3 of the License, or        */
/*    (at your option) any later version.                                      */
/*                                                                             */
/*    This program is distributed in the hope that it will be useful,          */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of           */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            */
/*    GNU General Public License for more details.                             */
/*                                                                             */
/*    You should have received a copy of the GNU General Public License        */
/*    along with this program.  If not, see <http://www.gnu.org/licenses/>.    */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#ifndef __MPLIB__
#define __MPLIB__

/*-----------------------------------------------------------------------------*/
/** @file    motionPid.h
  * @brief   Advanced PID with motion profiles - header
*//*---------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

#define PID_LOOP_SPEED              5
#define VF_SAMPLES                  4

// Assumptions
// motors[0] must be encoded with an IME
// 4.6285 inches per rev with 12 tooth sprocket

#define MP_MAX_MOTORS               4
#define MP_MAX_POWER              127
#define MP_MAX_RPM                 80
#define MP_INTEGRAL_DRIVE_MAX    0.25

/** @brief size of linearizing table
 */
#define REMAP_LUT_SIZE           128
/** @brief severity pf the linearize power function
 */
#define REMAP_LUT_FACTOR        20.0
/** @brief offset below which the linearize function is not used
 */
#define REMAP_LUT_OFFSET          10

/** @brief TIme to accelerate from stop to maximum
 */
#define MP_DEFAULT_ACCELERATION_TIME    0.5

/** @brief variables used by a PID controller
 */
typedef struct _pid_controller {
    int32_t         target;                 ///< target position
    int32_t         current;                ///< current position
    int32_t         last;                   ///< last position
    float           error;                  ///< error between actual position and target
    float           last_error;             ///< error last time update called
    float           integral;               ///< integrated error
    float           integral_limit;         ///< limit for integrated error
    float           derivative;             ///< change in error from last time
    float           error_threshold;        ///< threshold below which error is ignored
    float           Kp;                     ///< constant - proportional
    float           Kd;                     ///< constant - derivative
    float           Ki;                     ///< constant - integral
    float           Kvff;                   ///< constant - Velocity feed forward
    float           Kbias;                  ///< constant - fixed drive offset
    float           drive;                  ///< final drive out of PID (-1.0 to 1.0)
    } pid_controller;

// Structure to gather all the motion profile data
typedef struct _mp_controller {
    int32_t         counter;                ///< loop counter used for debug
    tVexMotor       motors[MP_MAX_MOTORS];  ///< motors that control mechanism

    // encoder tick per revolution
    float           ticks_per_rev;

    // Limits
    int32_t         upper_limit;
    int32_t         lower_limit;
    void          (*limitCallback)( struct _mp_controller *mp );

    // profile control
    int32_t         mp_target;              ///< target position
    int32_t         mp_error;               ///< error between profile position and target
    int32_t         mp_error_threshold;     ///< threshold below which error is ignored
    int32_t         mp_max_velocity;        ///< maximum allowed drive speed in counts/sec
    int32_t         mp_max_acceleration;    ///< change of drive speed per update loop
    float           mp_stop_constant;       ///< constant used when calculating distance to stop
    float           mp_acceleration;        ///< motion profile acceleration (counts/sec^2)
    float           mp_velocity;            ///< motion profile velocity     (counts/sec)
    float           mp_position;            ///< motion profile position     (counts)
    float           mp_deltaT;              ///< motion profile time slice
    int32_t         mp_disp_velocity;       ///< motion profile velocity     (rpm)

    // position control PID
    pid_controller  p_pid;                  ///< position PID controller

    // velocity measurement
    int32_t         v_current;              ///< current speed
    int32_t         v_deltas[VF_SAMPLES];   ///< samples for velocity digital filter
    float           v_speed_constant;       ///< constant used to calculate motor velocity
    uint32_t        v_time;                 ///< Time of last velocity calculation

    // velocity control PID
    pid_controller  v_pid;                  ///< velocity PID controller

    // final motor drive
    int16_t         motor_drive;            ///< final motor control value

    // remap lut
    int16_t         motor_remap[REMAP_LUT_SIZE];

    // variables used for current calculation
    float           i_free;                 ///< free current for kotor
    float           i_stall;                ///< stall current for motor
    float           r_motor;                ///< resistance of motor
    float           l_motor;                ///< inductance of motor
    float           ke_motor;               ///< back emf constant
    float           rpm_free;               ///< free speed of motor with no load
    float           v_bemf_max;             ///< maximum back emf motor will generate
    float           current;                ///< instantaneous calculated current
    float           filtered_current;       ///< average current
    float           peak_current;           ///< peak current
    } mp_controller;


void    MpInit( mp_controller *mp, tVexMotor m1, tVexMotor m2, tVexMotor m3, tVexMotor m4, int32_t ul, int32_t ll );
void    MpSetLimitCallback( mp_controller *mp, void *cb );
void    MpSetPosition( mp_controller *mp, int position, int v_max );
void    MpResetEncoder( mp_controller *mp );
int32_t MpCalculateStopPosition( mp_controller *mp );
void    MpControlUpdateMotionProfile( mp_controller *mp );
void    MpControlUpdatePositionPid( mp_controller *mp );
void    MpControlUpdateVelocityPid( mp_controller *mp );
void    MpCalculateSpeed( mp_controller *mp );
void    MpControlUpdate( mp_controller *mp, bool_t kill );
float   MpMotorCurrent( mp_controller *mp, float v_battery  );

#ifdef __cplusplus
}
#endif

#endif  // __MPLIB__
