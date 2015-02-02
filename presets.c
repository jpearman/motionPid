/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*        Module:     PstLib.c                                                 */
/*        Author:     James Pearman                                            */
/*        Created:    28 Jan 2012                                              */
/*                                                                             */
/*        Revisions:  V0.1                                                     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    This file is part of ConVEX.                                             */
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. ConVEX is free software; you can redistribute it         */
/*    and/or modify it under the terms of the GNU General Public License       */
/*    as published by the Free Software Foundation; either version 3 of        */
/*    the License, or (at your option) any later version.                      */
/*                                                                             */
/*    ConVEX is distributed in the hope that it will be useful,                */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of           */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            */
/*    GNU General Public License for more details.                             */
/*                                                                             */
/*    You should have received a copy of the GNU General Public License        */
/*    along with this program.  If not, see <http://www.gnu.org/licenses/>.    */
/*                                                                             */
/*    A special exception to the GPL can be applied should you wish to         */
/*    distribute a combined work that includes ConVEX, without being obliged   */
/*    to provide the source code for any proprietary components.               */
/*    See the file exception.txt for full details of how and when the          */
/*    exception can be applied.                                                */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*-----------------------------------------------------------------------------*/
/*    Description:                                                             */
/*                                                                             */
/*    Standardized presets                                                     */
/*    Stores an array of of presets, a preset position is a signed integer     */
/*    in the range -32768 to 32767 (using long had issues in ROBOTC V3.51)     */
/*    This port for ConVEX, ints are now 32 bit                                */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
#include <stdlib.h>

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"

#include "presets.h"

/*-----------------------------------------------------------------------------*/
/** @brief  Init a preset structure                                            */
/*-----------------------------------------------------------------------------*/

void
PresetInit( vexPreset *v )
{
    int     i;

    // no presets in list
    v->preset_num  = 0;
    v->preset_ptr  = 0;

    // default tolerance 25, good for pid controlled arms etc.
    v->tolerance   = 25;

    // clear presets
    for(i=0;i<kMaxPresets;i++)
        v->presets[i] = 0;
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/** @brief  Add a preset to the list of preset positions                       */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Presets should be added in incrementing order in this version
 *  may add sort later.
 *
 */

void
PresetAdd( vexPreset *v, int position )
{
    if( v->preset_num > kMaxPresets )
        return;

    v->presets[ v->preset_num++ ] = position;
}

/*-----------------------------------------------------------------------------*/
/** @brief Set preset tolerance                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Set the tolerance used when determining if the current position is already
 *  at a preset position. Set to 1 to force an exact match.
 */

void
PresetSetTolerance( vexPreset *v, short tolerance )
{
    if( tolerance >= 1)
        v->tolerance = tolerance;
    else
        v->tolerance = 1;
}

/*-----------------------------------------------------------------------------*/
/** @brief Get next preset position                                            */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Based on a given position determine the next preset position
 *  returns the preset index (not the actual position) if a preset is found
 *  or (-1) on failure
 */

short
PresetGetNextPosition( vexPreset *v, int position, short dir )
{
    int     i;
    int     current_position = position;

    // No action
    v->preset_ptr = (-1);

    // Do we have any ?
    if( v->preset_num == 0 )
        return( v->preset_ptr );

    // see if we are exactly (within tolerance) on a preset
    for(i=0;i<v->preset_num;i++)
        {
        if( abs(v->presets[i] - current_position) < v->tolerance )
            {
            // found, pick next or previous
            if( dir == 1 )
                v->preset_ptr = ( i < (v->preset_num-1) ) ? i+1 : i;
            else
                v->preset_ptr = ( i >= 1 ) ? i-1 : 0;
            return( v->preset_ptr );
            }
        }

    // Ok, not near a curent preset so look for limit
    for(i=0;i<v->preset_num;i++)
        {
        if( v->presets[i] > current_position )
            {
            // found, pick next or previous
            if( dir == 1 )
                v->preset_ptr = i;
            else
                v->preset_ptr = ( i >= 1 ) ? i-1 : (-1);
            return( v->preset_ptr );
            }
        }

    // Beyond the last preset
    if( dir == 1 )
        v->preset_ptr = (-1);
    else
        v->preset_ptr = v->preset_num - 1;

    return( v->preset_ptr );
}

/*-----------------------------------------------------------------------------*/
/** @brief  Select a preset                                                    */
/*-----------------------------------------------------------------------------*/
void
PresetSet( vexPreset *v, short p )
{
    v->preset_ptr = p;
}

/*-----------------------------------------------------------------------------*/
/** @brief  return number of presets in the list                               */
/*-----------------------------------------------------------------------------*/
short
PresetGetNum( vexPreset *v )
{
    return( v->preset_num );
}

/*-----------------------------------------------------------------------------*/
/** @brief  get current preset position based on the current preset pointer    */
/*-----------------------------------------------------------------------------*/

long
PresetCurrentPosition( vexPreset *v )
{
    if( v->preset_ptr != (-1) )
        return( v->presets[ v->preset_ptr ] );
    else
        return(0);
}

/*-----------------------------------------------------------------------------*/
/** @brief  check that the preset pointer is valid                             */
/*-----------------------------------------------------------------------------*/
int
PresetIsValid( vexPreset *v )
{
    return( v->preset_ptr );
}
