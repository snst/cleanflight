/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/maths.h"

#if defined(NAV)

#define HZ2US(hz)   (1000000 / (hz))
#define US2S(us)    ((us) * 1e-6f)
#define MS2US(ms)   ((ms) * 1000)
#define HZ2S(hz)    US2S(HZ2US(hz))

typedef struct navigationFlags_s {
    bool horizontalPositionDataNew;
    bool verticalPositionDataNew;
    bool surfaceDistanceDataNew;
    bool headingDataNew;

    bool horizontalPositionDataConsumed;
    bool verticalPositionDataConsumed;

    bool hasValidAltitudeSensor;        // Indicates that we have a working altitude sensor (got at least one valid reading from it)
    bool hasValidPositionSensor;        // Indicates that GPS is working (or not)
    bool hasValidSurfaceSensor;
    bool hasValidHeadingSensor;         // Indicate valid heading - wither mag or GPS at certain speed on airplane

    bool isAdjustingPosition;
    bool isAdjustingAltitude;
    bool isAdjustingHeading;

    // Behaviour modifiers
    bool isGCSAssistedNavigationEnabled;    // Does iNav accept WP#255 - follow-me etc.
    bool isTerrainFollowEnabled;            // Does iNav use sonar for terrain following (adjusting baro altitude target according to sonar readings)

    bool forcedRTHActivated;
} navigationFlags_t;

typedef struct {
    t_fp_vector pos;
    t_fp_vector vel;
    int32_t     yaw;
    float       sinYaw;
    float       cosYaw;
    float       surface;
    float       surfaceVel;
    float       surfaceMin;
    float       velXY;
} navigationEstimatedState_t;


typedef struct {
/*
    // Flags and navigation system state
    navigationFSMState_t        navState;
*/
    navigationFlags_t           flags;
/*
    // Navigation PID controllers + pre-computed flight parameters
    navigationPIDControllers_t  pids;
    float                       posDecelerationTime;
    float                       posResponseExpo;
*/
    // Local system state, both actual (estimated) and desired (target setpoint)
    navigationEstimatedState_t  actualState;
//    navigationDesiredState_t    desiredState;   // waypoint coordinates + velocity

    uint32_t                    lastValidPositionTimeMs;
    uint32_t                    lastValidAltitudeTimeMs;

    // INAV GPS origin (position where GPS fix was first acquired)
    gpsOrigin_s                 gpsOrigin;
/*
    // Home parameters (NEU coordinated), geodetic position of home (LLH) is stores in GPS_home variable
    navWaypointPosition_t       homePosition;       // Special waypoint, stores original yaw (heading when launched)
    navWaypointPosition_t       homeWaypointAbove;  // NEU-coordinates and initial bearing + desired RTH altitude

    uint32_t                    homeDistance;   // cm
    int32_t                     homeDirection;  // deg*100

    // Waypoint list 
    navWaypoint_t               waypointList[NAV_MAX_WAYPOINTS];
    bool                        waypointListValid;
    int8_t                      waypointCount;

    navWaypointPosition_t       activeWaypoint;     // Local position and initial bearing, filled on waypoint activation
    int8_t                      activeWaypointIndex;

    // Internals 
    int16_t                     rcAdjustment[4];
*/	

    navConfig_t *               navConfig;
/*
    rcControlsConfig_t *        rcControlsConfig;
    pidProfile_t *              pidProfile;
    rxConfig_t *                rxConfig;
    flight3DConfig_t *          flight3DConfig;
    escAndServoConfig_t *       escAndServoConfig;
*/
} navigationPosControl_t;

extern navigationPosControl_t posControl;

void updateActualHeading(int32_t newHeading);
void updateActualHorizontalPositionAndVelocity(bool hasValidSensor, float newX, float newY, float newVelX, float newVelY);
void updateActualAltitudeAndClimbRate(bool hasValidSensor, float newAltitude, float newVelocity);
void updateActualSurfaceDistance(bool hasValidSensor, float surfaceDistance, float surfaceVelocity);
#endif
