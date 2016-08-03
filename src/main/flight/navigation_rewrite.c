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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"
//#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"
#include "fc/runtime_config.h"

#if defined(NAV)
navigationPosControl_t  posControl;
#endif  // NAV
/*-----------------------------------------------------------
 * Processes an update to XY-position and velocity
 *-----------------------------------------------------------*/
void updateActualHorizontalPositionAndVelocity(bool hasValidSensor, float newX, float newY, float newVelX, float newVelY)
{
    posControl.actualState.pos.V.X = newX;
    posControl.actualState.pos.V.Y = newY;

    posControl.actualState.vel.V.X = newVelX;
    posControl.actualState.vel.V.Y = newVelY;
    posControl.actualState.velXY = sqrtf(sq(newVelX) + sq(newVelY));

    posControl.flags.hasValidPositionSensor = hasValidSensor;
    posControl.flags.hasValidHeadingSensor = isImuHeadingValid();

    if (hasValidSensor) {
        posControl.flags.horizontalPositionDataNew = 1;
        posControl.lastValidPositionTimeMs = millis();
    }
    else {
        posControl.flags.horizontalPositionDataNew = 0;
    }

#if defined(NAV_BLACKBOX)
    navLatestActualPosition[X] = newX;
    navLatestActualPosition[Y] = newY;
    navActualVelocity[X] = constrain(newVelX, -32678, 32767);
    navActualVelocity[Y] = constrain(newVelY, -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Processes an update to Z-position and velocity
 *-----------------------------------------------------------*/
void updateActualAltitudeAndClimbRate(bool hasValidSensor, float newAltitude, float newVelocity)
{
    posControl.actualState.pos.V.Z = newAltitude;
    posControl.actualState.vel.V.Z = newVelocity;

    posControl.flags.hasValidAltitudeSensor = hasValidSensor;

    // Update altitude that would be used when executing RTH
    if (hasValidSensor) {
//S        updateDesiredRTHAltitude();
        posControl.flags.verticalPositionDataNew = 1;
        posControl.lastValidAltitudeTimeMs = millis();
    }
    else {
        posControl.flags.verticalPositionDataNew = 0;
    }

#if defined(NAV_BLACKBOX)
    navLatestActualPosition[Z] = constrain(newAltitude, -32678, 32767);
    navActualVelocity[Z] = constrain(newVelocity, -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Processes an update to surface distance
 *-----------------------------------------------------------*/
void updateActualSurfaceDistance(bool hasValidSensor, float surfaceDistance, float surfaceVelocity)
{
    posControl.actualState.surface = surfaceDistance;
    posControl.actualState.surfaceVel = surfaceVelocity;

    if (ARMING_FLAG(ARMED)) {
        if (surfaceDistance > 0) {
            if (posControl.actualState.surfaceMin > 0) {
                posControl.actualState.surfaceMin = MIN(posControl.actualState.surfaceMin, surfaceDistance);
            }
            else {
                posControl.actualState.surfaceMin = surfaceDistance;
            }
        }
    }
    else {
        posControl.actualState.surfaceMin = -1;
    }

    posControl.flags.hasValidSurfaceSensor = hasValidSensor;

    if (hasValidSensor) {
        posControl.flags.surfaceDistanceDataNew = 1;
    }
    else {
        posControl.flags.surfaceDistanceDataNew = 0;
    }

#if defined(NAV_BLACKBOX)
    navActualSurface = surfaceDistance;
#endif
}

/*-----------------------------------------------------------
 * Processes an update to estimated heading
 *-----------------------------------------------------------*/
void updateActualHeading(int32_t newHeading)
{
    /* Update heading */
    posControl.actualState.yaw = newHeading;

    /* Precompute sin/cos of yaw angle */
    posControl.actualState.sinYaw = sin_approx(CENTIDEGREES_TO_RADIANS(newHeading));
    posControl.actualState.cosYaw = cos_approx(CENTIDEGREES_TO_RADIANS(newHeading));

    posControl.flags.headingDataNew = 1;
}
/*-----------------------------------------------------------
 * NAV main control functions
 *-----------------------------------------------------------*/
void navigationUseConfig(navConfig_t *navConfigToUse)
{
    posControl.navConfig = navConfigToUse;
}


void resetNavConfig(navConfig_t * navConfig)
{
    // Navigation flags
/*    navConfig->flags.use_thr_mid_for_althold = 0;
    navConfig->flags.extra_arming_safety = 1;
    navConfig->flags.user_control_mode = NAV_GPS_ATTI;
    navConfig->flags.rth_alt_control_style = NAV_RTH_AT_LEAST_ALT;
    navConfig->flags.rth_tail_first = 0;
    navConfig->flags.disarm_on_landing = 0;
*/
    // Inertial position estimator parameters
#if defined(NAV_AUTO_MAG_DECLINATION)
    navConfig->inav.automatic_mag_declination = 1;
#endif
    navConfig->inav.gps_min_sats = 6;
    navConfig->inav.gps_delay_ms = 200;
    navConfig->inav.accz_unarmed_cal = 1;
    navConfig->inav.use_gps_velned = 1;         // "Disabled" is mandatory with gps_dyn_model = Pedestrian

    navConfig->inav.w_z_baro_p = 0.35f;

    navConfig->inav.w_z_gps_p = 0.2f;
    navConfig->inav.w_z_gps_v = 0.5f;

    navConfig->inav.w_xy_gps_p = 1.0f;
    navConfig->inav.w_xy_gps_v = 2.0f;

    navConfig->inav.w_z_res_v = 0.5f;
    navConfig->inav.w_xy_res_v = 0.5f;

    navConfig->inav.w_acc_bias = 0.01f;

    navConfig->inav.max_eph_epv = 1000.0f;
    navConfig->inav.baro_epv = 100.0f;
/*
    // General navigation parameters
    navConfig->pos_failure_timeout = 5;     // 5 sec
    navConfig->waypoint_radius = 100;       // 2m diameter
    navConfig->max_speed = 300;             // 3 m/s = 10.8 km/h
    navConfig->max_climb_rate = 500;        // 5 m/s
    navConfig->max_manual_speed = 500;
    navConfig->max_manual_climb_rate = 200;
    navConfig->land_descent_rate = 200;     // 2 m/s
    navConfig->land_slowdown_minalt = 500;  // 5 meters of altitude
    navConfig->land_slowdown_maxalt = 2000; // 20 meters of altitude
    navConfig->emerg_descent_rate = 500;    // 5 m/s
    navConfig->min_rth_distance = 500;      // If closer than 5m - land immediately
    navConfig->rth_altitude = 1000;         // 10m

    // MC-specific
    navConfig->mc_max_bank_angle = 30;      // 30 deg
    navConfig->mc_hover_throttle = 1500;
    navConfig->mc_auto_disarm_delay = 2000;
*/
}

navConfig_t initialnavConfig;

void navigationInit()
{
	resetNavConfig(&initialnavConfig);
    // Initial state
    //posControl.navState = NAV_STATE_IDLE;

    posControl.flags.horizontalPositionDataNew = 0;
    posControl.flags.verticalPositionDataNew = 0;
    posControl.flags.surfaceDistanceDataNew = 0;
    posControl.flags.headingDataNew = 0;

    posControl.flags.hasValidAltitudeSensor = 0;
    posControl.flags.hasValidPositionSensor = 0;
    posControl.flags.hasValidSurfaceSensor = 0;
    posControl.flags.hasValidHeadingSensor = 0;

    posControl.flags.forcedRTHActivated = 0;
    //posControl.waypointCount = 0;
    //posControl.activeWaypointIndex = 0;
    //posControl.waypointListValid = false;

    /* Set initial surface invalid */
    posControl.actualState.surface = -1.0f;
    posControl.actualState.surfaceVel = 0.0f;
    posControl.actualState.surfaceMin = -1.0f;

    /* Use system config */
    navigationUseConfig(&initialnavConfig);
//    navigationUsePIDs(initialPidProfile);
//    navigationUseRcControlsConfig(initialRcControlsConfig);
//    navigationUseRxConfig(initialRxConfig);
//    navigationUseEscAndServoConfig(initialEscAndServoConfig);
//    navigationUseFlight3DConfig(initialFlight3DConfig);
}