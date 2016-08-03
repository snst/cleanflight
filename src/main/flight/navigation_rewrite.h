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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>
#include "common/axis.h"
#include "build/build_config.h"


typedef struct navConfig_s {
/*
    struct {
        uint8_t use_thr_mid_for_althold;    // Don't remember throttle when althold was initiated, assume that throttle is at Thr Mid = zero climb rate
        uint8_t extra_arming_safety;        // Forcibly apply 100% throttle tilt compensation
        uint8_t user_control_mode;          // NAV_GPS_ATTI or NAV_GPS_CRUISE
        uint8_t rth_alt_control_style;      // Controls how RTH controls altitude
        uint8_t rth_tail_first;             // Return to home tail first
        uint8_t disarm_on_landing;          // 
    } flags;
*/
    struct {
#if defined(NAV_AUTO_MAG_DECLINATION)
        uint8_t automatic_mag_declination;
#endif
        uint8_t gps_min_sats;
        uint8_t accz_unarmed_cal;
        uint8_t use_gps_velned;
        uint16_t gps_delay_ms;

        float w_z_baro_p;   // Weight (cutoff frequency) for barometer altitude measurements

        float w_z_sonar_p;  // Weight (cutoff frequency) for sonar altitude measurements
        float w_z_sonar_v;  // Weight (cutoff frequency) for sonar velocity measurements

        float w_z_gps_p;    // GPS altitude data is very noisy and should be used only on airplanes
        float w_z_gps_v;    // Weight (cutoff frequency) for GPS climb rate measurements

        float w_xy_gps_p;   // Weight (cutoff frequency) for GPS position measurements
        float w_xy_gps_v;   // Weight (cutoff frequency) for GPS velocity measurements

        float w_z_res_v;    // When velocity sources lost slowly decrease estimated velocity with this weight
        float w_xy_res_v;

        float w_acc_bias;   // Weight (cutoff frequency) for accelerometer bias estimation. 0 to disable.

        float max_eph_epv;  // Max estimated position error acceptable for estimation (cm)
        float baro_epv;     // Baro position error
    } inav;
/*
    uint8_t  pos_failure_timeout;           // Time to wait before switching to emergency landing (0 - disable)
    uint16_t waypoint_radius;               // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint16_t max_speed;                     // autonomous navigation speed cm/sec
    uint16_t max_climb_rate;                // max vertical speed limitation cm/sec
    uint16_t max_manual_speed;              // manual velocity control max horizontal speed
    uint16_t max_manual_climb_rate;         // manual velocity control max vertical speed
    uint16_t land_descent_rate;             // normal RTH landing descent rate
    uint16_t land_slowdown_minalt;          // Altitude to stop lowering descent rate during RTH descend
    uint16_t land_slowdown_maxalt;          // Altitude to start lowering descent rate during RTH descend
    uint16_t emerg_descent_rate;            // emergency landing descent rate
    uint16_t rth_altitude;                  // altitude to maintain when RTH is active (depends on rth_alt_control_style) (cm)
    uint16_t min_rth_distance;              // 0 Disables. Minimal distance for RTL in cm, otherwise it will just autoland

    uint8_t  mc_max_bank_angle;             // multicopter max banking angle (deg)
    uint16_t mc_hover_throttle;             // multicopter hover throttle
    uint16_t mc_auto_disarm_delay;          // multicopter safety delay for landing detector
*/
} navConfig_t;

typedef struct gpsOrigin_s {
    bool    valid;
    float   scale;
    int32_t lat;    // Lattitude * 1e+7
    int32_t lon;    // Longitude * 1e+7
    int32_t alt;    // Altitude in centimeters (meters * 100)
} gpsOrigin_s;
void navigationUseConfig(navConfig_t *navConfigToUse);
void updatePositionEstimator_BaroTopic(uint32_t currentTime);
void updatePositionEstimator_SonarTopic(uint32_t currentTime);
void updatePositionEstimator(void);
extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                // it's the angles that must be applied for GPS correction

/* Navigation system updates */
void onNewGPSData(void);
void resetNavConfig(navConfig_t * navConfig);


