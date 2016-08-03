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

#include <platform.h>
#include "build/build_config.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"
#include "bus_i2c.h"
#include "sensors/sonar.h"

#include "sonar_i2c.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

#if defined(SONAR_I2C)

STATIC_UNIT_TESTED volatile int32_t sonar_distance = -1;
//static uint32_t lastMeasurementAt;

#define I2C_GPS_ADDRESS     0x20 //7 bits       
#define I2C_GPS_STATUS      01
#define I2C_GPS_DATA        02
#define I2C_SONAR_DATA      03

bool i2cSonarModuleDetect(void)
{
    int16_t distance;
    bool ack = i2cRead(I2C_GPS_ADDRESS, I2C_SONAR_DATA, sizeof(distance), (uint8_t *)&distance);  
    return ack;
}

void sonar_i2c_init(sonarRange_t *sonarRange)
{
    sonarRange->maxRangeCm = HCSR04_MAX_RANGE_CM;
    sonarRange->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    sonarRange->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;
}

void sonar_i2c_start_reading(void)
{
    int16_t distance;
    bool ack = i2cRead(I2C_GPS_ADDRESS, I2C_SONAR_DATA, sizeof(distance), (uint8_t *)&distance);  
	sonar_distance = ack ? distance : SONAR_OUT_OF_RANGE;
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t sonar_i2c_get_distance(void)
{
    return sonar_distance;
}
#endif
