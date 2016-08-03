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
#include "build/debug.h"

#include "gps_i2cnav.h"
#include "gps_i2c_data.h"

#include "gpio.h"
#include "system.h"
#include "bus_i2c.h"


bool i2cnavGPSModuleDetect(void)
{
    bool ack;
    uint8_t i2cGpsStatus;
    
    ack = i2cRead(I2C_GPS_ADDRESS, I2C_GPS_STATUS, 1, &i2cGpsStatus); /* status register */ 
    
    if (ack) 
        return true;
    
    return false;
}

void i2cnavGPSModuleRead(gpsDataI2CNAV_t * gpsMsg)
{
    bool ack;
    I2C_REGISTERS i2cGpsData;
    
    gpsMsg->flags.newData = 0;
    gpsMsg->flags.fix3D = 0;
    gpsMsg->flags.gpsOk = 0;
    
    ack = i2cRead(I2C_GPS_ADDRESS, I2C_GPS_DATA, sizeof(I2C_REGISTERS), (uint8_t*)&i2cGpsData); 

    if (!ack)
	{
		gpsMsg->flags.gpsOk = 0;
        return;
	}

    gpsMsg->flags.gpsOk = 1;
	gpsMsg->flags.newData = i2cGpsData.status.new_data;
    gpsMsg->numSat = i2cGpsData.status.numsats;
	gpsMsg->flags.fix3D = i2cGpsData.status.gps3dfix;
	gpsMsg->latitude = i2cGpsData.lat;
	gpsMsg->longitude = i2cGpsData.lon;
	gpsMsg->speed = i2cGpsData.ground_speed;
	gpsMsg->ground_course = i2cGpsData.ground_course;
	gpsMsg->altitude = i2cGpsData.altitude;
	gpsMsg->hdop = i2cGpsData.hdop;
//	debug[2] = i2cGpsData.time;
}
