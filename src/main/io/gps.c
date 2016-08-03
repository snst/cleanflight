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
#include <ctype.h>
#include <string.h>
#include <math.h>

#include <platform.h>
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"

#include "drivers/dma.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/gps_i2cnav.h"

#include "sensors/sensors.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/fc_serial.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/gps.h"

#include "flight/gps_conversion.h"
#include "flight/pid.h"
#include "flight/navigation.h"


#ifdef GPS

PG_REGISTER_WITH_RESET_FN(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

void pgResetFn_gpsConfig(gpsConfig_t *instance)
{
    instance->autoConfig = GPS_AUTOCONFIG_ON;
}

#define LOG_ERROR        '?'
#define LOG_IGNORED      '!'
#define LOG_SKIPPED      '>'
#define LOG_NMEA_GGA     'g'
#define LOG_NMEA_RMC     'r'
#define LOG_UBLOX_SOL    'O'
#define LOG_UBLOX_STATUS 'S'
#define LOG_UBLOX_SVINFO 'I'
#define LOG_UBLOX_POSLLH 'P'
#define LOG_UBLOX_VELNED 'V'

#define GPS_SV_MAXSATS   16

char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
static char *gpsPacketLogChar = gpsPacketLog;

uint16_t gpsConstrainEPE(uint32_t epe)
{
    return (epe > 9999) ? 9999 : epe;
}

// **********************
// GPS
// **********************
int32_t GPS_coord[2];               // LAT/LON

uint8_t GPS_numSat;
uint16_t GPS_hdop = 9999;           // Compute GPS quality signal
uint32_t GPS_packetCount = 0;
uint32_t GPS_svInfoReceivedCount = 0; // SV = Space Vehicle, counter increments each time SV info is received.
uint8_t GPS_update = 0;             // it's a binary toggle to distinct a GPS position update

uint16_t GPS_altitude;              // altitude in 0.1m
uint16_t GPS_speed;                 // speed in 0.1m/s
uint16_t GPS_ground_course = 0;     // degrees * 10

uint8_t GPS_numCh;                          // Number of channels
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS];     // Channel number
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS];    // Satellite ID
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS];     // Carrier to Noise Ratio (Signal Strength)

uint32_t GPS_garbageByteCount = 0;

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in gpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)



typedef struct gpsInitData_s {
    uint8_t index;
    uint8_t baudrateIndex; // see baudRate_e
    const char *ubx;
    const char *mtk;
} gpsInitData_t;

// NMEA will cycle through these until valid data is received
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUDRATE_115200,  BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUDRATE_57600,    BAUD_57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
    { GPS_BAUDRATE_38400,    BAUD_38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
    { GPS_BAUDRATE_19200,    BAUD_19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
    // 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
    { GPS_BAUDRATE_9600,      BAUD_9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n", "" }
};

#define GPS_INIT_DATA_ENTRY_COUNT (sizeof(gpsInitData) / sizeof(gpsInitData[0]))



typedef enum {
    GPS_UNKNOWN,
    GPS_INITIALIZING,
    GPS_CHANGE_BAUD,
    GPS_CONFIGURE,
    GPS_RECEIVING_DATA,
    GPS_LOST_COMMUNICATION,
} gpsState_e;

gpsData_t gpsData;


static void gpsSetState(gpsState_e state)
{
    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = millis();
    gpsData.messageState = GPS_MESSAGE_STATE_IDLE;
}

void gpsInit(void)
{
    gpsData.baudrateIndex = 0;
    gpsData.errors = 0;
    gpsData.timeouts = 0;

    memset(gpsPacketLog, 0x00, sizeof(gpsPacketLog));

    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsSetState(GPS_UNKNOWN);

    gpsData.lastMessage = millis();

    // signal GPS "thread" to initialize when it gets to it
    gpsSetState(GPS_INITIALIZING);
}


void gpsInitHardware(void)
{
}

void gpsThread(void)
{
    gpsDataI2CNAV_t gpsMsg;
		
    i2cnavGPSModuleRead(&gpsMsg);
//	debug[1] ++;
	
    if (gpsMsg.flags.gpsOk) 
	{
//		debug[2] ++;
		gpsSetState(GPS_RECEIVING_DATA);
		
        if(gpsMsg.flags.fix3D) 
		{
            ENABLE_STATE(GPS_FIX);
        } else 
		{
            DISABLE_STATE(GPS_FIX);
        }
		
        // sat count
        GPS_numSat = gpsMsg.numSat;

        // Other data
        if (gpsMsg.flags.newData) 
		{
//			debug[3] ++;
//            if (gpsMsg.flags.fix3D) 
			{
                GPS_hdop = gpsConstrainEPE(gpsMsg.hdop);  // i2c-nav doesn't give vdop data, fake it using hdop
                GPS_speed = gpsMsg.speed;
                GPS_ground_course = gpsMsg.ground_course;
                GPS_coord[LAT] = gpsMsg.latitude;
                GPS_coord[LON] = gpsMsg.longitude;
                GPS_altitude = gpsMsg.altitude;
            }

			GPS_packetCount++;
			// new data received and parsed, we're in business
			gpsData.lastLastMessage = gpsData.lastMessage;
			gpsData.lastMessage = millis();
			sensorsSet(SENSOR_GPS);
		}
    }

//	DISABLE_STATE(GPS_FIX);		

    GPS_update = GPS_update ? 0 : 1;

	if (millis() - gpsData.lastMessage > GPS_TIMEOUT) 
	{
		// remove GPS from capability
		sensorsClear(SENSOR_GPS);
		gpsSetState(GPS_LOST_COMMUNICATION);
	}

    onNewGPSData();
}



//void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
//{
//}

void updateGpsIndicator(uint32_t currentTime)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (GPS_numSat >= 5)) {
        GPSLEDTime = currentTime + 150000;
        LED1_TOGGLE;
    }
}
#endif
