#ifndef DATA_H_
#define DATA_H_


#define I2C_GPS_ADDRESS    0x20
#define I2C_GPS_STATUS      01
#define I2C_GPS_DATA      02
#define I2C_SONAR_DATA    03

typedef struct 
{
  uint8_t    new_data:1;
  uint8_t    gps2dfix:1;
  uint8_t    gps3dfix:1;
  uint8_t    reserved:1;
  uint8_t    numsats:4;
} __attribute__ ((__packed__))  STATUS_REGISTER;


typedef struct 
{
    uint32_t              lat;
    uint32_t              lon;
    uint16_t              ground_speed;             // ground speed from gps m/s*100
    uint16_t              ground_course;            // GPS ground course
    int16_t               altitude;                 // gps altitude
    uint16_t              hdop;
    uint32_t              time;
    STATUS_REGISTER       status;                   // 0x00  status register
} __attribute__ ((__packed__))  I2C_REGISTERS;

#endif
