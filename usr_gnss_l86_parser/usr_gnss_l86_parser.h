/*
 * usr_gnss_l86_parser.h
 *
 *  Created on: Mar 16, 2024
 *      Author: numan
 */

#ifndef USR_GNSS_L86_PARSER_H_
#define USR_GNSS_L86_PARSER_H_
#include "stm32f0xx_hal.h"
#include "usr_gnssGeneral.h"

typedef bool _f; // flag type

typedef struct S_GPS_L86_DATA_TAG
{
    float lat;
    float lon;
    float speedKN;
    float timeDateBuf;
    float fixedTime;
    float fixedLatBaseFormat;
    float fixedLonBaseFormat;
    int fixQualityID;
    int satInUse;
    float hdop;
    float altitudeInMeter;
    float WGS84;

}S_GPS_L86_DATA;


void UsrGpsL86Init();
/*
 * L86 initial
 * Starts the dma
 *
 * */
void Usr_GpsL86GetValues(S_GPS_L86_DATA *gpsData_);
/*
 * data set for L86
 *
   gpsData.lat;
   gpsData.lon;
   gpsData.hdop;
   gpsData.speedKN;
   gpsData.satInUse;
   gpsData.timeDateBuf;
   gpsData.fixQualityID;
   gpsData.altitudeInMeter;

 * 2nd structure for the usr
 * */

#endif /* USR_GNSS_L86_PARSER_H_ */
