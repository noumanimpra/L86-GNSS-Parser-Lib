/*
 * usr_gnss_l86_parser.c
 *
 *  Created on: Mar 16, 2024
 *      Author: numan
 */

#include "usr_gnss_l86_parser.h"
#define DMA_READ_DEF_SIZE 600
#define _buffer_size 1024
#define _max_message_size 90

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

typedef enum
{
    NO_ERROR_STATE,
    INITIAL_ERROR,
    RX_ERROR,
    NORTH_EAST_ERROR,
    CONNECTION_ERROR
} ErrorCode;

_f g_GnssRx_Flag = false;
_f g_openFixedDataTransmition = false;
_f rmcValidFlag = false;
_f f_northFlag = false;
_f f_eastFlag = false;

_io int MsgIndex;
_io char *ptr;
_io char gpsPayload[100];
_io char gpsGGAPayload[100];
char m_rxData[_buffer_size];
_io char m_gpsTransmitBuf[_buffer_size];
_io void getRmc(void);
_io void getGGA(void);
_io void formatLatLong(void);

int lati = 0;
_io float m_nonFormattedLat;
_io float m_nonFormattedLon;
float decimal_lat, decimal_lon;

S_GPS_L86_DATA gpsData;
ErrorCode error = NO_ERROR_STATE;

/*const char *gnssErrHandler(ErrorCode code) // errCodesIfNeeded
{
    switch (code)
    {
    case 0:
        __disable_irq();
        while (1)
            ;
    case 1:
        // logg
        break;
    case 2:
        // data clear + log
        break;
    case 3:
        __disable_irq();
        while (1)
            ;
    default:
        return "Invalid error code";
    }
}*/

//============================= Callback section
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    g_GnssRx_Flag = true;
    g_openFixedDataTransmition = true;
}

//============================= Initial section

void UsrGpsL86Init(void)
{
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)m_rxData, DMA_READ_DEF_SIZE);
}

//============================= public L86 mechanism

void Usr_GpsL86GetValues(S_GPS_L86_DATA *gpsData_)
{
    getRmc();
    getGGA();
    if (rmcValidFlag)
    {
        rmcValidFlag = false;
    }
    formatLatLong();

    gpsData_->lat = gpsData.lat;
    gpsData_->lon = gpsData.lon;
    gpsData_->hdop = gpsData.hdop;
    gpsData_->speedKN = gpsData.speedKN;
    gpsData_->satInUse = gpsData.satInUse;
    gpsData_->timeDateBuf = gpsData.timeDateBuf;
    gpsData_->fixQualityID = gpsData.fixQualityID;
    gpsData_->altitudeInMeter = gpsData.altitudeInMeter;
}

//============================= Statics

_io void getRmc(void)
{
    if (g_GnssRx_Flag)
    {
        MsgIndex = 0;
        strcpy(m_gpsTransmitBuf, (char *)(m_rxData));
        ptr = strstr(m_gpsTransmitBuf, "GNRMC");

        if (*ptr == 'G')
        {
            while (1)
            {
                gpsPayload[MsgIndex] = *ptr;
                MsgIndex++;
                *ptr = *(ptr + MsgIndex);
                if (*ptr == '\n' || MsgIndex > _max_message_size)
                {
                    MsgIndex = 0;
                    memset(m_gpsTransmitBuf, 0, sizeof(m_gpsTransmitBuf));
                    memset(m_rxData, 0, sizeof(m_rxData));

                    for (int i = 0; i < 100; i++)
                    {
                        if (gpsPayload[i] == 'N')
                            f_northFlag = true;
                        if (gpsPayload[i] == 'E')
                            f_eastFlag = true;
                    }
                    if (strlen(gpsPayload) > 10)
                    {
                        if (f_eastFlag && f_northFlag)
                        {
                            f_northFlag = false;
                            f_eastFlag = false;
                            //&gpsData.lat
                            sscanf(gpsPayload, "GNRMC,%f,A,%f,N,%f,E,%f,", &gpsData.timeDateBuf, &m_nonFormattedLat, &m_nonFormattedLon, &gpsData.speedKN);
                            rmcValidFlag = true;
                            formatLatLong();
                        }
                        else
                        {
                            // connErr Log
                        }
                    }
                    else
                    {
                        // dataErr Log
                        memset(gpsPayload, 0, sizeof(gpsPayload));
                    }

                    break;
                }
            }
        }
        g_GnssRx_Flag = false;
    }
}

_io void formatLatLong(void)
{
    int degrees = (int)m_nonFormattedLat / 100;        // dec
    float minutes = m_nonFormattedLat - degrees * 100; // min
    gpsData.lat = degrees + (minutes / 60);            // dec to deg

    degrees = (int)m_nonFormattedLon / 100;
    minutes = m_nonFormattedLon - degrees * 100;
    gpsData.lon = degrees + (minutes / 60);
}

_io void getGGA(void)
{
    if (g_openFixedDataTransmition)
    {
        MsgIndex = 0;
        strcpy(m_gpsTransmitBuf, (char *)(m_rxData));
        ptr = strstr(m_gpsTransmitBuf, "GPGGA");

        if (*ptr == 'G')
        {
            while (1)
            {
                gpsGGAPayload[MsgIndex] = *ptr;
                MsgIndex++;
                *ptr = *(ptr + MsgIndex);
                if (*ptr == '\n' || MsgIndex > _max_message_size)
                {
                    MsgIndex = 0;
                    memset(m_gpsTransmitBuf, 0, sizeof(m_gpsTransmitBuf));
                    memset(m_rxData, 0, sizeof(m_rxData));

                    if (strlen(gpsGGAPayload) > 10)
                    {
                        sscanf(gpsGGAPayload, "GPGGA,%f,%f,N,%f,E,%d,%d,%f,%f,M,%f,M,", &gpsData.fixedTime, &gpsData.fixedLatBaseFormat, &gpsData.fixedLonBaseFormat, &gpsData.fixQualityID, &gpsData.satInUse, &gpsData.hdop, &gpsData.altitudeInMeter, &gpsData.WGS84);
                    }
                    else
                    {
                        memset(gpsPayload, 0, sizeof(gpsPayload));
                    }
                    break;
                }
            }
        }
        g_openFixedDataTransmition = false;
    }
}
