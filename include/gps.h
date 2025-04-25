#ifndef GPS_H
#define GPS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>

/* GPS NMEA sentence types */
#define NMEA_UNKNOWN       0
#define NMEA_GGA           1  /* Global Positioning System Fix Data */
#define NMEA_RMC           2  /* Recommended Minimum Navigation Information */
#define NMEA_GSA           3  /* GPS DOP and active satellites */
#define NMEA_GSV           4  /* Satellites in view */
#define NMEA_VTG           5  /* Track made good and ground speed */
#define NMEA_GLL           6  /* Geographic position, latitude / longitude */

/* GPS fix types */
#define GPS_FIX_NONE       0
#define GPS_FIX_2D         1
#define GPS_FIX_3D         2

/* GPS position structure */
typedef struct {
    float latitude;              /* Latitude in degrees (positive = North, negative = South) */
    float longitude;             /* Longitude in degrees (positive = East, negative = West) */
    float altitude;              /* Altitude in meters above mean sea level */
    float speed;                 /* Speed over ground in km/h */
    float course;                /* Course over ground in degrees from true north */
    uint8_t fix_type;            /* Fix type (0 = no fix, 1 = 2D fix, 2 = 3D fix) */
    uint8_t satellites;          /* Number of satellites used for fix */
    float hdop;                  /* Horizontal dilution of precision */
    uint32_t timestamp;          /* UTC timestamp (hhmmss.sss) */
    uint32_t date;               /* Date (ddmmyy) */
    uint8_t valid;               /* 1 if position is valid, 0 otherwise */
} GPSPosition_t;

/* GPS zone structure */
typedef struct {
    uint32_t id;                 /* Zone ID */
    float center_latitude;       /* Center latitude in degrees */
    float center_longitude;      /* Center longitude in degrees */
    float radius;                /* Zone radius in meters */
    uint8_t active;              /* 1 if zone is active, 0 otherwise */
} GPSZone_t;

/* GPS function prototypes */
BaseType_t xGPSInit(void);
BaseType_t xGPSGetPosition(GPSPosition_t *pxPosition);
BaseType_t xGPSAddZone(GPSZone_t *pxZone);
BaseType_t xGPSRemoveZone(uint32_t ulZoneID);
BaseType_t xGPSCheckZone(uint32_t ulZoneID, uint8_t *pucInZone);
BaseType_t xGPSGetDistance(float fLat1, float fLon1, float fLat2, float fLon2, float *pfDistance);
BaseType_t xGPSGetBearing(float fLat1, float fLon1, float fLat2, float fLon2, float *pfBearing);
BaseType_t xGPSEstimatePosition(GPSPosition_t *pxPosition);

#endif /* GPS_H */
