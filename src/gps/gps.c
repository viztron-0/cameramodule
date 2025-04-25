#include "gps.h"
#include "task_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Include Raspberry Pi Pico specific headers */
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

/* UART instance for GPS module */
#define GPS_UART_INSTANCE          uart1
#define GPS_UART_TX_PIN            8
#define GPS_UART_RX_PIN            9
#define GPS_UART_BAUD_RATE         9600

/* GPS buffer sizes */
#define GPS_RX_BUFFER_SIZE         256
#define GPS_NMEA_MAX_LENGTH        82
#define MAX_GPS_ZONES              10

/* GPS position history for filtering */
#define GPS_POSITION_HISTORY_SIZE  10

/* Earth radius in meters */
#define EARTH_RADIUS_METERS        6371000.0f

/* Static variables */
static GPSPosition_t xCurrentPosition;
static GPSPosition_t xPositionHistory[GPS_POSITION_HISTORY_SIZE];
static uint8_t ucPositionHistoryIndex = 0;
static uint8_t ucPositionHistoryCount = 0;
static GPSZone_t xZones[MAX_GPS_ZONES];
static uint8_t ucNumZones = 0;
static uint8_t ucGPSInitialized = 0;
static uint32_t ulLastValidFix = 0;

/* UART receive buffer */
static uint8_t ucRxBuffer[GPS_RX_BUFFER_SIZE];
static uint16_t usRxBufferHead = 0;
static uint16_t usRxBufferTail = 0;

/* Forward declarations of static functions */
static void prvGPSUARTInit(void);
static void prvGPSUARTRxCallback(void);
static BaseType_t prvGPSProcessNMEA(char *pcNMEASentence);
static BaseType_t prvGPSParseGGA(char *pcNMEASentence);
static BaseType_t prvGPSParseRMC(char *pcNMEASentence);
static void prvGPSFilterPosition(void);
static float prvGPSDegreesToRadians(float fDegrees);
static float prvGPSRadiansToDegrees(float fRadians);
static void prvGPSAddToPositionHistory(GPSPosition_t *pxPosition);
static uint8_t prvGPSChecksum(const char *pcNMEASentence);

/**
 * @brief Initialize the GPS module
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSInit(void)
{
    /* Initialize UART for GPS module */
    prvGPSUARTInit();
    
    /* Initialize GPS data structures */
    memset(&xCurrentPosition, 0, sizeof(GPSPosition_t));
    memset(xPositionHistory, 0, sizeof(xPositionHistory));
    memset(xZones, 0, sizeof(xZones));
    
    ucPositionHistoryIndex = 0;
    ucPositionHistoryCount = 0;
    ucNumZones = 0;
    ucGPSInitialized = 1;
    
    printf("GPS module initialized\n");
    return pdPASS;
}

/**
 * @brief Get current GPS position
 * 
 * @param pxPosition Pointer to position structure to fill
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSGetPosition(GPSPosition_t *pxPosition)
{
    if (pxPosition == NULL || !ucGPSInitialized)
    {
        return pdFAIL;
    }
    
    /* Take mutex to ensure atomic access to position data */
    if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(100)) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Copy current position */
    memcpy(pxPosition, &xCurrentPosition, sizeof(GPSPosition_t));
    
    /* Release mutex */
    xSemaphoreGive(xUARTMutex);
    
    return pdPASS;
}

/**
 * @brief Add a GPS zone
 * 
 * @param pxZone Pointer to zone structure
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSAddZone(GPSZone_t *pxZone)
{
    if (pxZone == NULL || !ucGPSInitialized || ucNumZones >= MAX_GPS_ZONES)
    {
        return pdFAIL;
    }
    
    /* Check if zone with same ID already exists */
    for (int i = 0; i < ucNumZones; i++)
    {
        if (xZones[i].id == pxZone->id)
        {
            /* Update existing zone */
            memcpy(&xZones[i], pxZone, sizeof(GPSZone_t));
            return pdPASS;
        }
    }
    
    /* Add new zone */
    memcpy(&xZones[ucNumZones], pxZone, sizeof(GPSZone_t));
    ucNumZones++;
    
    return pdPASS;
}

/**
 * @brief Remove a GPS zone
 * 
 * @param ulZoneID Zone ID to remove
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSRemoveZone(uint32_t ulZoneID)
{
    if (!ucGPSInitialized)
    {
        return pdFAIL;
    }
    
    /* Find zone with matching ID */
    for (int i = 0; i < ucNumZones; i++)
    {
        if (xZones[i].id == ulZoneID)
        {
            /* Remove zone by shifting remaining zones */
            for (int j = i; j < ucNumZones - 1; j++)
            {
                memcpy(&xZones[j], &xZones[j + 1], sizeof(GPSZone_t));
            }
            
            ucNumZones--;
            return pdPASS;
        }
    }
    
    return pdFAIL;
}

/**
 * @brief Check if current position is within a zone
 * 
 * @param ulZoneID Zone ID to check
 * @param pucInZone Pointer to store result (1 if in zone, 0 otherwise)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSCheckZone(uint32_t ulZoneID, uint8_t *pucInZone)
{
    if (pucInZone == NULL || !ucGPSInitialized)
    {
        return pdFAIL;
    }
    
    *pucInZone = 0;
    
    /* Check if position is valid */
    if (!xCurrentPosition.valid)
    {
        return pdFAIL;
    }
    
    /* Find zone with matching ID */
    for (int i = 0; i < ucNumZones; i++)
    {
        if (xZones[i].id == ulZoneID && xZones[i].active)
        {
            float fDistance;
            
            /* Calculate distance to zone center */
            xGPSGetDistance(
                xCurrentPosition.latitude, xCurrentPosition.longitude,
                xZones[i].center_latitude, xZones[i].center_longitude,
                &fDistance
            );
            
            /* Check if within zone radius */
            if (fDistance <= xZones[i].radius)
            {
                *pucInZone = 1;
            }
            
            return pdPASS;
        }
    }
    
    return pdFAIL;
}

/**
 * @brief Calculate distance between two GPS coordinates
 * 
 * @param fLat1 Latitude of first point in degrees
 * @param fLon1 Longitude of first point in degrees
 * @param fLat2 Latitude of second point in degrees
 * @param fLon2 Longitude of second point in degrees
 * @param pfDistance Pointer to store distance in meters
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSGetDistance(float fLat1, float fLon1, float fLat2, float fLon2, float *pfDistance)
{
    if (pfDistance == NULL)
    {
        return pdFAIL;
    }
    
    /* Convert degrees to radians */
    float fLat1Rad = prvGPSDegreesToRadians(fLat1);
    float fLon1Rad = prvGPSDegreesToRadians(fLon1);
    float fLat2Rad = prvGPSDegreesToRadians(fLat2);
    float fLon2Rad = prvGPSDegreesToRadians(fLon2);
    
    /* Haversine formula */
    float fDeltaLat = fLat2Rad - fLat1Rad;
    float fDeltaLon = fLon2Rad - fLon1Rad;
    
    float fA = sinf(fDeltaLat / 2) * sinf(fDeltaLat / 2) +
               cosf(fLat1Rad) * cosf(fLat2Rad) *
               sinf(fDeltaLon / 2) * sinf(fDeltaLon / 2);
    
    float fC = 2 * atan2f(sqrtf(fA), sqrtf(1 - fA));
    
    /* Distance in meters */
    *pfDistance = EARTH_RADIUS_METERS * fC;
    
    return pdPASS;
}

/**
 * @brief Calculate bearing between two GPS coordinates
 * 
 * @param fLat1 Latitude of first point in degrees
 * @param fLon1 Longitude of first point in degrees
 * @param fLat2 Latitude of second point in degrees
 * @param fLon2 Longitude of second point in degrees
 * @param pfBearing Pointer to store bearing in degrees (0-360)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSGetBearing(float fLat1, float fLon1, float fLat2, float fLon2, float *pfBearing)
{
    if (pfBearing == NULL)
    {
        return pdFAIL;
    }
    
    /* Convert degrees to radians */
    float fLat1Rad = prvGPSDegreesToRadians(fLat1);
    float fLon1Rad = prvGPSDegreesToRadians(fLon1);
    float fLat2Rad = prvGPSDegreesToRadians(fLat2);
    float fLon2Rad = prvGPSDegreesToRadians(fLon2);
    
    /* Calculate bearing */
    float fY = sinf(fLon2Rad - fLon1Rad) * cosf(fLat2Rad);
    float fX = cosf(fLat1Rad) * sinf(fLat2Rad) -
               sinf(fLat1Rad) * cosf(fLat2Rad) * cosf(fLon2Rad - fLon1Rad);
    
    float fBearingRad = atan2f(fY, fX);
    
    /* Convert to degrees */
    *pfBearing = prvGPSRadiansToDegrees(fBearingRad);
    
    /* Normalize to 0-360 */
    *pfBearing = fmodf(*pfBearing + 360.0f, 360.0f);
    
    return pdPASS;
}

/**
 * @brief Estimate position when GPS is unavailable
 * 
 * @param pxPosition Pointer to position structure to fill
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xGPSEstimatePosition(GPSPosition_t *pxPosition)
{
    if (pxPosition == NULL || !ucGPSInitialized)
    {
        return pdFAIL;
    }
    
    /* Take mutex to ensure atomic access to position data */
    if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(100)) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Check if we have a valid position */
    if (xCurrentPosition.valid)
    {
        /* Copy current position */
        memcpy(pxPosition, &xCurrentPosition, sizeof(GPSPosition_t));
        xSemaphoreGive(xUARTMutex);
        return pdPASS;
    }
    
    /* Check if we have position history */
    if (ucPositionHistoryCount == 0)
    {
        xSemaphoreGive(xUARTMutex);
        return pdFAIL;
    }
    
    /* Simple estimation: use last known position and apply velocity */
    uint32_t ulTimeSinceLastFix = xTaskGetTickCount() - ulLastValidFix;
    float fTimeInSeconds = ulTimeSinceLastFix / 1000.0f;
    
    /* Limit estimation to 60 seconds */
    if (fTimeInSeconds > 60.0f)
    {
        xSemaphoreGive(xUARTMutex);
        return pdFAIL;
    }
    
    /* Get last valid position from history */
    int iLastValidIndex = (ucPositionHistoryIndex - 1 + GPS_POSITION_HISTORY_SIZE) % GPS_POSITION_HISTORY_SIZE;
    GPSPosition_t *pxLastPosition = &xPositionHistory[iLastValidIndex];
    
    /* Estimate new position based on last known speed and course */
    float fDistanceTraveled = pxLastPosition->speed * fTimeInSeconds / 3.6f;  /* km/h to m/s */
    float fBearingRad = prvGPSDegreesToRadians(pxLastPosition->course);
    
    /* Calculate new position using dead reckoning */
    float fLat1Rad = prvGPSDegreesToRadians(pxLastPosition->latitude);
    float fLon1Rad = prvGPSDegreesToRadians(pxLastPosition->longitude);
    
    float fAngularDistance = fDistanceTraveled / EARTH_RADIUS_METERS;
    
    float fLat2Rad = asinf(sinf(fLat1Rad) * cosf(fAngularDistance) +
                          cosf(fLat1Rad) * sinf(fAngularDistance) * cosf(fBearingRad));
    
    float fLon2Rad = fLon1Rad + atan2f(sinf(fBearingRad) * sinf(fAngularDistance) * cosf(fLat1Rad),
                                      cosf(fAngularDistance) - sinf(fLat1Rad) * sinf(fLat2Rad));
    
    /* Fill estimated position */
    pxPosition->latitude = prvGPSRadiansToDegrees(fLat2Rad);
    pxPosition->longitude = prvGPSRadiansToDegrees(fLon2Rad);
    pxPosition->altitude = pxLastPosition->altitude;
    pxPosition->speed = pxLastPosition->speed;
    pxPosition->course = pxLastPosition->course;
    pxPosition->fix_type = GPS_FIX_NONE;
    pxPosition->satellites = 0;
    pxPosition->hdop = 99.99f;  /* High value to indicate low precision */
    pxPosition->timestamp = pxLastPosition->timestamp;
    pxPosition->date = pxLastPosition->date;
    pxPosition->valid = 0;  /* Mark as invalid since it's an estimation */
    
    xSemaphoreGive(xUARTMutex);
    return pdPASS;
}

/**
 * @brief Initialize UART for GPS module
 */
static void prvGPSUARTInit(void)
{
    /* Initialize UART pins */
    gpio_set_function(GPS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_UART_RX_PIN, GPIO_FUNC_UART);
    
    /* Initialize UART with 9600 baud rate */
    uart_init(GPS_UART_INSTANCE, GPS_UART_BAUD_RATE);
    
    /* Set data format (8 bits, no parity, 1 stop bit) */
    uart_set_format(GPS_UART_INSTANCE, 8, 1, UART_PARITY_NONE);
    
    /* Enable FIFO */
    uart_set_fifo_enabled(GPS_UART_INSTANCE, true);
    
    /* Set up RX interrupt */
    uart_set_irq_enables(GPS_UART_INSTANCE, true, false);
    irq_set_exclusive_handler(UART1_IRQ, prvGPSUARTRxCallback);
    irq_set_enabled(UART1_IRQ, true);
    
    /* Initialize RX buffer */
    usRxBufferHead = 0;
    usRxBufferTail = 0;
}

/**
 * @brief UART receive interrupt callback
 */
static void prvGPSUARTRxCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Read data from UART FIFO */
    while (uart_is_readable(GPS_UART_INSTANCE))
    {
        uint8_t ucData = uart_getc(GPS_UART_INSTANCE);
        
        /* Store in circular buffer */
        ucRxBuffer[usRxBufferHead] = ucData;
        usRxBufferHead = (usRxBufferHead + 1) % GPS_RX_BUFFER_SIZE;
        
        /* Check for buffer overflow */
        if (usRxBufferHead == usRxBufferTail)
        {
            /* Buffer full, discard oldest byte */
            usRxBufferTail = (usRxBufferTail + 1) % GPS_RX_BUFFER_SIZE;
        }
        
        /* Check for end of NMEA sentence */
        if (ucData == '\n')
        {
            /* Notify GPS task that a sentence is ready */
            if (xGPSTaskHandle != NULL)
            {
                vTaskNotifyGiveFromISR(xGPSTaskHandle, &xHigherPriorityTaskWoken);
            }
        }
    }
    
    /* Yield if a higher priority task was woken */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Process NMEA sentence
 * 
 * @param pcNMEASentence NMEA sentence to process
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvGPSProcessNMEA(char *pcNMEASentence)
{
    /* Check for valid NMEA sentence */
    if (pcNMEASentence == NULL || pcNMEASentence[0] != '$')
    {
        return pdFAIL;
    }
    
    /* Verify checksum */
    if (!prvGPSChecksum(pcNMEASentence))
    {
        return pdFAIL;
    }
    
    /* Determine sentence type */
    if (strncmp(pcNMEASentence + 1, "GPGGA", 5) == 0 || 
        strncmp(pcNMEASentence + 1, "GNGGA", 5) == 0)
    {
        return prvGPSParseGGA(pcNMEASentence);
    }
    else if (strncmp(pcNMEASentence + 1, "GPRMC", 5) == 0 || 
             strncmp(pcNMEASentence + 1, "GNRMC", 5) == 0)
    {
        return prvGPSParseRMC(pcNMEASentence);
    }
    
    /* Unsupported sentence type */
    return pdFAIL;
}

/**
 * @brief Parse GGA sentence (Global Positioning System Fix Data)
 * 
 * @param pcNMEASentence NMEA sentence to parse
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvGPSParseGGA(char *pcNMEASentence)
{
    char *pcToken;
    char *pcSavePtr;
    int iField = 0;
    GPSPosition_t xNewPosition;
    
    /* Initialize new position with current values */
    memcpy(&xNewPosition, &xCurrentPosition, sizeof(GPSPosition_t));
    
    /* Tokenize sentence */
    pcToken = strtok_r(pcNMEASentence, ",", &pcSavePtr);
    
    while (pcToken != NULL)
    {
        switch (iField)
        {
            case 1:  /* Time (hhmmss.sss) */
                xNewPosition.timestamp = atoi(pcToken);
                break;
                
            case 2:  /* Latitude (ddmm.mmmm) */
                if (strlen(pcToken) > 0)
                {
                    float fLatitude = atof(pcToken);
                    int iDegrees = (int)(fLatitude / 100);
                    float fMinutes = fLatitude - (iDegrees * 100);
                    xNewPosition.latitude = iDegrees + (fMinutes / 60.0f);
                }
                break;
                
            case 3:  /* N/S indicator */
                if (pcToken[0] == 'S')
                {
                    xNewPosition.latitude = -xNewPosition.latitude;
                }
                break;
                
            case 4:  /* Longitude (dddmm.mmmm) */
                if (strlen(pcToken) > 0)
                {
                    float fLongitude = atof(pcToken);
                    int iDegrees = (int)(fLongitude / 100);
                    float fMinutes = fLongitude - (iDegrees * 100);
                    xNewPosition.longitude = iDegrees + (fMinutes / 60.0f);
                }
                break;
                
            case 5:  /* E/W indicator */
                if (pcToken[0] == 'W')
                {
                    xNewPosition.longitude = -xNewPosition.longitude;
                }
                break;
                
            case 6:  /* Fix quality */
                xNewPosition.fix_type = atoi(pcToken);
                xNewPosition.valid = (xNewPosition.fix_type > 0) ? 1 : 0;
                break;
                
            case 7:  /* Satellites used */
                xNewPosition.satellites = atoi(pcToken);
                break;
                
            case 8:  /* HDOP */
                xNewPosition.hdop = atof(pcToken);
                break;
                
            case 9:  /* Altitude */
                xNewPosition.altitude = atof(pcToken);
                break;
        }
        
        pcToken = strtok_r(NULL, ",*", &pcSavePtr);
        iField++;
    }
    
    /* Update position if valid */
    if (xNewPosition.valid)
    {
        /* Take mutex to ensure atomic access to position data */
        if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(100)) == pdPASS)
        {
            /* Add to position history */
            prvGPSAddToPositionHistory(&xNewPosition);
            
            /* Apply position filtering */
            prvGPSFilterPosition();
            
            /* Update last valid fix time */
            ulLastValidFix = xTaskGetTickCount();
            
            /* Release mutex */
            xSemaphoreGive(xUARTMutex);
        }
        
        /* Notify system of GPS fix */
        vTaskManagerNotifyEvent(EVENT_GPS_FIX);
    }
    
    return pdPASS;
}

/**
 * @brief Parse RMC sentence (Recommended Minimum Navigation Information)
 * 
 * @param pcNMEASentence NMEA sentence to parse
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvGPSParseRMC(char *pcNMEASentence)
{
    char *pcToken;
    char *pcSavePtr;
    int iField = 0;
    GPSPosition_t xNewPosition;
    
    /* Initialize new position with current values */
    memcpy(&xNewPosition, &xCurrentPosition, sizeof(GPSPosition_t));
    
    /* Tokenize sentence */
    pcToken = strtok_r(pcNMEASentence, ",", &pcSavePtr);
    
    while (pcToken != NULL)
    {
        switch (iField)
        {
            case 1:  /* Time (hhmmss.sss) */
                xNewPosition.timestamp = atoi(pcToken);
                break;
                
            case 2:  /* Status (A=active, V=void) */
                xNewPosition.valid = (pcToken[0] == 'A') ? 1 : 0;
                break;
                
            case 3:  /* Latitude (ddmm.mmmm) */
                if (strlen(pcToken) > 0)
                {
                    float fLatitude = atof(pcToken);
                    int iDegrees = (int)(fLatitude / 100);
                    float fMinutes = fLatitude - (iDegrees * 100);
                    xNewPosition.latitude = iDegrees + (fMinutes / 60.0f);
                }
                break;
                
            case 4:  /* N/S indicator */
                if (pcToken[0] == 'S')
                {
                    xNewPosition.latitude = -xNewPosition.latitude;
                }
                break;
                
            case 5:  /* Longitude (dddmm.mmmm) */
                if (strlen(pcToken) > 0)
                {
                    float fLongitude = atof(pcToken);
                    int iDegrees = (int)(fLongitude / 100);
                    float fMinutes = fLongitude - (iDegrees * 100);
                    xNewPosition.longitude = iDegrees + (fMinutes / 60.0f);
                }
                break;
                
            case 6:  /* E/W indicator */
                if (pcToken[0] == 'W')
                {
                    xNewPosition.longitude = -xNewPosition.longitude;
                }
                break;
                
            case 7:  /* Speed (knots) */
                xNewPosition.speed = atof(pcToken) * 1.852f;  /* Convert knots to km/h */
                break;
                
            case 8:  /* Course (degrees) */
                xNewPosition.course = atof(pcToken);
                break;
                
            case 9:  /* Date (ddmmyy) */
                xNewPosition.date = atoi(pcToken);
                break;
        }
        
        pcToken = strtok_r(NULL, ",*", &pcSavePtr);
        iField++;
    }
    
    /* Update position if valid */
    if (xNewPosition.valid)
    {
        /* Take mutex to ensure atomic access to position data */
        if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(100)) == pdPASS)
        {
            /* Add to position history */
            prvGPSAddToPositionHistory(&xNewPosition);
            
            /* Apply position filtering */
            prvGPSFilterPosition();
            
            /* Update last valid fix time */
            ulLastValidFix = xTaskGetTickCount();
            
            /* Release mutex */
            xSemaphoreGive(xUARTMutex);
        }
        
        /* Notify system of GPS fix */
        vTaskManagerNotifyEvent(EVENT_GPS_FIX);
    }
    
    return pdPASS;
}

/**
 * @brief Apply filtering to GPS position
 */
static void prvGPSFilterPosition(void)
{
    /* Simple moving average filter */
    if (ucPositionHistoryCount > 0)
    {
        float fLatSum = 0;
        float fLonSum = 0;
        float fAltSum = 0;
        
        for (int i = 0; i < ucPositionHistoryCount; i++)
        {
            fLatSum += xPositionHistory[i].latitude;
            fLonSum += xPositionHistory[i].longitude;
            fAltSum += xPositionHistory[i].altitude;
        }
        
        /* Update current position with filtered values */
        xCurrentPosition.latitude = fLatSum / ucPositionHistoryCount;
        xCurrentPosition.longitude = fLonSum / ucPositionHistoryCount;
        xCurrentPosition.altitude = fAltSum / ucPositionHistoryCount;
        
        /* Copy other values from most recent position */
        int iLatestIndex = (ucPositionHistoryIndex - 1 + GPS_POSITION_HISTORY_SIZE) % GPS_POSITION_HISTORY_SIZE;
        xCurrentPosition.speed = xPositionHistory[iLatestIndex].speed;
        xCurrentPosition.course = xPositionHistory[iLatestIndex].course;
        xCurrentPosition.fix_type = xPositionHistory[iLatestIndex].fix_type;
        xCurrentPosition.satellites = xPositionHistory[iLatestIndex].satellites;
        xCurrentPosition.hdop = xPositionHistory[iLatestIndex].hdop;
        xCurrentPosition.timestamp = xPositionHistory[iLatestIndex].timestamp;
        xCurrentPosition.date = xPositionHistory[iLatestIndex].date;
        xCurrentPosition.valid = xPositionHistory[iLatestIndex].valid;
    }
}

/**
 * @brief Add position to history
 * 
 * @param pxPosition Position to add
 */
static void prvGPSAddToPositionHistory(GPSPosition_t *pxPosition)
{
    /* Add to circular buffer */
    memcpy(&xPositionHistory[ucPositionHistoryIndex], pxPosition, sizeof(GPSPosition_t));
    
    /* Update index */
    ucPositionHistoryIndex = (ucPositionHistoryIndex + 1) % GPS_POSITION_HISTORY_SIZE;
    
    /* Update count */
    if (ucPositionHistoryCount < GPS_POSITION_HISTORY_SIZE)
    {
        ucPositionHistoryCount++;
    }
}

/**
 * @brief Convert degrees to radians
 * 
 * @param fDegrees Angle in degrees
 * @return float Angle in radians
 */
static float prvGPSDegreesToRadians(float fDegrees)
{
    return fDegrees * (M_PI / 180.0f);
}

/**
 * @brief Convert radians to degrees
 * 
 * @param fRadians Angle in radians
 * @return float Angle in degrees
 */
static float prvGPSRadiansToDegrees(float fRadians)
{
    return fRadians * (180.0f / M_PI);
}

/**
 * @brief Verify NMEA checksum
 * 
 * @param pcNMEASentence NMEA sentence to verify
 * @return uint8_t 1 if checksum is valid, 0 otherwise
 */
static uint8_t prvGPSChecksum(const char *pcNMEASentence)
{
    uint8_t ucCalculatedChecksum = 0;
    uint8_t ucProvidedChecksum = 0;
    const char *pcChecksumStart = NULL;
    
    /* Find checksum in sentence */
    pcChecksumStart = strchr(pcNMEASentence, '*');
    if (pcChecksumStart == NULL)
    {
        return 0;
    }
    
    /* Extract provided checksum */
    if (sscanf(pcChecksumStart + 1, "%2hhx", &ucProvidedChecksum) != 1)
    {
        return 0;
    }
    
    /* Calculate checksum (XOR of all bytes between $ and *) */
    for (const char *pc = pcNMEASentence + 1; pc < pcChecksumStart; pc++)
    {
        ucCalculatedChecksum ^= *pc;
    }
    
    /* Compare checksums */
    return (ucCalculatedChecksum == ucProvidedChecksum) ? 1 : 0;
}
