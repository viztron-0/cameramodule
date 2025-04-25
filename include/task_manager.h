#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"

/* Task priorities */
#define TASK_PRIORITY_CAMERA        (configMAX_PRIORITIES - 1)  /* Highest priority */
#define TASK_PRIORITY_AI            (configMAX_PRIORITIES - 2)
#define TASK_PRIORITY_COMMUNICATION (configMAX_PRIORITIES - 3)
#define TASK_PRIORITY_GPS           (configMAX_PRIORITIES - 4)
#define TASK_PRIORITY_AUDIO         (configMAX_PRIORITIES - 4)
#define TASK_PRIORITY_POWER         (configMAX_PRIORITIES - 5)  /* Lowest priority */

/* Task stack sizes */
#define TASK_STACK_SIZE_CAMERA        (configMINIMAL_STACK_SIZE * 4)
#define TASK_STACK_SIZE_AI            (configMINIMAL_STACK_SIZE * 8)  /* Larger stack for AI processing */
#define TASK_STACK_SIZE_COMMUNICATION (configMINIMAL_STACK_SIZE * 4)
#define TASK_STACK_SIZE_GPS           (configMINIMAL_STACK_SIZE * 2)
#define TASK_STACK_SIZE_AUDIO         (configMINIMAL_STACK_SIZE * 2)
#define TASK_STACK_SIZE_POWER         (configMINIMAL_STACK_SIZE * 2)

/* Task handles */
extern TaskHandle_t xCameraTaskHandle;
extern TaskHandle_t xAITaskHandle;
extern TaskHandle_t xCommunicationTaskHandle;
extern TaskHandle_t xGPSTaskHandle;
extern TaskHandle_t xAudioTaskHandle;
extern TaskHandle_t xPowerTaskHandle;

/* Queue handles */
extern QueueHandle_t xFrameQueue;           /* Queue for camera frames */
extern QueueHandle_t xDetectionQueue;       /* Queue for detection results */
extern QueueHandle_t xGPSDataQueue;         /* Queue for GPS data */
extern QueueHandle_t xAudioQueue;           /* Queue for audio data */
extern QueueHandle_t xCommandQueue;         /* Queue for commands from Homebase */

/* Semaphore handles */
extern SemaphoreHandle_t xI2CMutex;         /* Mutex for I2C bus access */
extern SemaphoreHandle_t xSPIMutex;         /* Mutex for SPI bus access */
extern SemaphoreHandle_t xUARTMutex;        /* Mutex for UART access */
extern SemaphoreHandle_t xWiFiMutex;        /* Mutex for WiFi access */

/* Event group handles */
extern EventGroupHandle_t xSystemEvents;    /* Event group for system-wide events */

/* System event bits */
#define EVENT_MOTION_DETECTED       (1 << 0)
#define EVENT_OBJECT_DETECTED       (1 << 1)
#define EVENT_LOW_BATTERY           (1 << 2)
#define EVENT_WIFI_CONNECTED        (1 << 3)
#define EVENT_HOMEBASE_CONNECTED    (1 << 4)
#define EVENT_GPS_FIX               (1 << 5)
#define EVENT_SYSTEM_ERROR          (1 << 6)
#define EVENT_FIRMWARE_UPDATE       (1 << 7)

/* System states */
typedef enum {
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_ALERT,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_LOW_POWER,
    SYSTEM_STATE_SLEEP,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_UPDATE
} SystemState_t;

extern SystemState_t eSystemState;

/* Task function prototypes */
void vCameraTask(void *pvParameters);
void vAITask(void *pvParameters);
void vCommunicationTask(void *pvParameters);
void vGPSTask(void *pvParameters);
void vAudioTask(void *pvParameters);
void vPowerTask(void *pvParameters);

/* Task manager functions */
BaseType_t xTaskManagerInit(void);
void vTaskManagerSuspendAllTasks(void);
void vTaskManagerResumeAllTasks(void);
void vTaskManagerSetSystemState(SystemState_t eNewState);
SystemState_t xTaskManagerGetSystemState(void);
void vTaskManagerNotifyEvent(EventBits_t xEventBits);

#endif /* TASK_MANAGER_H */
