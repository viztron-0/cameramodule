#include "task_manager.h"
#include "freertos_config.h"
#include <stdio.h>

/* Task handles */
TaskHandle_t xCameraTaskHandle = NULL;
TaskHandle_t xAITaskHandle = NULL;
TaskHandle_t xCommunicationTaskHandle = NULL;
TaskHandle_t xGPSTaskHandle = NULL;
TaskHandle_t xAudioTaskHandle = NULL;
TaskHandle_t xPowerTaskHandle = NULL;

/* Queue handles */
QueueHandle_t xFrameQueue = NULL;
QueueHandle_t xDetectionQueue = NULL;
QueueHandle_t xGPSDataQueue = NULL;
QueueHandle_t xAudioQueue = NULL;
QueueHandle_t xCommandQueue = NULL;

/* Semaphore handles */
SemaphoreHandle_t xI2CMutex = NULL;
SemaphoreHandle_t xSPIMutex = NULL;
SemaphoreHandle_t xUARTMutex = NULL;
SemaphoreHandle_t xWiFiMutex = NULL;

/* Event group handles */
EventGroupHandle_t xSystemEvents = NULL;

/* System state */
SystemState_t eSystemState = SYSTEM_STATE_INIT;

/**
 * @brief Initialize the task manager and create all system tasks
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xTaskManagerInit(void)
{
    BaseType_t xReturn = pdPASS;
    
    /* Create system event group */
    xSystemEvents = xEventGroupCreate();
    if (xSystemEvents == NULL)
    {
        printf("Failed to create system events group\n");
        return pdFAIL;
    }
    
    /* Create mutexes */
    xI2CMutex = xSemaphoreCreateMutex();
    xSPIMutex = xSemaphoreCreateMutex();
    xUARTMutex = xSemaphoreCreateMutex();
    xWiFiMutex = xSemaphoreCreateMutex();
    
    if (xI2CMutex == NULL || xSPIMutex == NULL || xUARTMutex == NULL || xWiFiMutex == NULL)
    {
        printf("Failed to create one or more mutexes\n");
        return pdFAIL;
    }
    
    /* Create queues */
    xFrameQueue = xQueueCreate(2, sizeof(void *));  /* Double buffering for frames */
    xDetectionQueue = xQueueCreate(10, sizeof(void *));
    xGPSDataQueue = xQueueCreate(5, sizeof(void *));
    xAudioQueue = xQueueCreate(5, sizeof(void *));
    xCommandQueue = xQueueCreate(10, sizeof(void *));
    
    if (xFrameQueue == NULL || xDetectionQueue == NULL || xGPSDataQueue == NULL || 
        xAudioQueue == NULL || xCommandQueue == NULL)
    {
        printf("Failed to create one or more queues\n");
        return pdFAIL;
    }
    
    /* Create tasks */
    xReturn = xTaskCreate(vPowerTask, "Power", TASK_STACK_SIZE_POWER, 
                         NULL, TASK_PRIORITY_POWER, &xPowerTaskHandle);
    if (xReturn != pdPASS)
    {
        printf("Failed to create Power task\n");
        return pdFAIL;
    }
    
    xReturn = xTaskCreate(vGPSTask, "GPS", TASK_STACK_SIZE_GPS, 
                         NULL, TASK_PRIORITY_GPS, &xGPSTaskHandle);
    if (xReturn != pdPASS)
    {
        printf("Failed to create GPS task\n");
        return pdFAIL;
    }
    
    xReturn = xTaskCreate(vAudioTask, "Audio", TASK_STACK_SIZE_AUDIO, 
                         NULL, TASK_PRIORITY_AUDIO, &xAudioTaskHandle);
    if (xReturn != pdPASS)
    {
        printf("Failed to create Audio task\n");
        return pdFAIL;
    }
    
    xReturn = xTaskCreate(vCommunicationTask, "Comm", TASK_STACK_SIZE_COMMUNICATION, 
                         NULL, TASK_PRIORITY_COMMUNICATION, &xCommunicationTaskHandle);
    if (xReturn != pdPASS)
    {
        printf("Failed to create Communication task\n");
        return pdFAIL;
    }
    
    xReturn = xTaskCreate(vAITask, "AI", TASK_STACK_SIZE_AI, 
                         NULL, TASK_PRIORITY_AI, &xAITaskHandle);
    if (xReturn != pdPASS)
    {
        printf("Failed to create AI task\n");
        return pdFAIL;
    }
    
    xReturn = xTaskCreate(vCameraTask, "Camera", TASK_STACK_SIZE_CAMERA, 
                         NULL, TASK_PRIORITY_CAMERA, &xCameraTaskHandle);
    if (xReturn != pdPASS)
    {
        printf("Failed to create Camera task\n");
        return pdFAIL;
    }
    
    /* Set initial system state */
    eSystemState = SYSTEM_STATE_INIT;
    
    return pdPASS;
}

/**
 * @brief Suspend all system tasks
 */
void vTaskManagerSuspendAllTasks(void)
{
    if (xCameraTaskHandle != NULL)
        vTaskSuspend(xCameraTaskHandle);
    
    if (xAITaskHandle != NULL)
        vTaskSuspend(xAITaskHandle);
    
    if (xCommunicationTaskHandle != NULL)
        vTaskSuspend(xCommunicationTaskHandle);
    
    if (xGPSTaskHandle != NULL)
        vTaskSuspend(xGPSTaskHandle);
    
    if (xAudioTaskHandle != NULL)
        vTaskSuspend(xAudioTaskHandle);
    
    /* Don't suspend power task as it manages system state */
}

/**
 * @brief Resume all system tasks
 */
void vTaskManagerResumeAllTasks(void)
{
    if (xCameraTaskHandle != NULL)
        vTaskResume(xCameraTaskHandle);
    
    if (xAITaskHandle != NULL)
        vTaskResume(xAITaskHandle);
    
    if (xCommunicationTaskHandle != NULL)
        vTaskResume(xCommunicationTaskHandle);
    
    if (xGPSTaskHandle != NULL)
        vTaskResume(xGPSTaskHandle);
    
    if (xAudioTaskHandle != NULL)
        vTaskResume(xAudioTaskHandle);
}

/**
 * @brief Set the system state and notify all tasks
 * 
 * @param eNewState The new system state
 */
void vTaskManagerSetSystemState(SystemState_t eNewState)
{
    eSystemState = eNewState;
    
    /* Notify all tasks of state change */
    if (xCameraTaskHandle != NULL)
        xTaskNotify(xCameraTaskHandle, (uint32_t)eNewState, eSetValueWithOverwrite);
    
    if (xAITaskHandle != NULL)
        xTaskNotify(xAITaskHandle, (uint32_t)eNewState, eSetValueWithOverwrite);
    
    if (xCommunicationTaskHandle != NULL)
        xTaskNotify(xCommunicationTaskHandle, (uint32_t)eNewState, eSetValueWithOverwrite);
    
    if (xGPSTaskHandle != NULL)
        xTaskNotify(xGPSTaskHandle, (uint32_t)eNewState, eSetValueWithOverwrite);
    
    if (xAudioTaskHandle != NULL)
        xTaskNotify(xAudioTaskHandle, (uint32_t)eNewState, eSetValueWithOverwrite);
    
    if (xPowerTaskHandle != NULL)
        xTaskNotify(xPowerTaskHandle, (uint32_t)eNewState, eSetValueWithOverwrite);
}

/**
 * @brief Get the current system state
 * 
 * @return SystemState_t Current system state
 */
SystemState_t xTaskManagerGetSystemState(void)
{
    return eSystemState;
}

/**
 * @brief Notify system event to all tasks
 * 
 * @param xEventBits Event bits to set
 */
void vTaskManagerNotifyEvent(EventBits_t xEventBits)
{
    if (xSystemEvents != NULL)
    {
        xEventGroupSetBits(xSystemEvents, xEventBits);
    }
}

/* Task function implementations - these will be expanded in separate files */

void vCameraTask(void *pvParameters)
{
    /* Task initialization */
    printf("Camera task started\n");
    
    for (;;)
    {
        /* Main task loop - will be implemented in camera.c */
        vTaskDelay(pdMS_TO_TICKS(10)); /* 100 Hz for camera task */
    }
}

void vAITask(void *pvParameters)
{
    /* Task initialization */
    printf("AI task started\n");
    
    for (;;)
    {
        /* Main task loop - will be implemented in ai.c */
        vTaskDelay(pdMS_TO_TICKS(50)); /* 20 Hz for AI task */
    }
}

void vCommunicationTask(void *pvParameters)
{
    /* Task initialization */
    printf("Communication task started\n");
    
    for (;;)
    {
        /* Main task loop - will be implemented in communication.c */
        vTaskDelay(pdMS_TO_TICKS(100)); /* 10 Hz for communication task */
    }
}

void vGPSTask(void *pvParameters)
{
    /* Task initialization */
    printf("GPS task started\n");
    
    for (;;)
    {
        /* Main task loop - will be implemented in gps.c */
        vTaskDelay(pdMS_TO_TICKS(1000)); /* 1 Hz for GPS task */
    }
}

void vAudioTask(void *pvParameters)
{
    /* Task initialization */
    printf("Audio task started\n");
    
    for (;;)
    {
        /* Main task loop - will be implemented in audio.c */
        vTaskDelay(pdMS_TO_TICKS(20)); /* 50 Hz for audio task */
    }
}

void vPowerTask(void *pvParameters)
{
    /* Task initialization */
    printf("Power task started\n");
    
    for (;;)
    {
        /* Main task loop - will be implemented in power.c */
        vTaskDelay(pdMS_TO_TICKS(1000)); /* 1 Hz for power management task */
    }
}
