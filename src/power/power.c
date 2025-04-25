#include "power.h"
#include "task_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* Include Raspberry Pi Pico specific headers */
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "pico/sleep.h"

/* PIR motion sensor pin */
#define PIR_SENSOR_PIN              22

/* Battery ADC pin */
#define BATTERY_ADC_PIN             26
#define BATTERY_ADC_CHANNEL         0

/* Battery parameters */
#define BATTERY_CAPACITY_MAH        2000
#define BATTERY_FULL_VOLTAGE        4.2f
#define BATTERY_EMPTY_VOLTAGE       3.3f
#define BATTERY_LOW_THRESHOLD       15

/* Power consumption estimates (mA) */
#define POWER_CONSUMPTION_ACTIVE    300
#define POWER_CONSUMPTION_ALERT     350
#define POWER_CONSUMPTION_IDLE      150
#define POWER_CONSUMPTION_LOW_POWER 30
#define POWER_CONSUMPTION_SLEEP     5

/* CPU frequencies */
#define CPU_FREQ_HIGH               125000
#define CPU_FREQ_MEDIUM             80000
#define CPU_FREQ_LOW                48000

/* Static variables */
static PowerState_t eCurrentPowerState = POWER_STATE_ACTIVE;
static BatteryStatus_t xBatteryStatus;
static uint8_t ucPIRWakeupEnabled = 0;
static uint32_t ulCurrentCPUFrequency = CPU_FREQ_HIGH;
static uint32_t ulLastBatteryCheck = 0;
static uint32_t ulBatteryCheckInterval = 10000;  /* 10 seconds */

/* Forward declarations of static functions */
static void prvPowerInitGPIO(void);
static void prvPowerInitADC(void);
static void prvPowerUpdateBatteryStatus(void);
static void prvPowerPIRCallback(uint gpio, uint32_t events);
static uint8_t prvPowerCalculateBatteryPercentage(float fVoltage);
static uint32_t prvPowerEstimateRemainingTime(void);
static void prvPowerApplyStateSettings(PowerState_t eState);

/**
 * @brief Initialize the power management system
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xPowerInit(void)
{
    /* Initialize GPIO for PIR sensor */
    prvPowerInitGPIO();
    
    /* Initialize ADC for battery monitoring */
    prvPowerInitADC();
    
    /* Initialize battery status */
    memset(&xBatteryStatus, 0, sizeof(BatteryStatus_t));
    
    /* Update battery status */
    prvPowerUpdateBatteryStatus();
    
    /* Set initial power state */
    eCurrentPowerState = POWER_STATE_ACTIVE;
    prvPowerApplyStateSettings(eCurrentPowerState);
    
    printf("Power management system initialized\n");
    return pdPASS;
}

/**
 * @brief Set power state
 * 
 * @param eState New power state
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xPowerSetState(PowerState_t eState)
{
    /* Check if state is valid */
    if (eState > POWER_STATE_SLEEP)
    {
        return pdFAIL;
    }
    
    /* Apply state settings */
    prvPowerApplyStateSettings(eState);
    
    /* Update current state */
    eCurrentPowerState = eState;
    
    /* Notify system of state change */
    vTaskManagerSetSystemState((SystemState_t)eState);
    
    return pdPASS;
}

/**
 * @brief Get current power state
 * 
 * @return PowerState_t Current power state
 */
PowerState_t xPowerGetState(void)
{
    return eCurrentPowerState;
}

/**
 * @brief Get battery status
 * 
 * @param pxStatus Pointer to battery status structure
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xPowerGetBatteryStatus(BatteryStatus_t *pxStatus)
{
    if (pxStatus == NULL)
    {
        return pdFAIL;
    }
    
    /* Check if it's time to update battery status */
    uint32_t ulCurrentTime = xTaskGetTickCount();
    if (ulCurrentTime - ulLastBatteryCheck >= ulBatteryCheckInterval)
    {
        prvPowerUpdateBatteryStatus();
        ulLastBatteryCheck = ulCurrentTime;
    }
    
    /* Copy battery status */
    memcpy(pxStatus, &xBatteryStatus, sizeof(BatteryStatus_t));
    
    return pdPASS;
}

/**
 * @brief Enable or disable PIR wakeup
 * 
 * @param ucEnable 1 to enable, 0 to disable
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xPowerEnablePIRWakeup(uint8_t ucEnable)
{
    ucPIRWakeupEnabled = ucEnable ? 1 : 0;
    
    /* Configure PIR interrupt */
    gpio_set_irq_enabled(PIR_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, ucPIRWakeupEnabled);
    
    return pdPASS;
}

/**
 * @brief Set CPU frequency
 * 
 * @param ulFrequencyMHz CPU frequency in MHz
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xPowerSetCPUFrequency(uint32_t ulFrequencyMHz)
{
    /* Validate frequency */
    if (ulFrequencyMHz < 48 || ulFrequencyMHz > 133)
    {
        return pdFAIL;
    }
    
    /* Convert MHz to kHz */
    uint32_t ulFrequencyKHz = ulFrequencyMHz * 1000;
    
    /* Set system clock frequency */
    set_sys_clock_khz(ulFrequencyKHz, true);
    
    /* Update current frequency */
    ulCurrentCPUFrequency = ulFrequencyKHz;
    
    return pdPASS;
}

/**
 * @brief Get current CPU frequency
 * 
 * @return uint32_t Current CPU frequency in kHz
 */
uint32_t ulPowerGetCPUFrequency(void)
{
    return ulCurrentCPUFrequency;
}

/**
 * @brief Enter sleep mode for specified time
 * 
 * @param ulSleepTimeMs Sleep time in milliseconds
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xPowerEnterSleep(uint32_t ulSleepTimeMs)
{
    /* Suspend all tasks except power task */
    vTaskManagerSuspendAllTasks();
    
    /* Enable PIR wakeup if needed */
    if (ucPIRWakeupEnabled)
    {
        xPowerEnablePIRWakeup(1);
    }
    
    /* Save current state */
    PowerState_t ePreviousState = eCurrentPowerState;
    
    /* Set sleep state */
    eCurrentPowerState = POWER_STATE_SLEEP;
    
    /* Configure sleep mode */
    sleep_run_from_xosc();
    
    /* Sleep for specified time */
    sleep_ms(ulSleepTimeMs);
    
    /* Restore previous state */
    eCurrentPowerState = ePreviousState;
    prvPowerApplyStateSettings(eCurrentPowerState);
    
    /* Resume all tasks */
    vTaskManagerResumeAllTasks();
    
    return pdPASS;
}

/**
 * @brief Handle motion detection event
 */
void vPowerHandleMotionDetection(void)
{
    /* If in sleep or low power mode, transition to active mode */
    if (eCurrentPowerState == POWER_STATE_SLEEP || eCurrentPowerState == POWER_STATE_LOW_POWER)
    {
        xPowerSetState(POWER_STATE_ACTIVE);
    }
    
    /* Notify system of motion detection */
    vTaskManagerNotifyEvent(EVENT_MOTION_DETECTED);
}

/**
 * @brief Initialize GPIO for PIR sensor
 */
static void prvPowerInitGPIO(void)
{
    /* Initialize PIR sensor pin */
    gpio_init(PIR_SENSOR_PIN);
    gpio_set_dir(PIR_SENSOR_PIN, GPIO_IN);
    gpio_pull_down(PIR_SENSOR_PIN);
    
    /* Set up PIR interrupt */
    gpio_set_irq_callback(prvPowerPIRCallback);
    
    /* Initially disable PIR interrupt */
    gpio_set_irq_enabled(PIR_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, false);
}

/**
 * @brief Initialize ADC for battery monitoring
 */
static void prvPowerInitADC(void)
{
    /* Initialize ADC */
    adc_init();
    
    /* Initialize battery ADC pin */
    adc_gpio_init(BATTERY_ADC_PIN);
    
    /* Select ADC channel */
    adc_select_input(BATTERY_ADC_CHANNEL);
}

/**
 * @brief Update battery status
 */
static void prvPowerUpdateBatteryStatus(void)
{
    /* Read ADC value */
    adc_select_input(BATTERY_ADC_CHANNEL);
    uint16_t raw = adc_read();
    
    /* Convert to voltage (assuming 3.3V reference and 12-bit ADC) */
    float voltage = (float)raw * 3.3f / 4096.0f;
    
    /* Apply voltage divider correction if needed */
    /* This assumes a voltage divider is used to measure battery voltage */
    /* For example, if using 100k and 100k resistors: */
    /* voltage = voltage * 2.0f; */
    
    /* Update battery status */
    xBatteryStatus.voltage = voltage;
    xBatteryStatus.percentage = prvPowerCalculateBatteryPercentage(voltage);
    
    /* Determine if battery is low */
    xBatteryStatus.low_battery = (xBatteryStatus.percentage <= BATTERY_LOW_THRESHOLD) ? 1 : 0;
    
    /* Estimate remaining time */
    xBatteryStatus.remaining_time = prvPowerEstimateRemainingTime();
    
    /* Check if charging (this would require additional hardware) */
    /* For now, assume not charging */
    xBatteryStatus.is_charging = 0;
    xBatteryStatus.current = -100;  /* Assume 100mA discharge current */
    
    /* Notify system if battery is low */
    if (xBatteryStatus.low_battery)
    {
        vTaskManagerNotifyEvent(EVENT_LOW_BATTERY);
    }
}

/**
 * @brief PIR sensor interrupt callback
 * 
 * @param gpio GPIO pin number
 * @param events GPIO events
 */
static void prvPowerPIRCallback(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (gpio == PIR_SENSOR_PIN && (events & GPIO_IRQ_EDGE_RISE))
    {
        /* Notify power task of motion detection */
        if (xPowerTaskHandle != NULL)
        {
            vTaskNotifyGiveFromISR(xPowerTaskHandle, &xHigherPriorityTaskWoken);
        }
    }
    
    /* Yield if a higher priority task was woken */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Calculate battery percentage from voltage
 * 
 * @param fVoltage Battery voltage
 * @return uint8_t Battery percentage (0-100)
 */
static uint8_t prvPowerCalculateBatteryPercentage(float fVoltage)
{
    /* Simple linear mapping from voltage to percentage */
    if (fVoltage >= BATTERY_FULL_VOLTAGE)
    {
        return 100;
    }
    else if (fVoltage <= BATTERY_EMPTY_VOLTAGE)
    {
        return 0;
    }
    else
    {
        return (uint8_t)((fVoltage - BATTERY_EMPTY_VOLTAGE) * 100.0f / 
                         (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE));
    }
}

/**
 * @brief Estimate remaining battery time
 * 
 * @return uint32_t Estimated remaining time in seconds
 */
static uint32_t prvPowerEstimateRemainingTime(void)
{
    /* Estimate current consumption based on power state */
    uint16_t current_consumption;
    
    switch (eCurrentPowerState)
    {
        case POWER_STATE_ACTIVE:
            current_consumption = POWER_CONSUMPTION_ACTIVE;
            break;
            
        case POWER_STATE_ALERT:
            current_consumption = POWER_CONSUMPTION_ALERT;
            break;
            
        case POWER_STATE_IDLE:
            current_consumption = POWER_CONSUMPTION_IDLE;
            break;
            
        case POWER_STATE_LOW_POWER:
            current_consumption = POWER_CONSUMPTION_LOW_POWER;
            break;
            
        case POWER_STATE_SLEEP:
            current_consumption = POWER_CONSUMPTION_SLEEP;
            break;
            
        default:
            current_consumption = POWER_CONSUMPTION_ACTIVE;
            break;
    }
    
    /* Calculate remaining capacity (mAh) */
    float remaining_capacity = (float)BATTERY_CAPACITY_MAH * (float)xBatteryStatus.percentage / 100.0f;
    
    /* Calculate remaining time (hours) */
    float remaining_hours = remaining_capacity / (float)current_consumption;
    
    /* Convert to seconds */
    return (uint32_t)(remaining_hours * 3600.0f);
}

/**
 * @brief Apply settings for the specified power state
 * 
 * @param eState Power state
 */
static void prvPowerApplyStateSettings(PowerState_t eState)
{
    switch (eState)
    {
        case POWER_STATE_ACTIVE:
            /* Set high CPU frequency */
            xPowerSetCPUFrequency(CPU_FREQ_HIGH / 1000);
            
            /* Disable PIR wakeup */
            xPowerEnablePIRWakeup(0);
            break;
            
        case POWER_STATE_ALERT:
            /* Set high CPU frequency */
            xPowerSetCPUFrequency(CPU_FREQ_HIGH / 1000);
            
            /* Disable PIR wakeup */
            xPowerEnablePIRWakeup(0);
            break;
            
        case POWER_STATE_IDLE:
            /* Set medium CPU frequency */
            xPowerSetCPUFrequency(CPU_FREQ_MEDIUM / 1000);
            
            /* Enable PIR wakeup */
            xPowerEnablePIRWakeup(1);
            break;
            
        case POWER_STATE_LOW_POWER:
            /* Set low CPU frequency */
            xPowerSetCPUFrequency(CPU_FREQ_LOW / 1000);
            
            /* Enable PIR wakeup */
            xPowerEnablePIRWakeup(1);
            break;
            
        case POWER_STATE_SLEEP:
            /* Set lowest CPU frequency */
            xPowerSetCPUFrequency(CPU_FREQ_LOW / 1000);
            
            /* Enable PIR wakeup */
            xPowerEnablePIRWakeup(1);
            break;
            
        default:
            /* Invalid state, use active as default */
            xPowerSetCPUFrequency(CPU_FREQ_HIGH / 1000);
            xPowerEnablePIRWakeup(0);
            break;
    }
}
