#ifndef POWER_H
#define POWER_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* Power states */
typedef enum {
    POWER_STATE_ACTIVE,       /* Full power, all systems operational */
    POWER_STATE_ALERT,        /* Increased processing during detection events */
    POWER_STATE_IDLE,         /* Reduced frame rate and processing */
    POWER_STATE_LOW_POWER,    /* Minimal processing, camera in standby */
    POWER_STATE_SLEEP         /* Deep sleep, wake on motion only */
} PowerState_t;

/* Battery status */
typedef struct {
    uint8_t percentage;       /* Battery level (0-100%) */
    float voltage;            /* Battery voltage */
    int16_t current;          /* Battery current (positive = charging, negative = discharging) */
    uint8_t is_charging;      /* 1 if battery is charging, 0 otherwise */
    uint32_t remaining_time;  /* Estimated remaining time in seconds */
    uint8_t low_battery;      /* 1 if battery is low, 0 otherwise */
} BatteryStatus_t;

/* Power management function prototypes */
BaseType_t xPowerInit(void);
BaseType_t xPowerSetState(PowerState_t eState);
PowerState_t xPowerGetState(void);
BaseType_t xPowerGetBatteryStatus(BatteryStatus_t *pxStatus);
BaseType_t xPowerEnablePIRWakeup(uint8_t ucEnable);
BaseType_t xPowerSetCPUFrequency(uint32_t ulFrequencyMHz);
uint32_t ulPowerGetCPUFrequency(void);
BaseType_t xPowerEnterSleep(uint32_t ulSleepTimeMs);
void vPowerHandleMotionDetection(void);

#endif /* POWER_H */
