#ifndef AUDIO_H
#define AUDIO_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>

/* Audio sampling parameters */
#define AUDIO_SAMPLE_RATE           16000  /* 16 kHz */
#define AUDIO_BITS_PER_SAMPLE       16     /* 16-bit PCM */
#define AUDIO_CHANNELS              1      /* Mono */
#define AUDIO_FRAME_SIZE            512    /* Samples per frame */
#define AUDIO_BUFFER_SIZE           (AUDIO_FRAME_SIZE * (AUDIO_BITS_PER_SAMPLE / 8) * AUDIO_CHANNELS)

/* Audio frame structure */
typedef struct {
    uint8_t *data;                  /* Pointer to audio data */
    uint32_t size;                  /* Size of audio data in bytes */
    uint32_t timestamp;             /* Frame timestamp */
} AudioFrame_t;

/* Audio alert types */
typedef enum {
    AUDIO_ALERT_NONE,
    AUDIO_ALERT_MOTION_DETECTED,
    AUDIO_ALERT_PERSON_DETECTED,
    AUDIO_ALERT_LOW_BATTERY,
    AUDIO_ALERT_SYSTEM_ERROR,
    AUDIO_ALERT_CUSTOM
} AudioAlertType_t;

/* Audio detection types */
typedef enum {
    AUDIO_DETECTION_NONE,
    AUDIO_DETECTION_SOUND,
    AUDIO_DETECTION_VOICE,
    AUDIO_DETECTION_GLASS_BREAK,
    AUDIO_DETECTION_ALARM,
    AUDIO_DETECTION_DOG_BARK
} AudioDetectionType_t;

/* Audio detection result structure */
typedef struct {
    AudioDetectionType_t type;      /* Detection type */
    float confidence;               /* Detection confidence (0.0-1.0) */
    uint32_t timestamp;             /* Detection timestamp */
} AudioDetectionResult_t;

/* Audio function prototypes */
BaseType_t xAudioInit(void);
BaseType_t xAudioStart(void);
BaseType_t xAudioStop(void);
BaseType_t xAudioGetFrame(AudioFrame_t *pxFrame, TickType_t xTimeout);
BaseType_t xAudioReleaseFrame(AudioFrame_t *pxFrame);
BaseType_t xAudioPlayAlert(AudioAlertType_t eAlertType);
BaseType_t xAudioPlayCustomSound(const uint8_t *pucSoundData, uint32_t ulSoundSize);
BaseType_t xAudioSetVolume(uint8_t ucVolume);
uint8_t ucAudioGetVolume(void);
BaseType_t xAudioEnableDetection(uint8_t ucEnable);
BaseType_t xAudioGetDetectionResult(AudioDetectionResult_t *pxResult);

#endif /* AUDIO_H */
