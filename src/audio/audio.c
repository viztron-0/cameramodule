#include "audio.h"
#include "task_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Include Raspberry Pi Pico specific headers */
#include "pico/stdlib.h"
#include "hardware/i2s.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

/* I2S pins for audio interface */
#define AUDIO_I2S_DATA_PIN          26
#define AUDIO_I2S_CLOCK_PIN         27
#define AUDIO_I2S_WS_PIN            28

/* Speaker amplifier enable pin */
#define AUDIO_AMP_ENABLE_PIN        15

/* Microphone parameters */
#define MIC_GAIN                    20      /* dB */

/* Audio alert sounds */
#define MAX_ALERT_SOUNDS            5
#define MAX_ALERT_SIZE              16000   /* 1 second at 16kHz */

/* DMA channel for audio */
static int audio_dma_channel;

/* Audio buffers */
#define AUDIO_BUFFER_COUNT          3       /* Triple buffering */
static uint8_t ucAudioBuffers[AUDIO_BUFFER_COUNT][AUDIO_BUFFER_SIZE] __attribute__((aligned(32)));
static AudioFrame_t xAudioFrames[AUDIO_BUFFER_COUNT];
static uint8_t ucCurrentBuffer = 0;
static uint8_t ucProcessingBuffer = 0;

/* Audio alert sounds */
static uint8_t ucAlertSounds[MAX_ALERT_SOUNDS][MAX_ALERT_SIZE];
static uint32_t ulAlertSizes[MAX_ALERT_SOUNDS];

/* Audio state */
static uint8_t ucAudioInitialized = 0;
static uint8_t ucAudioRunning = 0;
static uint8_t ucAudioVolume = 80;          /* 0-100 */
static uint8_t ucDetectionEnabled = 0;

/* Audio queues */
static QueueHandle_t xAudioFrameQueue;
static QueueHandle_t xAudioDetectionQueue;

/* Forward declarations of static functions */
static void prvAudioI2SInit(void);
static void prvAudioDMAInit(void);
static void prvAudioInitAlertSounds(void);
static void prvAudioDMACallback(void);
static void prvAudioProcessFrame(AudioFrame_t *pxFrame);
static float prvAudioCalculateRMS(const int16_t *psSamples, uint32_t ulNumSamples);
static void prvAudioDetectSounds(const int16_t *psSamples, uint32_t ulNumSamples);
static void prvGenerateSineWave(uint8_t *pucBuffer, uint32_t ulSize, float fFrequency, float fAmplitude);

/**
 * @brief Initialize the audio module
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioInit(void)
{
    /* Initialize I2S for audio interface */
    prvAudioI2SInit();
    
    /* Initialize DMA for audio data */
    prvAudioDMAInit();
    
    /* Initialize audio buffers */
    for (int i = 0; i < AUDIO_BUFFER_COUNT; i++)
    {
        xAudioFrames[i].data = ucAudioBuffers[i];
        xAudioFrames[i].size = AUDIO_BUFFER_SIZE;
        xAudioFrames[i].timestamp = 0;
    }
    
    /* Initialize alert sounds */
    prvAudioInitAlertSounds();
    
    /* Create audio frame queue */
    xAudioFrameQueue = xQueueCreate(AUDIO_BUFFER_COUNT, sizeof(AudioFrame_t *));
    if (xAudioFrameQueue == NULL)
    {
        printf("Failed to create audio frame queue\n");
        return pdFAIL;
    }
    
    /* Create audio detection queue */
    xAudioDetectionQueue = xQueueCreate(10, sizeof(AudioDetectionResult_t));
    if (xAudioDetectionQueue == NULL)
    {
        printf("Failed to create audio detection queue\n");
        return pdFAIL;
    }
    
    /* Initialize speaker amplifier pin */
    gpio_init(AUDIO_AMP_ENABLE_PIN);
    gpio_set_dir(AUDIO_AMP_ENABLE_PIN, GPIO_OUT);
    gpio_put(AUDIO_AMP_ENABLE_PIN, 0);  /* Initially disabled */
    
    ucAudioInitialized = 1;
    
    printf("Audio module initialized\n");
    return pdPASS;
}

/**
 * @brief Start audio capture
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioStart(void)
{
    if (!ucAudioInitialized)
    {
        return pdFAIL;
    }
    
    if (ucAudioRunning)
    {
        return pdPASS;  /* Already running */
    }
    
    /* Start DMA transfer */
    dma_channel_start(audio_dma_channel);
    
    ucAudioRunning = 1;
    
    return pdPASS;
}

/**
 * @brief Stop audio capture
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioStop(void)
{
    if (!ucAudioInitialized || !ucAudioRunning)
    {
        return pdFAIL;
    }
    
    /* Stop DMA transfer */
    dma_channel_abort(audio_dma_channel);
    
    ucAudioRunning = 0;
    
    return pdPASS;
}

/**
 * @brief Get audio frame
 * 
 * @param pxFrame Pointer to frame structure to fill
 * @param xTimeout Timeout in ticks to wait for a frame
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioGetFrame(AudioFrame_t *pxFrame, TickType_t xTimeout)
{
    AudioFrame_t *pxQueuedFrame;
    
    if (pxFrame == NULL || !ucAudioInitialized)
    {
        return pdFAIL;
    }
    
    /* Get frame from queue */
    if (xQueueReceive(xAudioFrameQueue, &pxQueuedFrame, xTimeout) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Copy frame data */
    pxFrame->data = pxQueuedFrame->data;
    pxFrame->size = pxQueuedFrame->size;
    pxFrame->timestamp = pxQueuedFrame->timestamp;
    
    return pdPASS;
}

/**
 * @brief Release audio frame
 * 
 * @param pxFrame Pointer to frame structure to release
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioReleaseFrame(AudioFrame_t *pxFrame)
{
    if (pxFrame == NULL || !ucAudioInitialized)
    {
        return pdFAIL;
    }
    
    /* Mark buffer as available for capture */
    for (int i = 0; i < AUDIO_BUFFER_COUNT; i++)
    {
        if (pxFrame->data == xAudioFrames[i].data)
        {
            /* Buffer is now available for capture */
            return pdPASS;
        }
    }
    
    return pdFAIL;
}

/**
 * @brief Play audio alert
 * 
 * @param eAlertType Alert type to play
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioPlayAlert(AudioAlertType_t eAlertType)
{
    if (!ucAudioInitialized || eAlertType >= MAX_ALERT_SOUNDS)
    {
        return pdFAIL;
    }
    
    /* Enable speaker amplifier */
    gpio_put(AUDIO_AMP_ENABLE_PIN, 1);
    
    /* Play alert sound */
    i2s_write(ucAlertSounds[eAlertType], ulAlertSizes[eAlertType]);
    
    /* Disable speaker amplifier */
    gpio_put(AUDIO_AMP_ENABLE_PIN, 0);
    
    return pdPASS;
}

/**
 * @brief Play custom sound
 * 
 * @param pucSoundData Pointer to sound data
 * @param ulSoundSize Size of sound data in bytes
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioPlayCustomSound(const uint8_t *pucSoundData, uint32_t ulSoundSize)
{
    if (!ucAudioInitialized || pucSoundData == NULL || ulSoundSize == 0)
    {
        return pdFAIL;
    }
    
    /* Enable speaker amplifier */
    gpio_put(AUDIO_AMP_ENABLE_PIN, 1);
    
    /* Play custom sound */
    i2s_write(pucSoundData, ulSoundSize);
    
    /* Disable speaker amplifier */
    gpio_put(AUDIO_AMP_ENABLE_PIN, 0);
    
    return pdPASS;
}

/**
 * @brief Set audio volume
 * 
 * @param ucVolume Volume level (0-100)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioSetVolume(uint8_t ucVolume)
{
    if (ucVolume > 100)
    {
        ucVolume = 100;
    }
    
    ucAudioVolume = ucVolume;
    
    /* Apply volume setting to hardware */
    /* This would depend on the specific amplifier or DAC used */
    
    return pdPASS;
}

/**
 * @brief Get audio volume
 * 
 * @return uint8_t Volume level (0-100)
 */
uint8_t ucAudioGetVolume(void)
{
    return ucAudioVolume;
}

/**
 * @brief Enable or disable audio detection
 * 
 * @param ucEnable 1 to enable, 0 to disable
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioEnableDetection(uint8_t ucEnable)
{
    ucDetectionEnabled = ucEnable ? 1 : 0;
    return pdPASS;
}

/**
 * @brief Get audio detection result
 * 
 * @param pxResult Pointer to detection result structure
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAudioGetDetectionResult(AudioDetectionResult_t *pxResult)
{
    if (pxResult == NULL || !ucAudioInitialized)
    {
        return pdFAIL;
    }
    
    /* Get detection result from queue */
    if (xQueueReceive(xAudioDetectionQueue, pxResult, 0) != pdPASS)
    {
        return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Initialize I2S for audio interface
 */
static void prvAudioI2SInit(void)
{
    /* Initialize I2S pins */
    gpio_set_function(AUDIO_I2S_DATA_PIN, GPIO_FUNC_I2S);
    gpio_set_function(AUDIO_I2S_CLOCK_PIN, GPIO_FUNC_I2S);
    gpio_set_function(AUDIO_I2S_WS_PIN, GPIO_FUNC_I2S);
    
    /* Initialize I2S */
    i2s_config_t config = {
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = AUDIO_BITS_PER_SAMPLE,
        .channel_format = AUDIO_CHANNELS == 1 ? I2S_CHANNEL_FMT_MONO : I2S_CHANNEL_FMT_STEREO,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = AUDIO_BUFFER_COUNT,
        .dma_buf_len = AUDIO_FRAME_SIZE,
        .use_apll = false,
        .intr_alloc_flags = 0
    };
    
    i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = AUDIO_I2S_CLOCK_PIN,
        .ws_io_num = AUDIO_I2S_WS_PIN,
        .data_out_num = -1,  /* Not used for input */
        .data_in_num = AUDIO_I2S_DATA_PIN
    };
    
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

/**
 * @brief Initialize DMA for audio data
 */
static void prvAudioDMAInit(void)
{
    /* Claim a DMA channel */
    audio_dma_channel = dma_claim_unused_channel(true);
    if (audio_dma_channel < 0)
    {
        printf("Failed to claim DMA channel for audio\n");
        return;
    }
    
    /* Configure DMA channel */
    dma_channel_config c = dma_channel_get_default_config(audio_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_I2S_RX);
    
    /* Set up DMA to transfer from I2S to memory */
    dma_channel_configure(
        audio_dma_channel,
        &c,
        ucAudioBuffers[ucCurrentBuffer],  /* Initial write address */
        &i2s_rx_fifo,                    /* Read address (I2S FIFO) */
        AUDIO_BUFFER_SIZE / 2,           /* Transfer count (16-bit samples) */
        false                            /* Don't start yet */
    );
    
    /* Set up DMA completion callback */
    dma_channel_set_irq0_enabled(audio_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, prvAudioDMACallback);
    irq_set_enabled(DMA_IRQ_0, true);
}

/**
 * @brief Initialize alert sounds
 */
static void prvAudioInitAlertSounds(void)
{
    /* Generate alert sounds */
    
    /* Motion detected alert (two-tone beep) */
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_MOTION_DETECTED], MAX_ALERT_SIZE / 2, 1000, 0.8);
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_MOTION_DETECTED] + MAX_ALERT_SIZE / 2, MAX_ALERT_SIZE / 2, 1200, 0.8);
    ulAlertSizes[AUDIO_ALERT_MOTION_DETECTED] = MAX_ALERT_SIZE;
    
    /* Person detected alert (three-tone beep) */
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_PERSON_DETECTED], MAX_ALERT_SIZE / 3, 1000, 0.8);
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_PERSON_DETECTED] + MAX_ALERT_SIZE / 3, MAX_ALERT_SIZE / 3, 1200, 0.8);
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_PERSON_DETECTED] + 2 * MAX_ALERT_SIZE / 3, MAX_ALERT_SIZE / 3, 1400, 0.8);
    ulAlertSizes[AUDIO_ALERT_PERSON_DETECTED] = MAX_ALERT_SIZE;
    
    /* Low battery alert (descending tone) */
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_LOW_BATTERY], MAX_ALERT_SIZE / 2, 800, 0.8);
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_LOW_BATTERY] + MAX_ALERT_SIZE / 2, MAX_ALERT_SIZE / 2, 400, 0.8);
    ulAlertSizes[AUDIO_ALERT_LOW_BATTERY] = MAX_ALERT_SIZE;
    
    /* System error alert (rapid beeps) */
    for (int i = 0; i < 5; i++)
    {
        prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_SYSTEM_ERROR] + i * MAX_ALERT_SIZE / 5, MAX_ALERT_SIZE / 10, 1000, 0.8);
    }
    ulAlertSizes[AUDIO_ALERT_SYSTEM_ERROR] = MAX_ALERT_SIZE / 2;
    
    /* Custom alert (placeholder) */
    prvGenerateSineWave(ucAlertSounds[AUDIO_ALERT_CUSTOM], MAX_ALERT_SIZE, 500, 0.8);
    ulAlertSizes[AUDIO_ALERT_CUSTOM] = MAX_ALERT_SIZE;
}

/**
 * @brief DMA completion callback
 */
static void prvAudioDMACallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Clear the interrupt request */
    dma_hw->ints0 = 1u << audio_dma_channel;
    
    /* Set timestamp */
    xAudioFrames[ucCurrentBuffer].timestamp = xTaskGetTickCount();
    
    /* Process audio frame */
    prvAudioProcessFrame(&xAudioFrames[ucCurrentBuffer]);
    
    /* Send frame to queue */
    AudioFrame_t *pxFrame = &xAudioFrames[ucCurrentBuffer];
    if (xQueueSendFromISR(xAudioFrameQueue, &pxFrame, &xHigherPriorityTaskWoken) != pdPASS)
    {
        /* Queue full, frame will be overwritten */
    }
    
    /* Switch to next buffer */
    ucProcessingBuffer = ucCurrentBuffer;
    ucCurrentBuffer = (ucCurrentBuffer + 1) % AUDIO_BUFFER_COUNT;
    
    /* Update DMA write address for next frame */
    dma_channel_set_write_addr(audio_dma_channel, ucAudioBuffers[ucCurrentBuffer], false);
    
    /* Restart DMA transfer */
    dma_channel_start(audio_dma_channel);
    
    /* Notify audio task that a new frame is available */
    if (xAudioTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(xAudioTaskHandle, &xHigherPriorityTaskWoken);
    }
    
    /* Yield if a higher priority task was woken */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Process audio frame
 * 
 * @param pxFrame Pointer to audio frame
 */
static void prvAudioProcessFrame(AudioFrame_t *pxFrame)
{
    /* Skip processing if detection is disabled */
    if (!ucDetectionEnabled)
    {
        return;
    }
    
    /* Process audio data for sound detection */
    int16_t *psSamples = (int16_t *)pxFrame->data;
    uint32_t ulNumSamples = pxFrame->size / sizeof(int16_t);
    
    /* Calculate RMS (root mean square) for sound level detection */
    float fRMS = prvAudioCalculateRMS(psSamples, ulNumSamples);
    
    /* Detect sounds if RMS is above threshold */
    if (fRMS > 0.1f)  /* Arbitrary threshold, would be calibrated */
    {
        prvAudioDetectSounds(psSamples, ulNumSamples);
    }
}

/**
 * @brief Calculate RMS (root mean square) of audio samples
 * 
 * @param psSamples Pointer to audio samples
 * @param ulNumSamples Number of samples
 * @return float RMS value (0.0-1.0)
 */
static float prvAudioCalculateRMS(const int16_t *psSamples, uint32_t ulNumSamples)
{
    float fSum = 0.0f;
    
    for (uint32_t i = 0; i < ulNumSamples; i++)
    {
        float fSample = psSamples[i] / 32768.0f;  /* Normalize to -1.0 to 1.0 */
        fSum += fSample * fSample;
    }
    
    return sqrtf(fSum / ulNumSamples);
}

/**
 * @brief Detect sounds in audio data
 * 
 * @param psSamples Pointer to audio samples
 * @param ulNumSamples Number of samples
 */
static void prvAudioDetectSounds(const int16_t *psSamples, uint32_t ulNumSamples)
{
    /* This is a simplified implementation */
    /* In a real implementation, this would use more sophisticated algorithms */
    
    /* Simple energy-based detection */
    float fEnergy = prvAudioCalculateRMS(psSamples, ulNumSamples);
    
    /* Detect based on energy level */
    if (fEnergy > 0.3f)  /* High energy threshold */
    {
        AudioDetectionResult_t xResult;
        xResult.type = AUDIO_DETECTION_SOUND;
        xResult.confidence = fEnergy;
        xResult.timestamp = xTaskGetTickCount();
        
        /* Send detection result to queue */
        xQueueSendFromISR(xAudioDetectionQueue, &xResult, NULL);
    }
}

/**
 * @brief Generate sine wave
 * 
 * @param pucBuffer Pointer to output buffer
 * @param ulSize Size of buffer in bytes
 * @param fFrequency Frequency in Hz
 * @param fAmplitude Amplitude (0.0-1.0)
 */
static void prvGenerateSineWave(uint8_t *pucBuffer, uint32_t ulSize, float fFrequency, float fAmplitude)
{
    int16_t *psSamples = (int16_t *)pucBuffer;
    uint32_t ulNumSamples = ulSize / sizeof(int16_t);
    
    for (uint32_t i = 0; i < ulNumSamples; i++)
    {
        float fTime = (float)i / AUDIO_SAMPLE_RATE;
        float fValue = sinf(2.0f * M_PI * fFrequency * fTime);
        
        /* Apply amplitude and convert to 16-bit PCM */
        psSamples[i] = (int16_t)(fValue * fAmplitude * 32767.0f);
    }
}
