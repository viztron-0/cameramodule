#include "camera.h"
#include "task_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* Include Raspberry Pi Pico specific headers */
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

/* I2C instance for camera control */
#define CAMERA_I2C_INSTANCE         i2c0
#define CAMERA_I2C_SDA_PIN          4
#define CAMERA_I2C_SCL_PIN          5

/* Camera control pins */
#define CAMERA_RESET_PIN            2
#define CAMERA_PWDN_PIN             3
#define CAMERA_VSYNC_PIN            6
#define CAMERA_HREF_PIN             7
#define CAMERA_PCLK_PIN             8
#define CAMERA_XCLK_PIN             9
#define CAMERA_D0_PIN               10  /* Data pins D0-D7 */
#define CAMERA_D1_PIN               11
#define CAMERA_D2_PIN               12
#define CAMERA_D3_PIN               13
#define CAMERA_D4_PIN               14
#define CAMERA_D5_PIN               15
#define CAMERA_D6_PIN               16
#define CAMERA_D7_PIN               17

/* DMA channel for camera data */
static int camera_dma_channel;

/* Camera status */
static CameraStatus_t xCameraStatus = {0};

/* Camera configuration */
static CameraConfig_t xCameraConfig = {
    .width = CAMERA_WIDTH,
    .height = CAMERA_HEIGHT,
    .format = CAMERA_PIXEL_FORMAT,
    .fps = 20,
    .brightness = 50,
    .contrast = 50,
    .saturation = 50,
    .effect = 0
};

/* Frame buffers */
#define FRAME_BUFFER_COUNT          2  /* Double buffering */
static uint8_t ucFrameBuffers[FRAME_BUFFER_COUNT][CAMERA_FRAME_SIZE] __attribute__((aligned(32)));
static CameraFrame_t xFrames[FRAME_BUFFER_COUNT];
static uint8_t ucCurrentBuffer = 0;
static uint8_t ucProcessingBuffer = 0;

/* OV2640 default configuration */
static const uint8_t ov2640_default_regs[][2] = {
    /* Reset all registers */
    {OV2640_BANK_SEL, OV2640_BANK_DSP},
    {OV2640_RESET, 0x01},
    {0xFF, 0xFF}, /* Delay */
    
    /* Set to RGB565 mode */
    {OV2640_BANK_SEL, OV2640_BANK_DSP},
    {OV2640_R_BYPASS, 0x00},
    {OV2640_IMAGE_MODE, 0x06}, /* RGB565 */
    
    /* Set QVGA resolution (320x240) */
    {OV2640_BANK_SEL, OV2640_BANK_DSP},
    {OV2640_RESET, 0x00},
    {OV2640_HSIZE, 0x50}, /* 320 >> 2 = 80 = 0x50 */
    {OV2640_VSIZE, 0x3C}, /* 240 >> 2 = 60 = 0x3C */
    {OV2640_XOFFL, 0x00},
    {OV2640_YOFFL, 0x00},
    {OV2640_VHYX, 0x00},
    {OV2640_TEST, 0x00},
    {OV2640_ZMOW, 0x50}, /* 320 >> 2 = 80 = 0x50 */
    {OV2640_ZMOH, 0x3C}, /* 240 >> 2 = 60 = 0x3C */
    
    /* Set clock and timing */
    {OV2640_BANK_SEL, OV2640_BANK_SENSOR},
    {OV2640_CLKRC, 0x01}, /* Clock divider */
    
    /* Set automatic exposure and white balance */
    {OV2640_BANK_SEL, OV2640_BANK_SENSOR},
    {OV2640_COM8, 0xF5}, /* Enable AEC, AWB, AEC */
    
    /* End of configuration */
    {0xFF, 0xFF}
};

/* Forward declarations of static functions */
static BaseType_t prvCameraI2CInit(void);
static BaseType_t prvCameraGPIOInit(void);
static BaseType_t prvCameraDMAInit(void);
static BaseType_t prvCameraWriteReg(uint8_t reg, uint8_t value);
static BaseType_t prvCameraReadReg(uint8_t reg, uint8_t *value);
static BaseType_t prvCameraSetBank(uint8_t bank);
static void prvCameraFrameCallback(void);
static void prvCameraVSyncCallback(uint gpio, uint32_t events);
static void prvCameraHRefCallback(uint gpio, uint32_t events);

/**
 * @brief Initialize the camera module
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraInit(void)
{
    BaseType_t xReturn;
    uint8_t id_high, id_low;
    
    /* Initialize camera status */
    memset(&xCameraStatus, 0, sizeof(CameraStatus_t));
    
    /* Initialize I2C for camera control */
    xReturn = prvCameraI2CInit();
    if (xReturn != pdPASS)
    {
        printf("Failed to initialize camera I2C\n");
        return pdFAIL;
    }
    
    /* Initialize GPIO pins for camera interface */
    xReturn = prvCameraGPIOInit();
    if (xReturn != pdPASS)
    {
        printf("Failed to initialize camera GPIO\n");
        return pdFAIL;
    }
    
    /* Initialize DMA for camera data */
    xReturn = prvCameraDMAInit();
    if (xReturn != pdPASS)
    {
        printf("Failed to initialize camera DMA\n");
        return pdFAIL;
    }
    
    /* Reset camera */
    gpio_put(CAMERA_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_put(CAMERA_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Check camera ID */
    prvCameraSetBank(OV2640_BANK_SENSOR);
    prvCameraReadReg(OV2640_PIDH, &id_high);
    prvCameraReadReg(OV2640_PIDL, &id_low);
    
    if (id_high != 0x26 || id_low != 0x42)
    {
        printf("Camera ID mismatch: expected 0x2642, got 0x%02X%02X\n", id_high, id_low);
        return pdFAIL;
    }
    
    printf("Camera ID: 0x%02X%02X\n", id_high, id_low);
    
    /* Configure camera with default settings */
    for (int i = 0; ov2640_default_regs[i][0] != 0xFF || ov2640_default_regs[i][1] != 0xFF; i++)
    {
        if (ov2640_default_regs[i][0] == 0xFF && ov2640_default_regs[i][1] == 0xFF)
        {
            /* Delay */
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            prvCameraWriteReg(ov2640_default_regs[i][0], ov2640_default_regs[i][1]);
        }
    }
    
    /* Initialize frame buffers */
    for (int i = 0; i < FRAME_BUFFER_COUNT; i++)
    {
        xFrames[i].data = ucFrameBuffers[i];
        xFrames[i].size = CAMERA_FRAME_SIZE;
        xFrames[i].width = CAMERA_WIDTH;
        xFrames[i].height = CAMERA_HEIGHT;
        xFrames[i].format = CAMERA_PIXEL_FORMAT;
        xFrames[i].timestamp = 0;
    }
    
    /* Set camera as initialized */
    xCameraStatus.initialized = 1;
    
    return pdPASS;
}

/**
 * @brief Configure the camera with specified settings
 * 
 * @param pxConfig Pointer to camera configuration structure
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraConfig(CameraConfig_t *pxConfig)
{
    if (pxConfig == NULL || !xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Store new configuration */
    memcpy(&xCameraConfig, pxConfig, sizeof(CameraConfig_t));
    
    /* Apply configuration */
    xCameraSetBrightness(pxConfig->brightness);
    xCameraSetContrast(pxConfig->contrast);
    xCameraSetSaturation(pxConfig->saturation);
    xCameraSetEffect(pxConfig->effect);
    
    /* Configure resolution and format if different from current */
    if (pxConfig->width != CAMERA_WIDTH || pxConfig->height != CAMERA_HEIGHT || 
        pxConfig->format != CAMERA_PIXEL_FORMAT)
    {
        /* This would require reconfiguring the camera and frame buffers */
        printf("Changing resolution or format not supported in this version\n");
    }
    
    return pdPASS;
}

/**
 * @brief Start camera frame capture
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraStart(void)
{
    if (!xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Enable VSYNC and HREF interrupts */
    gpio_set_irq_enabled(CAMERA_VSYNC_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CAMERA_HREF_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    /* Start DMA transfer */
    dma_channel_start(camera_dma_channel);
    
    return pdPASS;
}

/**
 * @brief Stop camera frame capture
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraStop(void)
{
    if (!xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Disable VSYNC and HREF interrupts */
    gpio_set_irq_enabled(CAMERA_VSYNC_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(CAMERA_HREF_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    
    /* Stop DMA transfer */
    dma_channel_abort(camera_dma_channel);
    
    return pdPASS;
}

/**
 * @brief Get a camera frame
 * 
 * @param pxFrame Pointer to frame structure to fill
 * @param xTimeout Timeout in ticks to wait for a frame
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraGetFrame(CameraFrame_t *pxFrame, TickType_t xTimeout)
{
    CameraFrame_t *pxQueuedFrame;
    
    if (pxFrame == NULL || !xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Get frame from queue */
    if (xQueueReceive(xFrameQueue, &pxQueuedFrame, xTimeout) != pdPASS)
    {
        xCameraStatus.frames_dropped++;
        return pdFAIL;
    }
    
    /* Copy frame data */
    pxFrame->data = pxQueuedFrame->data;
    pxFrame->size = pxQueuedFrame->size;
    pxFrame->width = pxQueuedFrame->width;
    pxFrame->height = pxQueuedFrame->height;
    pxFrame->format = pxQueuedFrame->format;
    pxFrame->timestamp = pxQueuedFrame->timestamp;
    
    return pdPASS;
}

/**
 * @brief Release a frame buffer
 * 
 * @param pxFrame Pointer to frame structure to release
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraReleaseFrame(CameraFrame_t *pxFrame)
{
    if (pxFrame == NULL || !xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Mark buffer as available for capture */
    for (int i = 0; i < FRAME_BUFFER_COUNT; i++)
    {
        if (pxFrame->data == xFrames[i].data)
        {
            /* Buffer is now available for capture */
            return pdPASS;
        }
    }
    
    return pdFAIL;
}

/**
 * @brief Get camera status
 * 
 * @param pxStatus Pointer to status structure to fill
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraGetStatus(CameraStatus_t *pxStatus)
{
    if (pxStatus == NULL)
    {
        return pdFAIL;
    }
    
    /* Copy status */
    memcpy(pxStatus, &xCameraStatus, sizeof(CameraStatus_t));
    
    return pdPASS;
}

/**
 * @brief Set camera brightness
 * 
 * @param ucBrightness Brightness value (0-100)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraSetBrightness(uint8_t ucBrightness)
{
    if (!xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Map 0-100 to camera register values */
    uint8_t reg_value = (ucBrightness * 255) / 100;
    
    /* Set brightness */
    prvCameraSetBank(OV2640_BANK_DSP);
    prvCameraWriteReg(0x9B, reg_value);
    
    /* Update configuration */
    xCameraConfig.brightness = ucBrightness;
    
    return pdPASS;
}

/**
 * @brief Set camera contrast
 * 
 * @param ucContrast Contrast value (0-100)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraSetContrast(uint8_t ucContrast)
{
    if (!xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Map 0-100 to camera register values */
    uint8_t reg_value = (ucContrast * 255) / 100;
    
    /* Set contrast */
    prvCameraSetBank(OV2640_BANK_DSP);
    prvCameraWriteReg(0x9C, reg_value);
    
    /* Update configuration */
    xCameraConfig.contrast = ucContrast;
    
    return pdPASS;
}

/**
 * @brief Set camera saturation
 * 
 * @param ucSaturation Saturation value (0-100)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraSetSaturation(uint8_t ucSaturation)
{
    if (!xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    /* Map 0-100 to camera register values */
    uint8_t reg_value = (ucSaturation * 255) / 100;
    
    /* Set saturation */
    prvCameraSetBank(OV2640_BANK_DSP);
    prvCameraWriteReg(0xA6, reg_value);
    prvCameraWriteReg(0xA7, reg_value);
    
    /* Update configuration */
    xCameraConfig.saturation = ucSaturation;
    
    return pdPASS;
}

/**
 * @brief Set camera special effect
 * 
 * @param ucEffect Effect value (0=normal, 1=negative, 2=grayscale, 3=sepia, etc.)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCameraSetEffect(uint8_t ucEffect)
{
    if (!xCameraStatus.initialized)
    {
        return pdFAIL;
    }
    
    prvCameraSetBank(OV2640_BANK_DSP);
    
    switch (ucEffect)
    {
        case 0: /* Normal */
            prvCameraWriteReg(0xA6, 0x00);
            prvCameraWriteReg(0xA7, 0x00);
            prvCameraWriteReg(0xA8, 0x00);
            prvCameraWriteReg(0xA9, 0x00);
            break;
            
        case 1: /* Negative */
            prvCameraWriteReg(0xA6, 0x40);
            prvCameraWriteReg(0xA7, 0x40);
            prvCameraWriteReg(0xA8, 0x00);
            prvCameraWriteReg(0xA9, 0x00);
            break;
            
        case 2: /* Grayscale */
            prvCameraWriteReg(0xA6, 0x80);
            prvCameraWriteReg(0xA7, 0x80);
            prvCameraWriteReg(0xA8, 0x00);
            prvCameraWriteReg(0xA9, 0x00);
            break;
            
        case 3: /* Sepia */
            prvCameraWriteReg(0xA6, 0x80);
            prvCameraWriteReg(0xA7, 0x80);
            prvCameraWriteReg(0xA8, 0x40);
            prvCameraWriteReg(0xA9, 0xA0);
            break;
            
        default:
            return pdFAIL;
    }
    
    /* Update configuration */
    xCameraConfig.effect = ucEffect;
    
    return pdPASS;
}

/**
 * @brief Initialize I2C for camera control
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvCameraI2CInit(void)
{
    /* Initialize I2C pins */
    gpio_set_function(CAMERA_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(CAMERA_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(CAMERA_I2C_SDA_PIN);
    gpio_pull_up(CAMERA_I2C_SCL_PIN);
    
    /* Initialize I2C with 100 kHz clock */
    i2c_init(CAMERA_I2C_INSTANCE, 100 * 1000);
    
    return pdPASS;
}

/**
 * @brief Initialize GPIO pins for camera interface
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvCameraGPIOInit(void)
{
    /* Initialize control pins */
    gpio_init(CAMERA_RESET_PIN);
    gpio_init(CAMERA_PWDN_PIN);
    gpio_set_dir(CAMERA_RESET_PIN, GPIO_OUT);
    gpio_set_dir(CAMERA_PWDN_PIN, GPIO_OUT);
    
    /* Initialize VSYNC and HREF pins with interrupt */
    gpio_init(CAMERA_VSYNC_PIN);
    gpio_init(CAMERA_HREF_PIN);
    gpio_set_dir(CAMERA_VSYNC_PIN, GPIO_IN);
    gpio_set_dir(CAMERA_HREF_PIN, GPIO_IN);
    gpio_set_irq_callback(prvCameraVSyncCallback);
    gpio_set_irq_callback(prvCameraHRefCallback);
    
    /* Initialize PCLK pin */
    gpio_init(CAMERA_PCLK_PIN);
    gpio_set_dir(CAMERA_PCLK_PIN, GPIO_IN);
    
    /* Initialize XCLK pin (output clock to camera) */
    gpio_init(CAMERA_XCLK_PIN);
    gpio_set_dir(CAMERA_XCLK_PIN, GPIO_OUT);
    
    /* Initialize data pins */
    for (int i = 0; i < 8; i++)
    {
        gpio_init(CAMERA_D0_PIN + i);
        gpio_set_dir(CAMERA_D0_PIN + i, GPIO_IN);
    }
    
    return pdPASS;
}

/**
 * @brief Initialize DMA for camera data
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvCameraDMAInit(void)
{
    /* Claim a DMA channel */
    camera_dma_channel = dma_claim_unused_channel(true);
    if (camera_dma_channel < 0)
    {
        return pdFAIL;
    }
    
    /* Configure DMA channel */
    dma_channel_config c = dma_channel_get_default_config(camera_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);  /* Use PIO for timing */
    
    /* Set up DMA to transfer from GPIO to memory */
    dma_channel_configure(
        camera_dma_channel,
        &c,
        ucFrameBuffers[ucCurrentBuffer],  /* Initial write address */
        NULL,                            /* Read address will be set by PIO */
        CAMERA_FRAME_SIZE,               /* Transfer count */
        false                            /* Don't start yet */
    );
    
    /* Set up DMA completion callback */
    dma_channel_set_irq0_enabled(camera_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, prvCameraFrameCallback);
    irq_set_enabled(DMA_IRQ_0, true);
    
    return pdPASS;
}

/**
 * @brief Write to camera register
 * 
 * @param reg Register address
 * @param value Value to write
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvCameraWriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    
    /* Take I2C mutex */
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Write register */
    int ret = i2c_write_blocking(CAMERA_I2C_INSTANCE, OV2640_ADDR >> 1, buf, 2, false);
    
    /* Release I2C mutex */
    xSemaphoreGive(xI2CMutex);
    
    return (ret == 2) ? pdPASS : pdFAIL;
}

/**
 * @brief Read from camera register
 * 
 * @param reg Register address
 * @param value Pointer to store read value
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvCameraReadReg(uint8_t reg, uint8_t *value)
{
    /* Take I2C mutex */
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Write register address */
    int ret = i2c_write_blocking(CAMERA_I2C_INSTANCE, OV2640_ADDR >> 1, &reg, 1, true);
    if (ret != 1)
    {
        xSemaphoreGive(xI2CMutex);
        return pdFAIL;
    }
    
    /* Read register value */
    ret = i2c_read_blocking(CAMERA_I2C_INSTANCE, OV2640_ADDR >> 1, value, 1, false);
    
    /* Release I2C mutex */
    xSemaphoreGive(xI2CMutex);
    
    return (ret == 1) ? pdPASS : pdFAIL;
}

/**
 * @brief Set camera register bank
 * 
 * @param bank Bank to select (OV2640_BANK_DSP or OV2640_BANK_SENSOR)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvCameraSetBank(uint8_t bank)
{
    return prvCameraWriteReg(OV2640_BANK_SEL, bank);
}

/**
 * @brief DMA completion callback
 */
static void prvCameraFrameCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Clear the interrupt request */
    dma_hw->ints0 = 1u << camera_dma_channel;
    
    /* Set timestamp */
    xFrames[ucCurrentBuffer].timestamp = xTaskGetTickCount();
    
    /* Send frame to queue */
    CameraFrame_t *pxFrame = &xFrames[ucCurrentBuffer];
    if (xQueueSendFromISR(xFrameQueue, &pxFrame, &xHigherPriorityTaskWoken) != pdPASS)
    {
        xCameraStatus.frames_dropped++;
    }
    else
    {
        xCameraStatus.frames_captured++;
    }
    
    /* Switch to next buffer */
    ucProcessingBuffer = ucCurrentBuffer;
    ucCurrentBuffer = (ucCurrentBuffer + 1) % FRAME_BUFFER_COUNT;
    
    /* Update DMA write address for next frame */
    dma_channel_set_write_addr(camera_dma_channel, ucFrameBuffers[ucCurrentBuffer], false);
    
    /* Calculate current FPS */
    static uint32_t last_time = 0;
    static uint32_t frame_count = 0;
    
    frame_count++;
    uint32_t current_time = xTaskGetTickCount();
    
    if (current_time - last_time >= pdMS_TO_TICKS(1000))
    {
        xCameraStatus.current_fps = frame_count;
        frame_count = 0;
        last_time = current_time;
    }
    
    /* Notify AI task that a new frame is available */
    if (xAITaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(xAITaskHandle, &xHigherPriorityTaskWoken);
    }
    
    /* Yield if a higher priority task was woken */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief VSYNC interrupt callback
 * 
 * @param gpio GPIO pin number
 * @param events GPIO events
 */
static void prvCameraVSyncCallback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        /* Start of frame */
        /* Reset DMA */
        dma_channel_abort(camera_dma_channel);
        dma_channel_set_write_addr(camera_dma_channel, ucFrameBuffers[ucCurrentBuffer], false);
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        /* End of frame */
        /* Start DMA for next frame */
        dma_channel_start(camera_dma_channel);
    }
}

/**
 * @brief HREF interrupt callback
 * 
 * @param gpio GPIO pin number
 * @param events GPIO events
 */
static void prvCameraHRefCallback(uint gpio, uint32_t events)
{
    /* This would be used for line-by-line processing if needed */
}
