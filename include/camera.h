#ifndef CAMERA_H
#define CAMERA_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>

/* OV2640 registers */
#define OV2640_ADDR                 (0x60)
#define OV2640_PID                  (0x0A)  /* Product ID MSB */
#define OV2640_VER                  (0x0B)  /* Product ID LSB */
#define OV2640_BANK_SEL             (0xFF)  /* Register Bank select */
#define OV2640_BANK_DSP             (0x00)
#define OV2640_BANK_SENSOR          (0x01)

/* OV2640 DSP registers (BANK_SEL = 0) */
#define OV2640_R_BYPASS             (0x05)
#define OV2640_QS                   (0x44)
#define OV2640_CTRLI                (0x50)
#define OV2640_HSIZE                (0x51)
#define OV2640_VSIZE                (0x52)
#define OV2640_XOFFL                (0x53)
#define OV2640_YOFFL                (0x54)
#define OV2640_VHYX                 (0x55)
#define OV2640_DPRP                 (0x56)
#define OV2640_TEST                 (0x57)
#define OV2640_ZMOW                 (0x5A)
#define OV2640_ZMOH                 (0x5B)
#define OV2640_ZMHH                 (0x5C)
#define OV2640_BPADDR               (0x7C)
#define OV2640_BPDATA               (0x7D)
#define OV2640_CTRL2                (0x86)
#define OV2640_CTRL3                (0x87)
#define OV2640_SIZEL                (0x8C)
#define OV2640_HSIZE8               (0xC0)
#define OV2640_VSIZE8               (0xC1)
#define OV2640_CTRL0                (0xC2)
#define OV2640_CTRL1                (0xC3)
#define OV2640_R_DVP_SP             (0xD3)
#define OV2640_IMAGE_MODE           (0xDA)
#define OV2640_RESET                (0xE0)
#define OV2640_MS_SP                (0xF0)
#define OV2640_SS_ID                (0xF7)
#define OV2640_SS_CTRL              (0xF8)
#define OV2640_MC_BIST              (0xF9)
#define OV2640_MC_AL                (0xFA)
#define OV2640_MC_AH                (0xFB)
#define OV2640_MC_D                 (0xFC)
#define OV2640_P_CMD                (0xFD)
#define OV2640_P_STATUS             (0xFE)

/* OV2640 Sensor registers (BANK_SEL = 1) */
#define OV2640_GAIN                 (0x00)
#define OV2640_COM1                 (0x03)
#define OV2640_REG04                (0x04)
#define OV2640_REG08                (0x08)
#define OV2640_COM2                 (0x09)
#define OV2640_PIDH                 (0x0A)
#define OV2640_PIDL                 (0x0B)
#define OV2640_COM3                 (0x0C)
#define OV2640_COM4                 (0x0D)
#define OV2640_AEC                  (0x10)
#define OV2640_CLKRC                (0x11)
#define OV2640_COM7                 (0x12)
#define OV2640_COM8                 (0x13)
#define OV2640_COM9                 (0x14)
#define OV2640_COM10                (0x15)
#define OV2640_HSTART               (0x17)
#define OV2640_HSTOP                (0x18)
#define OV2640_VSTART               (0x19)
#define OV2640_VSTOP                (0x1A)
#define OV2640_MIDH                 (0x1C)
#define OV2640_MIDL                 (0x1D)
#define OV2640_AEW                  (0x24)
#define OV2640_AEB                  (0x25)
#define OV2640_VV                   (0x26)
#define OV2640_REG2A                (0x2A)
#define OV2640_FRARL                (0x2B)
#define OV2640_ADDVSL               (0x2D)
#define OV2640_ADDVSH               (0x2E)
#define OV2640_YAVG                 (0x2F)
#define OV2640_HSDY                 (0x30)
#define OV2640_HEDY                 (0x31)
#define OV2640_REG32                (0x32)
#define OV2640_ARCOM2               (0x34)
#define OV2640_REG45                (0x45)
#define OV2640_FLL                  (0x46)
#define OV2640_FLH                  (0x47)
#define OV2640_COM19                (0x48)
#define OV2640_ZOOMS                (0x49)
#define OV2640_COM22                (0x4B)
#define OV2640_COM25                (0x4E)
#define OV2640_BD50                 (0x4F)
#define OV2640_BD60                 (0x50)
#define OV2640_REG5D                (0x5D)
#define OV2640_REG5E                (0x5E)
#define OV2640_REG5F                (0x5F)
#define OV2640_REG60                (0x60)
#define OV2640_HISTO_LOW            (0x61)
#define OV2640_HISTO_HIGH           (0x62)

/* Camera resolution */
#define CAMERA_WIDTH                (320)
#define CAMERA_HEIGHT               (240)
#define CAMERA_PIXEL_FORMAT         (2)  /* RGB565 */
#define CAMERA_FRAME_SIZE           (CAMERA_WIDTH * CAMERA_HEIGHT * CAMERA_PIXEL_FORMAT)

/* Camera frame structure */
typedef struct {
    uint8_t *data;                  /* Pointer to frame data */
    uint32_t size;                  /* Size of frame in bytes */
    uint32_t width;                 /* Frame width in pixels */
    uint32_t height;                /* Frame height in pixels */
    uint32_t format;                /* Pixel format (e.g., RGB565, YUV422) */
    uint32_t timestamp;             /* Frame capture timestamp */
} CameraFrame_t;

/* Camera configuration structure */
typedef struct {
    uint32_t width;                 /* Frame width in pixels */
    uint32_t height;                /* Frame height in pixels */
    uint32_t format;                /* Pixel format */
    uint32_t fps;                   /* Frames per second */
    uint8_t brightness;             /* Brightness level (0-100) */
    uint8_t contrast;               /* Contrast level (0-100) */
    uint8_t saturation;             /* Saturation level (0-100) */
    uint8_t effect;                 /* Special effect (normal, negative, etc.) */
} CameraConfig_t;

/* Camera status structure */
typedef struct {
    uint8_t initialized;            /* Camera initialization status */
    uint32_t frames_captured;       /* Total frames captured */
    uint32_t frames_dropped;        /* Total frames dropped */
    uint32_t current_fps;           /* Current frames per second */
    uint32_t last_error;            /* Last error code */
} CameraStatus_t;

/* Camera function prototypes */
BaseType_t xCameraInit(void);
BaseType_t xCameraConfig(CameraConfig_t *pxConfig);
BaseType_t xCameraStart(void);
BaseType_t xCameraStop(void);
BaseType_t xCameraGetFrame(CameraFrame_t *pxFrame, TickType_t xTimeout);
BaseType_t xCameraReleaseFrame(CameraFrame_t *pxFrame);
BaseType_t xCameraGetStatus(CameraStatus_t *pxStatus);
BaseType_t xCameraSetBrightness(uint8_t ucBrightness);
BaseType_t xCameraSetContrast(uint8_t ucContrast);
BaseType_t xCameraSetSaturation(uint8_t ucSaturation);
BaseType_t xCameraSetEffect(uint8_t ucEffect);

#endif /* CAMERA_H */
