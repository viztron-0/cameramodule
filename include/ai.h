#ifndef AI_H
#define AI_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "camera.h"
#include <stdint.h>

/* Detection object structure */
typedef struct {
    uint8_t class_id;               /* Class ID of detected object */
    float confidence;               /* Detection confidence (0.0-1.0) */
    float x;                        /* Bounding box center X (normalized 0.0-1.0) */
    float y;                        /* Bounding box center Y (normalized 0.0-1.0) */
    float width;                    /* Bounding box width (normalized 0.0-1.0) */
    float height;                   /* Bounding box height (normalized 0.0-1.0) */
    uint32_t track_id;              /* Tracking ID (assigned by ByteTrack) */
    float real_x;                   /* Real-world X coordinate (meters) */
    float real_y;                   /* Real-world Y coordinate (meters) */
} DetectionObject_t;

/* Detection result structure */
typedef struct {
    uint32_t frame_id;              /* Frame ID */
    uint32_t timestamp;             /* Timestamp of detection */
    uint8_t num_objects;            /* Number of detected objects */
    DetectionObject_t objects[20];  /* Array of detected objects (max 20) */
    float fps;                      /* Detection FPS */
    float inference_time;           /* Inference time in milliseconds */
} DetectionResult_t;

/* AI configuration structure */
typedef struct {
    float conf_threshold;           /* Confidence threshold (0.0-1.0) */
    float iou_threshold;            /* IOU threshold for NMS (0.0-1.0) */
    uint8_t enable_tracking;        /* Enable object tracking */
    uint8_t max_objects;            /* Maximum number of objects to detect */
} AIConfig_t;

/* AI function prototypes */
BaseType_t xAIInit(void);
BaseType_t xAIConfig(AIConfig_t *pxConfig);
BaseType_t xAIProcessFrame(CameraFrame_t *pxFrame);
BaseType_t xAIGetResult(DetectionResult_t *pxResult, TickType_t xTimeout);
BaseType_t xAIGetPerformanceMetrics(float *pxFPS, float *pxInferenceTime, float *pxmAP);
BaseType_t xAISetConfThreshold(float fConfThreshold);
BaseType_t xAISetIOUThreshold(float fIOUThreshold);
BaseType_t xAIEnableTracking(uint8_t ucEnable);

#endif /* AI_H */
