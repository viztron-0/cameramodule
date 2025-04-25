#include "ai.h"
#include "task_manager.h"
#include "camera.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Include TensorFlow Lite Micro for Raspberry Pi Pico */
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

/* YOLOv5n model parameters */
#define MODEL_WIDTH                 320
#define MODEL_HEIGHT                320
#define MODEL_CHANNELS              3
#define MODEL_INPUT_SIZE            (MODEL_WIDTH * MODEL_HEIGHT * MODEL_CHANNELS)
#define NUM_CLASSES                 80
#define ANCHORS_PER_GRID            3
#define NUM_GRIDS                   3
#define DETECTION_PER_GRID          (ANCHORS_PER_GRID * (NUM_CLASSES + 5))  /* 5 = x, y, w, h, confidence */

/* COCO dataset class names */
static const char* coco_classes[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

/* YOLOv5 anchor boxes for each grid scale (normalized to 0-1) */
static const float anchors[NUM_GRIDS][ANCHORS_PER_GRID][2] = {
    {{0.1, 0.1}, {0.2, 0.2}, {0.3, 0.3}},     /* Small objects */
    {{0.3, 0.3}, {0.4, 0.4}, {0.5, 0.5}},     /* Medium objects */
    {{0.5, 0.5}, {0.6, 0.6}, {0.7, 0.7}}      /* Large objects */
};

/* AI configuration */
static AIConfig_t xAIConfig = {
    .conf_threshold = 0.25f,
    .iou_threshold = 0.45f,
    .enable_tracking = 1,
    .max_objects = 20
};

/* AI performance metrics */
static struct {
    float fps;
    float inference_time;
    float mAP;
    uint32_t frames_processed;
    uint32_t detections_count;
} xAIMetrics = {0};

/* Detection result queue */
static QueueHandle_t xDetectionResultQueue;

/* TensorFlow Lite model and interpreter */
static const unsigned char model_data[] = {
    /* YOLOv5n quantized model data would be included here */
    /* This is a placeholder for the actual model data */
    0x00, 0x00, 0x00, 0x00
};

static tflite::MicroErrorReporter micro_error_reporter;
static tflite::AllOpsResolver resolver;
static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;

/* TensorFlow Lite memory arena */
#define TENSOR_ARENA_SIZE           (1024 * 1024)  /* 1MB tensor arena */
static uint8_t tensor_arena[TENSOR_ARENA_SIZE] __attribute__((aligned(16)));

/* Forward declarations of static functions */
static void prvPreprocessFrame(CameraFrame_t *pxFrame, float* input_data);
static void prvPostprocessDetections(float* output_data, DetectionResult_t *pxResult);
static float prvCalculateIOU(DetectionObject_t *box1, DetectionObject_t *box2);
static void prvNonMaxSuppression(DetectionObject_t *boxes, uint8_t *num_boxes);
static void prvByteTrack(DetectionObject_t *objects, uint8_t num_objects);
static void prvCalculateRealWorldCoordinates(DetectionObject_t *objects, uint8_t num_objects);

/**
 * @brief Initialize the AI module
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAIInit(void)
{
    /* Create detection result queue */
    xDetectionResultQueue = xQueueCreate(5, sizeof(DetectionResult_t));
    if (xDetectionResultQueue == NULL)
    {
        printf("Failed to create detection result queue\n");
        return pdFAIL;
    }
    
    /* Initialize TensorFlow Lite model */
    model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        printf("Model schema version mismatch: %d vs %d\n", model->version(), TFLITE_SCHEMA_VERSION);
        return pdFAIL;
    }
    
    /* Create interpreter */
    interpreter = new tflite::MicroInterpreter(
        model, resolver, tensor_arena, TENSOR_ARENA_SIZE, &micro_error_reporter);
    
    /* Allocate tensors */
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        printf("Failed to allocate tensors\n");
        return pdFAIL;
    }
    
    /* Check input tensor shape */
    TfLiteTensor* input = interpreter->input(0);
    if (input->dims->size != 4 || 
        input->dims->data[0] != 1 || 
        input->dims->data[1] != MODEL_HEIGHT || 
        input->dims->data[2] != MODEL_WIDTH || 
        input->dims->data[3] != MODEL_CHANNELS)
    {
        printf("Unexpected input tensor shape\n");
        return pdFAIL;
    }
    
    /* Initialize performance metrics */
    xAIMetrics.fps = 0.0f;
    xAIMetrics.inference_time = 0.0f;
    xAIMetrics.mAP = 0.0f;
    xAIMetrics.frames_processed = 0;
    xAIMetrics.detections_count = 0;
    
    printf("AI module initialized successfully\n");
    return pdPASS;
}

/**
 * @brief Configure the AI module
 * 
 * @param pxConfig Pointer to AI configuration structure
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAIConfig(AIConfig_t *pxConfig)
{
    if (pxConfig == NULL)
    {
        return pdFAIL;
    }
    
    /* Copy configuration */
    memcpy(&xAIConfig, pxConfig, sizeof(AIConfig_t));
    
    return pdPASS;
}

/**
 * @brief Process a camera frame for object detection
 * 
 * @param pxFrame Pointer to camera frame structure
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAIProcessFrame(CameraFrame_t *pxFrame)
{
    if (pxFrame == NULL || interpreter == NULL)
    {
        return pdFAIL;
    }
    
    /* Start timing for inference */
    TickType_t start_time = xTaskGetTickCount();
    
    /* Get input tensor */
    TfLiteTensor* input = interpreter->input(0);
    
    /* Preprocess frame */
    prvPreprocessFrame(pxFrame, input->data.f);
    
    /* Run inference */
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk)
    {
        printf("Inference failed\n");
        return pdFAIL;
    }
    
    /* Get output tensor */
    TfLiteTensor* output = interpreter->output(0);
    
    /* Create detection result */
    DetectionResult_t xResult;
    memset(&xResult, 0, sizeof(DetectionResult_t));
    xResult.frame_id = xAIMetrics.frames_processed;
    xResult.timestamp = pxFrame->timestamp;
    
    /* Postprocess detections */
    prvPostprocessDetections(output->data.f, &xResult);
    
    /* Calculate inference time */
    TickType_t end_time = xTaskGetTickCount();
    float inference_time = (float)(end_time - start_time) * portTICK_PERIOD_MS;
    
    /* Update performance metrics */
    xAIMetrics.inference_time = inference_time;
    xAIMetrics.frames_processed++;
    xAIMetrics.detections_count += xResult.num_objects;
    
    /* Calculate FPS using exponential moving average */
    if (inference_time > 0)
    {
        float current_fps = 1000.0f / inference_time;
        xAIMetrics.fps = (xAIMetrics.fps * 0.9f) + (current_fps * 0.1f);
    }
    
    /* Set FPS and inference time in result */
    xResult.fps = xAIMetrics.fps;
    xResult.inference_time = xAIMetrics.inference_time;
    
    /* Apply ByteTrack if enabled */
    if (xAIConfig.enable_tracking)
    {
        prvByteTrack(xResult.objects, xResult.num_objects);
    }
    
    /* Calculate real-world coordinates */
    prvCalculateRealWorldCoordinates(xResult.objects, xResult.num_objects);
    
    /* Send result to queue */
    if (xQueueSend(xDetectionResultQueue, &xResult, 0) != pdPASS)
    {
        printf("Failed to send detection result to queue\n");
        return pdFAIL;
    }
    
    /* Notify communication task that a new detection is available */
    if (xCommunicationTaskHandle != NULL)
    {
        xTaskNotify(xCommunicationTaskHandle, 0, eNoAction);
    }
    
    return pdPASS;
}

/**
 * @brief Get detection result
 * 
 * @param pxResult Pointer to detection result structure
 * @param xTimeout Timeout in ticks to wait for a result
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAIGetResult(DetectionResult_t *pxResult, TickType_t xTimeout)
{
    if (pxResult == NULL)
    {
        return pdFAIL;
    }
    
    /* Get result from queue */
    if (xQueueReceive(xDetectionResultQueue, pxResult, xTimeout) != pdPASS)
    {
        return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Get AI performance metrics
 * 
 * @param pxFPS Pointer to store FPS
 * @param pxInferenceTime Pointer to store inference time
 * @param pxmAP Pointer to store mAP
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAIGetPerformanceMetrics(float *pxFPS, float *pxInferenceTime, float *pxmAP)
{
    if (pxFPS != NULL)
    {
        *pxFPS = xAIMetrics.fps;
    }
    
    if (pxInferenceTime != NULL)
    {
        *pxInferenceTime = xAIMetrics.inference_time;
    }
    
    if (pxmAP != NULL)
    {
        *pxmAP = xAIMetrics.mAP;
    }
    
    return pdPASS;
}

/**
 * @brief Set confidence threshold
 * 
 * @param fConfThreshold Confidence threshold (0.0-1.0)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAISetConfThreshold(float fConfThreshold)
{
    if (fConfThreshold < 0.0f || fConfThreshold > 1.0f)
    {
        return pdFAIL;
    }
    
    xAIConfig.conf_threshold = fConfThreshold;
    return pdPASS;
}

/**
 * @brief Set IOU threshold for NMS
 * 
 * @param fIOUThreshold IOU threshold (0.0-1.0)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAISetIOUThreshold(float fIOUThreshold)
{
    if (fIOUThreshold < 0.0f || fIOUThreshold > 1.0f)
    {
        return pdFAIL;
    }
    
    xAIConfig.iou_threshold = fIOUThreshold;
    return pdPASS;
}

/**
 * @brief Enable or disable object tracking
 * 
 * @param ucEnable 1 to enable, 0 to disable
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xAIEnableTracking(uint8_t ucEnable)
{
    xAIConfig.enable_tracking = ucEnable ? 1 : 0;
    return pdPASS;
}

/**
 * @brief Preprocess camera frame for model input
 * 
 * @param pxFrame Pointer to camera frame structure
 * @param input_data Pointer to model input data
 */
static void prvPreprocessFrame(CameraFrame_t *pxFrame, float* input_data)
{
    /* Convert RGB565 to RGB888 and normalize to 0-1 */
    uint16_t* src = (uint16_t*)pxFrame->data;
    
    for (int y = 0; y < MODEL_HEIGHT; y++)
    {
        for (int x = 0; x < MODEL_WIDTH; x++)
        {
            /* Calculate source coordinates with bilinear interpolation if needed */
            int src_x = x * pxFrame->width / MODEL_WIDTH;
            int src_y = y * pxFrame->height / MODEL_HEIGHT;
            int src_idx = src_y * pxFrame->width + src_x;
            
            /* Extract RGB components from RGB565 */
            uint16_t pixel = src[src_idx];
            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;
            uint8_t b = (pixel & 0x1F) << 3;
            
            /* Normalize to 0-1 and store in CHW format */
            int dst_idx = y * MODEL_WIDTH + x;
            input_data[dst_idx] = r / 255.0f;
            input_data[MODEL_HEIGHT * MODEL_WIDTH + dst_idx] = g / 255.0f;
            input_data[2 * MODEL_HEIGHT * MODEL_WIDTH + dst_idx] = b / 255.0f;
        }
    }
}

/**
 * @brief Postprocess model output to get detections
 * 
 * @param output_data Pointer to model output data
 * @param pxResult Pointer to detection result structure
 */
static void prvPostprocessDetections(float* output_data, DetectionResult_t *pxResult)
{
    /* YOLOv5 outputs are in the format [x, y, w, h, confidence, class_scores...] */
    /* This is a simplified implementation of YOLOv5 postprocessing */
    
    uint8_t num_detections = 0;
    DetectionObject_t detections[100];  /* Temporary buffer for all detections */
    
    /* Process each grid scale */
    for (int grid = 0; grid < NUM_GRIDS; grid++)
    {
        int grid_size = MODEL_WIDTH / (8 * (1 << grid));  /* 40, 20, 10 for 320x320 input */
        int stride = MODEL_WIDTH / grid_size;
        
        /* Process each cell in the grid */
        for (int cy = 0; cy < grid_size; cy++)
        {
            for (int cx = 0; cx < grid_size; cx++)
            {
                /* Process each anchor */
                for (int a = 0; a < ANCHORS_PER_GRID; a++)
                {
                    /* Calculate offset in the output tensor */
                    int offset = grid * (grid_size * grid_size * DETECTION_PER_GRID) +
                                 cy * grid_size * DETECTION_PER_GRID +
                                 cx * DETECTION_PER_GRID +
                                 a * (NUM_CLASSES + 5);
                    
                    /* Get box confidence */
                    float confidence = output_data[offset + 4];
                    
                    /* Skip low confidence detections */
                    if (confidence < xAIConfig.conf_threshold)
                    {
                        continue;
                    }
                    
                    /* Find class with highest score */
                    float max_class_score = 0;
                    int max_class_id = 0;
                    
                    for (int c = 0; c < NUM_CLASSES; c++)
                    {
                        float class_score = output_data[offset + 5 + c];
                        if (class_score > max_class_score)
                        {
                            max_class_score = class_score;
                            max_class_id = c;
                        }
                    }
                    
                    /* Calculate final confidence */
                    float final_confidence = confidence * max_class_score;
                    
                    /* Skip low confidence detections */
                    if (final_confidence < xAIConfig.conf_threshold)
                    {
                        continue;
                    }
                    
                    /* Get bounding box coordinates */
                    float x = (output_data[offset] * 2 - 0.5 + cx) / grid_size;  /* Normalized 0-1 */
                    float y = (output_data[offset + 1] * 2 - 0.5 + cy) / grid_size;
                    float w = pow(output_data[offset + 2] * 2, 2) * anchors[grid][a][0];
                    float h = pow(output_data[offset + 3] * 2, 2) * anchors[grid][a][1];
                    
                    /* Ensure coordinates are within bounds */
                    x = fmaxf(0, fminf(1, x));
                    y = fmaxf(0, fminf(1, y));
                    w = fmaxf(0, fminf(1, w));
                    h = fmaxf(0, fminf(1, h));
                    
                    /* Add detection to list */
                    if (num_detections < 100)
                    {
                        detections[num_detections].class_id = max_class_id;
                        detections[num_detections].confidence = final_confidence;
                        detections[num_detections].x = x;
                        detections[num_detections].y = y;
                        detections[num_detections].width = w;
                        detections[num_detections].height = h;
                        detections[num_detections].track_id = 0;  /* Will be assigned by ByteTrack */
                        num_detections++;
                    }
                }
            }
        }
    }
    
    /* Apply non-maximum suppression */
    prvNonMaxSuppression(detections, &num_detections);
    
    /* Copy detections to result */
    pxResult->num_objects = (num_detections > xAIConfig.max_objects) ? xAIConfig.max_objects : num_detections;
    
    for (int i = 0; i < pxResult->num_objects; i++)
    {
        memcpy(&pxResult->objects[i], &detections[i], sizeof(DetectionObject_t));
    }
}

/**
 * @brief Calculate Intersection over Union (IOU) between two bounding boxes
 * 
 * @param box1 Pointer to first bounding box
 * @param box2 Pointer to second bounding box
 * @return float IOU value (0.0-1.0)
 */
static float prvCalculateIOU(DetectionObject_t *box1, DetectionObject_t *box2)
{
    /* Calculate box coordinates */
    float x1_min = box1->x - box1->width / 2;
    float y1_min = box1->y - box1->height / 2;
    float x1_max = box1->x + box1->width / 2;
    float y1_max = box1->y + box1->height / 2;
    
    float x2_min = box2->x - box2->width / 2;
    float y2_min = box2->y - box2->height / 2;
    float x2_max = box2->x + box2->width / 2;
    float y2_max = box2->y + box2->height / 2;
    
    /* Calculate intersection area */
    float x_overlap = fmaxf(0, fminf(x1_max, x2_max) - fmaxf(x1_min, x2_min));
    float y_overlap = fmaxf(0, fminf(y1_max, y2_max) - fmaxf(y1_min, y2_min));
    float intersection = x_overlap * y_overlap;
    
    /* Calculate union area */
    float area1 = box1->width * box1->height;
    float area2 = box2->width * box2->height;
    float union_area = area1 + area2 - intersection;
    
    /* Calculate IOU */
    if (union_area > 0)
    {
        return intersection / union_area;
    }
    
    return 0;
}

/**
 * @brief Apply Non-Maximum Suppression to remove overlapping detections
 * 
 * @param boxes Array of detection objects
 * @param num_boxes Pointer to number of boxes (will be updated)
 */
static void prvNonMaxSuppression(DetectionObject_t *boxes, uint8_t *num_boxes)
{
    if (*num_boxes == 0)
    {
        return;
    }
    
    /* Sort boxes by confidence (descending) */
    for (int i = 0; i < *num_boxes - 1; i++)
    {
        for (int j = i + 1; j < *num_boxes; j++)
        {
            if (boxes[j].confidence > boxes[i].confidence)
            {
                /* Swap boxes */
                DetectionObject_t temp = boxes[i];
                boxes[i] = boxes[j];
                boxes[j] = temp;
            }
        }
    }
    
    /* NMS algorithm */
    uint8_t keep[100] = {0};  /* 1 = keep, 0 = discard */
    
    for (int i = 0; i < *num_boxes; i++)
    {
        keep[i] = 1;  /* Assume we keep this box */
        
        /* Check against all higher confidence boxes */
        for (int j = 0; j < i; j++)
        {
            /* If we're keeping the higher confidence box and they overlap significantly */
            if (keep[j] && boxes[i].class_id == boxes[j].class_id)
            {
                float iou = prvCalculateIOU(&boxes[i], &boxes[j]);
                if (iou > xAIConfig.iou_threshold)
                {
                    keep[i] = 0;  /* Discard this box */
                    break;
                }
            }
        }
    }
    
    /* Compact the array */
    int new_count = 0;
    for (int i = 0; i < *num_boxes; i++)
    {
        if (keep[i])
        {
            if (i != new_count)
            {
                boxes[new_count] = boxes[i];
            }
            new_count++;
        }
    }
    
    *num_boxes = new_count;
}

/**
 * @brief Apply ByteTrack algorithm for object tracking
 * 
 * @param objects Array of detection objects
 * @param num_objects Number of objects
 */
static void prvByteTrack(DetectionObject_t *objects, uint8_t num_objects)
{
    /* This is a simplified implementation of ByteTrack */
    /* In a real implementation, this would maintain track history and use Kalman filtering */
    
    static uint32_t next_track_id = 1;
    static DetectionObject_t previous_objects[20];
    static uint8_t previous_num_objects = 0;
    
    /* If this is the first frame, assign new track IDs to all objects */
    if (previous_num_objects == 0)
    {
        for (int i = 0; i < num_objects; i++)
        {
            objects[i].track_id = next_track_id++;
        }
    }
    else
    {
        /* For each current detection, find the best match in previous detections */
        for (int i = 0; i < num_objects; i++)
        {
            float best_iou = 0;
            int best_match = -1;
            
            for (int j = 0; j < previous_num_objects; j++)
            {
                /* Only match objects of the same class */
                if (objects[i].class_id == previous_objects[j].class_id)
                {
                    float iou = prvCalculateIOU(&objects[i], &previous_objects[j]);
                    if (iou > best_iou)
                    {
                        best_iou = iou;
                        best_match = j;
                    }
                }
            }
            
            /* If a good match is found, assign the same track ID */
            if (best_iou > 0.5 && best_match >= 0)
            {
                objects[i].track_id = previous_objects[best_match].track_id;
            }
            else
            {
                /* Otherwise, assign a new track ID */
                objects[i].track_id = next_track_id++;
            }
        }
    }
    
    /* Save current objects for next frame */
    previous_num_objects = (num_objects > 20) ? 20 : num_objects;
    memcpy(previous_objects, objects, previous_num_objects * sizeof(DetectionObject_t));
}

/**
 * @brief Calculate real-world coordinates for detected objects
 * 
 * @param objects Array of detection objects
 * @param num_objects Number of objects
 */
static void prvCalculateRealWorldCoordinates(DetectionObject_t *objects, uint8_t num_objects)
{
    /* This is a simplified implementation */
    /* In a real implementation, this would use camera calibration and GPS data */
    
    /* Assume a simple pinhole camera model */
    /* Assume the camera is at origin (0,0) and looking along the z-axis */
    /* Assume a field of view of 60 degrees */
    
    const float focal_length = 1.0f;  /* Normalized focal length */
    const float fov_x = 60.0f * M_PI / 180.0f;  /* Field of view in radians */
    const float fov_y = 60.0f * M_PI / 180.0f;
    
    /* Assume objects are at a fixed distance */
    const float default_distance = 5.0f;  /* 5 meters */
    
    for (int i = 0; i < num_objects; i++)
    {
        /* Convert normalized coordinates to camera coordinates */
        float cam_x = (objects[i].x - 0.5f) * 2.0f * tanf(fov_x / 2.0f);
        float cam_y = (objects[i].y - 0.5f) * 2.0f * tanf(fov_y / 2.0f);
        
        /* Calculate real-world coordinates */
        objects[i].real_x = cam_x * default_distance;
        objects[i].real_y = cam_y * default_distance;
    }
}
