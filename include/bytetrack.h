#ifndef BYTETRACK_H
#define BYTETRACK_H

#include "FreeRTOS.h"
#include "task.h"
#include "ai.h"
#include <stdint.h>

/* ByteTrack parameters */
#define MAX_TRACKS                  50
#define MAX_TRACK_HISTORY           30
#define HIGH_THRESHOLD              0.6f
#define LOW_THRESHOLD               0.1f
#define MAX_TIME_LOST               30  /* frames */

/* Track state enumeration */
typedef enum {
    TRACK_NEW,
    TRACK_TRACKED,
    TRACK_LOST,
    TRACK_REMOVED
} TrackState_t;

/* Track structure */
typedef struct {
    uint32_t id;                    /* Unique track ID */
    uint8_t class_id;               /* Class ID */
    float score;                    /* Detection score */
    float x;                        /* Center X */
    float y;                        /* Center Y */
    float width;                    /* Width */
    float height;                   /* Height */
    float vx;                       /* Velocity X */
    float vy;                       /* Velocity Y */
    TrackState_t state;             /* Track state */
    uint32_t time_since_update;     /* Frames since last update */
    uint32_t hits;                  /* Number of detections */
    uint32_t age;                   /* Track age in frames */
    float kalman_state[8];          /* Kalman filter state [x, y, w, h, vx, vy, vw, vh] */
    float kalman_covariance[8][8];  /* Kalman filter covariance */
} Track_t;

/* ByteTrack function prototypes */
void vByteTrackInit(void);
void vByteTrackUpdate(DetectionObject_t *pxDetections, uint8_t ucNumDetections);
uint8_t ucByteTrackGetActiveTracks(Track_t *pxTracks, uint8_t ucMaxTracks);
void vByteTrackApplyToDetections(DetectionObject_t *pxDetections, uint8_t ucNumDetections);

#endif /* BYTETRACK_H */
