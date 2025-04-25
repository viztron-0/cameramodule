#include "bytetrack.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Static track list */
static Track_t xTracks[MAX_TRACKS];
static uint8_t ucNumTracks = 0;
static uint32_t ulNextTrackID = 1;

/* Kalman filter parameters */
#define KALMAN_PROCESS_NOISE        0.01f
#define KALMAN_MEASUREMENT_NOISE    0.1f

/* Forward declarations of static functions */
static float prvCalculateIOU(Track_t *pxTrack, DetectionObject_t *pxDetection);
static void prvInitializeTrack(Track_t *pxTrack, DetectionObject_t *pxDetection);
static void prvUpdateTrack(Track_t *pxTrack, DetectionObject_t *pxDetection);
static void prvPredictNewLocations(void);
static void prvInitializeKalmanFilter(Track_t *pxTrack);
static void prvKalmanPredict(Track_t *pxTrack);
static void prvKalmanUpdate(Track_t *pxTrack, float x, float y, float width, float height);
static void prvManageTracks(void);

/**
 * @brief Initialize ByteTrack
 */
void vByteTrackInit(void)
{
    /* Initialize track list */
    memset(xTracks, 0, sizeof(xTracks));
    ucNumTracks = 0;
    ulNextTrackID = 1;
    
    printf("ByteTrack initialized\n");
}

/**
 * @brief Update tracks with new detections
 * 
 * @param pxDetections Array of detections
 * @param ucNumDetections Number of detections
 */
void vByteTrackUpdate(DetectionObject_t *pxDetections, uint8_t ucNumDetections)
{
    if (pxDetections == NULL || ucNumDetections == 0)
    {
        /* Just update track states if no detections */
        prvPredictNewLocations();
        prvManageTracks();
        return;
    }
    
    /* Step 1: Predict new locations of existing tracks */
    prvPredictNewLocations();
    
    /* Step 2: Associate detections with tracks */
    
    /* First, handle high-confidence detections */
    uint8_t ucHighDetections[MAX_TRACKS] = {0};  /* 1 if detection is high confidence and matched */
    uint8_t ucMatchedTracks[MAX_TRACKS] = {0};   /* 1 if track is matched */
    
    /* Match high-confidence detections to tracks */
    for (int i = 0; i < ucNumDetections; i++)
    {
        if (pxDetections[i].confidence >= HIGH_THRESHOLD)
        {
            float fBestIOU = 0.0f;
            int iBestMatch = -1;
            
            for (int j = 0; j < ucNumTracks; j++)
            {
                if (xTracks[j].state != TRACK_REMOVED && 
                    xTracks[j].class_id == pxDetections[i].class_id && 
                    !ucMatchedTracks[j])
                {
                    float fIOU = prvCalculateIOU(&xTracks[j], &pxDetections[i]);
                    if (fIOU > fBestIOU)
                    {
                        fBestIOU = fIOU;
                        iBestMatch = j;
                    }
                }
            }
            
            /* If a good match is found, update the track */
            if (fBestIOU > 0.3f && iBestMatch >= 0)
            {
                prvUpdateTrack(&xTracks[iBestMatch], &pxDetections[i]);
                ucHighDetections[i] = 1;
                ucMatchedTracks[iBestMatch] = 1;
                
                /* Assign track ID to detection */
                pxDetections[i].track_id = xTracks[iBestMatch].id;
            }
        }
    }
    
    /* Second, handle low-confidence detections */
    for (int i = 0; i < ucNumDetections; i++)
    {
        if (!ucHighDetections[i] && pxDetections[i].confidence >= LOW_THRESHOLD)
        {
            float fBestIOU = 0.0f;
            int iBestMatch = -1;
            
            for (int j = 0; j < ucNumTracks; j++)
            {
                if (xTracks[j].state != TRACK_REMOVED && 
                    xTracks[j].class_id == pxDetections[i].class_id && 
                    !ucMatchedTracks[j])
                {
                    float fIOU = prvCalculateIOU(&xTracks[j], &pxDetections[i]);
                    if (fIOU > fBestIOU)
                    {
                        fBestIOU = fIOU;
                        iBestMatch = j;
                    }
                }
            }
            
            /* If a good match is found, update the track */
            if (fBestIOU > 0.3f && iBestMatch >= 0)
            {
                prvUpdateTrack(&xTracks[iBestMatch], &pxDetections[i]);
                ucMatchedTracks[iBestMatch] = 1;
                
                /* Assign track ID to detection */
                pxDetections[i].track_id = xTracks[iBestMatch].id;
            }
        }
    }
    
    /* Step 3: Create new tracks for unmatched high-confidence detections */
    for (int i = 0; i < ucNumDetections; i++)
    {
        if (!ucHighDetections[i] && pxDetections[i].confidence >= HIGH_THRESHOLD)
        {
            if (ucNumTracks < MAX_TRACKS)
            {
                prvInitializeTrack(&xTracks[ucNumTracks], &pxDetections[i]);
                
                /* Assign track ID to detection */
                pxDetections[i].track_id = xTracks[ucNumTracks].id;
                
                ucNumTracks++;
            }
        }
    }
    
    /* Step 4: Update track states */
    prvManageTracks();
}

/**
 * @brief Get active tracks
 * 
 * @param pxTracks Array to store active tracks
 * @param ucMaxTracks Maximum number of tracks to return
 * @return uint8_t Number of active tracks
 */
uint8_t ucByteTrackGetActiveTracks(Track_t *pxTracks, uint8_t ucMaxTracks)
{
    if (pxTracks == NULL || ucMaxTracks == 0)
    {
        return 0;
    }
    
    uint8_t ucCount = 0;
    
    for (int i = 0; i < ucNumTracks && ucCount < ucMaxTracks; i++)
    {
        if (xTracks[i].state == TRACK_TRACKED)
        {
            memcpy(&pxTracks[ucCount], &xTracks[i], sizeof(Track_t));
            ucCount++;
        }
    }
    
    return ucCount;
}

/**
 * @brief Apply tracking results to detections
 * 
 * @param pxDetections Array of detections
 * @param ucNumDetections Number of detections
 */
void vByteTrackApplyToDetections(DetectionObject_t *pxDetections, uint8_t ucNumDetections)
{
    if (pxDetections == NULL || ucNumDetections == 0)
    {
        return;
    }
    
    /* For each detection, find the corresponding track and update with Kalman-filtered position */
    for (int i = 0; i < ucNumDetections; i++)
    {
        uint32_t ulTrackID = pxDetections[i].track_id;
        
        if (ulTrackID > 0)
        {
            for (int j = 0; j < ucNumTracks; j++)
            {
                if (xTracks[j].id == ulTrackID)
                {
                    /* Update detection with smoothed track position */
                    pxDetections[i].x = xTracks[j].x;
                    pxDetections[i].y = xTracks[j].y;
                    pxDetections[i].width = xTracks[j].width;
                    pxDetections[i].height = xTracks[j].height;
                    break;
                }
            }
        }
    }
}

/**
 * @brief Calculate IOU between track and detection
 * 
 * @param pxTrack Pointer to track
 * @param pxDetection Pointer to detection
 * @return float IOU value (0.0-1.0)
 */
static float prvCalculateIOU(Track_t *pxTrack, DetectionObject_t *pxDetection)
{
    /* Calculate box coordinates */
    float x1_min = pxTrack->x - pxTrack->width / 2;
    float y1_min = pxTrack->y - pxTrack->height / 2;
    float x1_max = pxTrack->x + pxTrack->width / 2;
    float y1_max = pxTrack->y + pxTrack->height / 2;
    
    float x2_min = pxDetection->x - pxDetection->width / 2;
    float y2_min = pxDetection->y - pxDetection->height / 2;
    float x2_max = pxDetection->x + pxDetection->width / 2;
    float y2_max = pxDetection->y + pxDetection->height / 2;
    
    /* Calculate intersection area */
    float x_overlap = fmaxf(0, fminf(x1_max, x2_max) - fmaxf(x1_min, x2_min));
    float y_overlap = fmaxf(0, fminf(y1_max, y2_max) - fmaxf(y1_min, y2_min));
    float intersection = x_overlap * y_overlap;
    
    /* Calculate union area */
    float area1 = pxTrack->width * pxTrack->height;
    float area2 = pxDetection->width * pxDetection->height;
    float union_area = area1 + area2 - intersection;
    
    /* Calculate IOU */
    if (union_area > 0)
    {
        return intersection / union_area;
    }
    
    return 0;
}

/**
 * @brief Initialize a new track
 * 
 * @param pxTrack Pointer to track
 * @param pxDetection Pointer to detection
 */
static void prvInitializeTrack(Track_t *pxTrack, DetectionObject_t *pxDetection)
{
    /* Initialize track with detection */
    pxTrack->id = ulNextTrackID++;
    pxTrack->class_id = pxDetection->class_id;
    pxTrack->score = pxDetection->confidence;
    pxTrack->x = pxDetection->x;
    pxTrack->y = pxDetection->y;
    pxTrack->width = pxDetection->width;
    pxTrack->height = pxDetection->height;
    pxTrack->vx = 0;
    pxTrack->vy = 0;
    pxTrack->state = TRACK_NEW;
    pxTrack->time_since_update = 0;
    pxTrack->hits = 1;
    pxTrack->age = 1;
    
    /* Initialize Kalman filter */
    prvInitializeKalmanFilter(pxTrack);
}

/**
 * @brief Update track with new detection
 * 
 * @param pxTrack Pointer to track
 * @param pxDetection Pointer to detection
 */
static void prvUpdateTrack(Track_t *pxTrack, DetectionObject_t *pxDetection)
{
    /* Update track with detection */
    pxTrack->time_since_update = 0;
    pxTrack->hits++;
    pxTrack->score = pxDetection->confidence;
    
    /* If track is new and has enough hits, mark as tracked */
    if (pxTrack->state == TRACK_NEW && pxTrack->hits >= 3)
    {
        pxTrack->state = TRACK_TRACKED;
    }
    
    /* If track was lost, mark as tracked again */
    if (pxTrack->state == TRACK_LOST)
    {
        pxTrack->state = TRACK_TRACKED;
    }
    
    /* Update Kalman filter with new measurement */
    prvKalmanUpdate(pxTrack, pxDetection->x, pxDetection->y, pxDetection->width, pxDetection->height);
}

/**
 * @brief Predict new locations of all tracks
 */
static void prvPredictNewLocations(void)
{
    for (int i = 0; i < ucNumTracks; i++)
    {
        if (xTracks[i].state != TRACK_REMOVED)
        {
            /* Predict new location using Kalman filter */
            prvKalmanPredict(&xTracks[i]);
            
            /* Increment age and time since update */
            xTracks[i].age++;
            xTracks[i].time_since_update++;
        }
    }
}

/**
 * @brief Initialize Kalman filter for track
 * 
 * @param pxTrack Pointer to track
 */
static void prvInitializeKalmanFilter(Track_t *pxTrack)
{
    /* State: [x, y, w, h, vx, vy, vw, vh] */
    
    /* Initialize state */
    pxTrack->kalman_state[0] = pxTrack->x;
    pxTrack->kalman_state[1] = pxTrack->y;
    pxTrack->kalman_state[2] = pxTrack->width;
    pxTrack->kalman_state[3] = pxTrack->height;
    pxTrack->kalman_state[4] = 0;  /* vx */
    pxTrack->kalman_state[5] = 0;  /* vy */
    pxTrack->kalman_state[6] = 0;  /* vw */
    pxTrack->kalman_state[7] = 0;  /* vh */
    
    /* Initialize covariance matrix */
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            pxTrack->kalman_covariance[i][j] = 0;
        }
        pxTrack->kalman_covariance[i][i] = 1.0f;  /* Identity matrix */
    }
}

/**
 * @brief Kalman filter prediction step
 * 
 * @param pxTrack Pointer to track
 */
static void prvKalmanPredict(Track_t *pxTrack)
{
    /* State transition matrix (simplified constant velocity model) */
    /* x' = x + vx */
    /* y' = y + vy */
    /* w' = w + vw */
    /* h' = h + vh */
    /* vx' = vx */
    /* vy' = vy */
    /* vw' = vw */
    /* vh' = vh */
    
    /* Predict state */
    float predicted_state[8];
    predicted_state[0] = pxTrack->kalman_state[0] + pxTrack->kalman_state[4];  /* x + vx */
    predicted_state[1] = pxTrack->kalman_state[1] + pxTrack->kalman_state[5];  /* y + vy */
    predicted_state[2] = pxTrack->kalman_state[2] + pxTrack->kalman_state[6];  /* w + vw */
    predicted_state[3] = pxTrack->kalman_state[3] + pxTrack->kalman_state[7];  /* h + vh */
    predicted_state[4] = pxTrack->kalman_state[4];  /* vx */
    predicted_state[5] = pxTrack->kalman_state[5];  /* vy */
    predicted_state[6] = pxTrack->kalman_state[6];  /* vw */
    predicted_state[7] = pxTrack->kalman_state[7];  /* vh */
    
    /* Predict covariance */
    /* P' = F*P*F^T + Q */
    /* Simplified: add process noise to diagonal */
    float predicted_covariance[8][8];
    memcpy(predicted_covariance, pxTrack->kalman_covariance, sizeof(predicted_covariance));
    
    for (int i = 0; i < 8; i++)
    {
        predicted_covariance[i][i] += KALMAN_PROCESS_NOISE;
    }
    
    /* Update track state and covariance */
    memcpy(pxTrack->kalman_state, predicted_state, sizeof(predicted_state));
    memcpy(pxTrack->kalman_covariance, predicted_covariance, sizeof(predicted_covariance));
    
    /* Update track position and velocity */
    pxTrack->x = pxTrack->kalman_state[0];
    pxTrack->y = pxTrack->kalman_state[1];
    pxTrack->width = pxTrack->kalman_state[2];
    pxTrack->height = pxTrack->kalman_state[3];
    pxTrack->vx = pxTrack->kalman_state[4];
    pxTrack->vy = pxTrack->kalman_state[5];
    
    /* Ensure width and height are positive */
    if (pxTrack->width < 0.01f) pxTrack->width = 0.01f;
    if (pxTrack->height < 0.01f) pxTrack->height = 0.01f;
    
    /* Ensure coordinates are within bounds */
    pxTrack->x = fmaxf(0, fminf(1, pxTrack->x));
    pxTrack->y = fmaxf(0, fminf(1, pxTrack->y));
    pxTrack->width = fmaxf(0.01f, fminf(1, pxTrack->width));
    pxTrack->height = fmaxf(0.01f, fminf(1, pxTrack->height));
}

/**
 * @brief Kalman filter update step
 * 
 * @param pxTrack Pointer to track
 * @param x Measured x
 * @param y Measured y
 * @param width Measured width
 * @param height Measured height
 */
static void prvKalmanUpdate(Track_t *pxTrack, float x, float y, float width, float height)
{
    /* Measurement vector */
    float measurement[4] = {x, y, width, height};
    
    /* Kalman gain calculation (simplified) */
    float kalman_gain[8][4];
    
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j)  /* Only update position and size, not velocity */
            {
                /* Simplified Kalman gain calculation */
                float innovation_covariance = pxTrack->kalman_covariance[i][i] + KALMAN_MEASUREMENT_NOISE;
                kalman_gain[i][j] = pxTrack->kalman_covariance[i][i] / innovation_covariance;
            }
            else
            {
                kalman_gain[i][j] = 0;
            }
        }
    }
    
    /* Update state */
    /* x = x + K * (z - H*x) */
    /* Simplified: update only position and size directly */
    float innovation[4];
    innovation[0] = measurement[0] - pxTrack->kalman_state[0];
    innovation[1] = measurement[1] - pxTrack->kalman_state[1];
    innovation[2] = measurement[2] - pxTrack->kalman_state[2];
    innovation[3] = measurement[3] - pxTrack->kalman_state[3];
    
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            pxTrack->kalman_state[i] += kalman_gain[i][j] * innovation[j];
        }
    }
    
    /* Update covariance */
    /* P = (I - K*H) * P */
    /* Simplified: reduce covariance based on Kalman gain */
    for (int i = 0; i < 4; i++)
    {
        pxTrack->kalman_covariance[i][i] *= (1 - kalman_gain[i][i]);
    }
    
    /* Update track position and velocity */
    pxTrack->x = pxTrack->kalman_state[0];
    pxTrack->y = pxTrack->kalman_state[1];
    pxTrack->width = pxTrack->kalman_state[2];
    pxTrack->height = pxTrack->kalman_state[3];
    pxTrack->vx = pxTrack->kalman_state[4];
    pxTrack->vy = pxTrack->kalman_state[5];
    
    /* Calculate velocity based on innovation */
    if (pxTrack->age > 1)
    {
        pxTrack->kalman_state[4] = innovation[0] * 0.3f + pxTrack->kalman_state[4] * 0.7f;  /* vx */
        pxTrack->kalman_state[5] = innovation[1] * 0.3f + pxTrack->kalman_state[5] * 0.7f;  /* vy */
        pxTrack->kalman_state[6] = innovation[2] * 0.3f + pxTrack->kalman_state[6] * 0.7f;  /* vw */
        pxTrack->kalman_state[7] = innovation[3] * 0.3f + pxTrack->kalman_state[7] * 0.7f;  /* vh */
    }
    
    /* Ensure width and height are positive */
    if (pxTrack->width < 0.01f) pxTrack->width = 0.01f;
    if (pxTrack->height < 0.01f) pxTrack->height = 0.01f;
    
    /* Ensure coordinates are within bounds */
    pxTrack->x = fmaxf(0, fminf(1, pxTrack->x));
    pxTrack->y = fmaxf(0, fminf(1, pxTrack->y));
    pxTrack->width = fmaxf(0.01f, fminf(1, pxTrack->width));
    pxTrack->height = fmaxf(0.01f, fminf(1, pxTrack->height));
}

/**
 * @brief Manage track states
 */
static void prvManageTracks(void)
{
    for (int i = 0; i < ucNumTracks; i++)
    {
        /* If track hasn't been updated for a while, mark as lost */
        if (xTracks[i].state == TRACK_TRACKED && xTracks[i].time_since_update > 1)
        {
            xTracks[i].state = TRACK_LOST;
        }
        
        /* If lost track hasn't been updated for too long, mark as removed */
        if (xTracks[i].state == TRACK_LOST && xTracks[i].time_since_update > MAX_TIME_LOST)
        {
            xTracks[i].state = TRACK_REMOVED;
        }
    }
    
    /* Compact track list by removing TRACK_REMOVED tracks */
    int iCompactIndex = 0;
    for (int i = 0; i < ucNumTracks; i++)
    {
        if (xTracks[i].state != TRACK_REMOVED)
        {
            if (i != iCompactIndex)
            {
                memcpy(&xTracks[iCompactIndex], &xTracks[i], sizeof(Track_t));
            }
            iCompactIndex++;
        }
    }
    
    ucNumTracks = iCompactIndex;
}
