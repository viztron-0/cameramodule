#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>
#include "ai.h"

/* Communication protocol versions */
#define PROTOCOL_VERSION            1

/* Message types */
#define MSG_TYPE_AUTH               0x01
#define MSG_TYPE_AUTH_RESPONSE      0x02
#define MSG_TYPE_HEARTBEAT          0x03
#define MSG_TYPE_HEARTBEAT_RESPONSE 0x04
#define MSG_TYPE_DETECTION          0x05
#define MSG_TYPE_CONFIG             0x06
#define MSG_TYPE_CONFIG_RESPONSE    0x07
#define MSG_TYPE_VIDEO_FRAME        0x08
#define MSG_TYPE_VIDEO_REQUEST      0x09
#define MSG_TYPE_STATUS             0x0A
#define MSG_TYPE_COMMAND            0x0B
#define MSG_TYPE_COMMAND_RESPONSE   0x0C
#define MSG_TYPE_ERROR              0xFF

/* Message header structure */
typedef struct __attribute__((packed)) {
    uint8_t protocol_version;       /* Protocol version */
    uint8_t message_type;           /* Message type */
    uint16_t message_id;            /* Message ID */
    uint16_t payload_length;        /* Payload length */
    uint32_t timestamp;             /* Timestamp */
    uint8_t flags;                  /* Message flags */
} MessageHeader_t;

/* Authentication message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t device_id[16];          /* Device ID (UUID) */
    uint8_t auth_token[32];         /* Authentication token */
} AuthMessage_t;

/* Authentication response message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t status;                 /* 0 = success, non-zero = error */
    uint8_t session_token[32];      /* Session token */
} AuthResponseMessage_t;

/* Heartbeat message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint32_t uptime;                /* Device uptime in seconds */
    uint8_t battery_level;          /* Battery level (0-100%) */
} HeartbeatMessage_t;

/* Heartbeat response message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint32_t server_time;           /* Server time */
} HeartbeatResponseMessage_t;

/* Detection message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint32_t frame_id;              /* Frame ID */
    uint8_t num_objects;            /* Number of detected objects */
    uint8_t data[];                 /* Variable-length detection data */
} DetectionMessage_t;

/* Configuration message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t config_type;            /* Configuration type */
    uint8_t data[];                 /* Variable-length configuration data */
} ConfigMessage_t;

/* Configuration response message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t config_type;            /* Configuration type */
    uint8_t status;                 /* 0 = success, non-zero = error */
} ConfigResponseMessage_t;

/* Video frame message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint32_t frame_id;              /* Frame ID */
    uint16_t width;                 /* Frame width */
    uint16_t height;                /* Frame height */
    uint8_t format;                 /* Frame format */
    uint8_t data[];                 /* Variable-length frame data */
} VideoFrameMessage_t;

/* Video request message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t request_type;           /* 0 = stop, 1 = start, 2 = single frame */
    uint8_t quality;                /* Video quality (0-100) */
    uint8_t fps;                    /* Requested FPS */
} VideoRequestMessage_t;

/* Status message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t system_state;           /* System state */
    uint8_t battery_level;          /* Battery level (0-100%) */
    float battery_voltage;          /* Battery voltage */
    uint32_t remaining_time;        /* Estimated remaining time in seconds */
    float latitude;                 /* GPS latitude */
    float longitude;                /* GPS longitude */
    uint8_t gps_fix;                /* GPS fix type */
    uint8_t satellites;             /* Number of satellites */
    float temperature;              /* Temperature in Celsius */
    uint8_t signal_strength;        /* Wi-Fi signal strength (0-100%) */
} StatusMessage_t;

/* Command message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t command_type;           /* Command type */
    uint8_t data[];                 /* Variable-length command data */
} CommandMessage_t;

/* Command response message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t command_type;           /* Command type */
    uint8_t status;                 /* 0 = success, non-zero = error */
    uint8_t data[];                 /* Variable-length response data */
} CommandResponseMessage_t;

/* Error message structure */
typedef struct __attribute__((packed)) {
    MessageHeader_t header;
    uint8_t error_code;             /* Error code */
    uint8_t error_message[64];      /* Error message */
} ErrorMessage_t;

/* Connection status */
typedef enum {
    CONNECTION_STATUS_DISCONNECTED,
    CONNECTION_STATUS_CONNECTING,
    CONNECTION_STATUS_AUTHENTICATING,
    CONNECTION_STATUS_CONNECTED,
    CONNECTION_STATUS_ERROR
} ConnectionStatus_t;

/* Communication function prototypes */
BaseType_t xCommunicationInit(void);
BaseType_t xCommunicationConnect(const char *pcHomebaseIP, uint16_t usPort);
BaseType_t xCommunicationDisconnect(void);
ConnectionStatus_t xCommunicationGetStatus(void);
BaseType_t xCommunicationSendDetection(DetectionResult_t *pxDetection);
BaseType_t xCommunicationSendStatus(void);
BaseType_t xCommunicationSendVideoFrame(CameraFrame_t *pxFrame);
BaseType_t xCommunicationProcessCommand(void);
BaseType_t xCommunicationSetDeviceID(const uint8_t *pucDeviceID);
BaseType_t xCommunicationSetAuthToken(const uint8_t *pucAuthToken);

#endif /* COMMUNICATION_H */
