#include "communication.h"
#include "task_manager.h"
#include "camera.h"
#include "gps.h"
#include "power.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* Include Raspberry Pi Pico specific headers */
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/platform.h"

/* TLS configuration */
#define TLS_SERVER_CERT_BUFFER_SIZE 2048
#define TLS_CLIENT_CERT_BUFFER_SIZE 2048
#define TLS_CLIENT_KEY_BUFFER_SIZE  2048

/* Communication parameters */
#define HEARTBEAT_INTERVAL          30000  /* 30 seconds */
#define RECONNECT_INTERVAL          5000   /* 5 seconds */
#define COMMAND_QUEUE_SIZE          10
#define MAX_PACKET_SIZE             1500

/* H.264 encoding parameters */
#define H264_GOP_SIZE               30     /* Group of pictures size */
#define H264_BITRATE                500000 /* 500 kbps */
#define H264_FRAME_RATE             20     /* 20 fps */

/* Static variables */
static ConnectionStatus_t eConnectionStatus = CONNECTION_STATUS_DISCONNECTED;
static struct tcp_pcb *pxTcpPcb = NULL;
static uint16_t usNextMessageId = 0;
static uint8_t ucDeviceID[16] = {0};
static uint8_t ucAuthToken[32] = {0};
static uint8_t ucSessionToken[32] = {0};
static uint32_t ulLastHeartbeatTime = 0;
static uint32_t ulLastReconnectTime = 0;
static uint8_t ucTxBuffer[MAX_PACKET_SIZE];
static uint8_t ucRxBuffer[MAX_PACKET_SIZE];
static uint16_t usRxBufferPos = 0;
static QueueHandle_t xCommandQueue;
static uint8_t ucVideoStreamEnabled = 0;
static uint8_t ucVideoQuality = 80;
static uint8_t ucVideoFps = 20;

/* TLS context */
static mbedtls_ssl_context xSslContext;
static mbedtls_ssl_config xSslConfig;
static mbedtls_entropy_context xEntropy;
static mbedtls_ctr_drbg_context xCtrDrbg;
static mbedtls_x509_crt xCaCert;
static mbedtls_x509_crt xClientCert;
static mbedtls_pk_context xClientKey;

/* Server certificate (replace with actual certificate) */
static const char cServerCert[] = 
"-----BEGIN CERTIFICATE-----\n"
"MIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/\n"
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n"
"DkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow\n"
"PzEkMCIGA1UEChMbRGlnaXRhbCBTaWduYXR1cmUgVHJ1c3QgQ28uMRcwFQYDVQQD\n"
"Ew5EU1QgUm9vdCBDQSBYMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEB\n"
"AN+v6ZdQCINXtMxiZfaQguzH0yxrMMpb7NnDfcdAwRgUi+DoM3ZJKuM/IUmTrE4O\n"
"rz5Iy2Xu/NMhD2XSKtkyj4zl93ewEnu1lcCJo6m67XMuegwGMoOifooUMM0RoOEq\n"
"OLl5CjH9UL2AZd+3UWODyOKIYepLYYHsUmu5ouJLGiifSKOeDNoJjj4XLh7dIN9b\n"
"xiqKqy69cK3FCxolkHRyxXtqqzTWMIn/5WgTe1QLyNau7Fqckh49ZLOMxt+/yUFw\n"
"7BZy1SbsOFU5Q9D8/RhcQPGX69Wam40dutolucbY38EVAjqr2m7xPi71XAicPNaD\n"
"aeQQmxkqtilX4+U9m5/wAl0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\n"
"HQ8BAf8EBAMCAQYwHQYDVR0OBBYEFMSnsaR7LHH62+FLkHX/xBVghYkQMA0GCSqG\n"
"SIb3DQEBBQUAA4IBAQCjGiybFwBcqR7uKGY3Or+Dxz9LwwmglSBd49lZRNI+DT69\n"
"ikugdB/OEIKcdBodfpga3csTS7MgROSR6cz8faXbauX+5v3gTt23ADq1cEmv8uXr\n"
"AvHRAosZy5Q6XkjEGB5YGV8eAlrwDPGxrancWYaLbumR9YbK+rlmM6pZW87ipxZz\n"
"R8srzJmwN0jP41ZL9c8PDHIyh8bwRLtTcm1D9SZImlJnt1ir/md2cXjbDaJWFBM5\n"
"JDGFoqgCWjBH4d1QB7wCCZAA62RjYJsWvIjJEubSfZGL+T0yjWW06XyxV3bqxbYo\n"
"Ob8VZRzI9neWagqNdwvYkQsEjgfbKbYK7p2CNTUQ\n"
"-----END CERTIFICATE-----\n";

/* Client certificate (replace with actual certificate) */
static const char cClientCert[] = 
"-----BEGIN CERTIFICATE-----\n"
"MIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/\n"
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n"
"DkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow\n"
"PzEkMCIGA1UEChMbRGlnaXRhbCBTaWduYXR1cmUgVHJ1c3QgQ28uMRcwFQYDVQQD\n"
"Ew5EU1QgUm9vdCBDQSBYMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEB\n"
"AN+v6ZdQCINXtMxiZfaQguzH0yxrMMpb7NnDfcdAwRgUi+DoM3ZJKuM/IUmTrE4O\n"
"rz5Iy2Xu/NMhD2XSKtkyj4zl93ewEnu1lcCJo6m67XMuegwGMoOifooUMM0RoOEq\n"
"OLl5CjH9UL2AZd+3UWODyOKIYepLYYHsUmu5ouJLGiifSKOeDNoJjj4XLh7dIN9b\n"
"xiqKqy69cK3FCxolkHRyxXtqqzTWMIn/5WgTe1QLyNau7Fqckh49ZLOMxt+/yUFw\n"
"7BZy1SbsOFU5Q9D8/RhcQPGX69Wam40dutolucbY38EVAjqr2m7xPi71XAicPNaD\n"
"aeQQmxkqtilX4+U9m5/wAl0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\n"
"HQ8BAf8EBAMCAQYwHQYDVR0OBBYEFMSnsaR7LHH62+FLkHX/xBVghYkQMA0GCSqG\n"
"SIb3DQEBBQUAA4IBAQCjGiybFwBcqR7uKGY3Or+Dxz9LwwmglSBd49lZRNI+DT69\n"
"ikugdB/OEIKcdBodfpga3csTS7MgROSR6cz8faXbauX+5v3gTt23ADq1cEmv8uXr\n"
"AvHRAosZy5Q6XkjEGB5YGV8eAlrwDPGxrancWYaLbumR9YbK+rlmM6pZW87ipxZz\n"
"R8srzJmwN0jP41ZL9c8PDHIyh8bwRLtTcm1D9SZImlJnt1ir/md2cXjbDaJWFBM5\n"
"JDGFoqgCWjBH4d1QB7wCCZAA62RjYJsWvIjJEubSfZGL+T0yjWW06XyxV3bqxbYo\n"
"Ob8VZRzI9neWagqNdwvYkQsEjgfbKbYK7p2CNTUQ\n"
"-----END CERTIFICATE-----\n";

/* Client key (replace with actual key) */
static const char cClientKey[] = 
"-----BEGIN PRIVATE KEY-----\n"
"MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQDfr+mXUAiDV7TM\n"
"YmX2kILsx9MsazDKW+zZw33HQMEYFIvg6DN2SSrjPyFJk6xODq8+SMtl7vzTIQ9l\n"
"0irZMo+M5fd3sBJ7tZXAiaOpuu1zLnoMBjKDon6KFDDNEaDhKji5eQox/VC9gGXf\n"
"t1Fjg8jiiGHqS2GB7FJruaLiSxoon0ijngzaCY4+Fy4e3SDfW8YqiqsuvXCtxQsa\n"
"JZB0csV7aqs01jCJ/+VoE3tUC8jWruxanJIePWSzjMbfv8lBcOwWctUm7DhVOUPQ\n"
"/P0YXEDxl+vVmpuNHbraJbnG2N/BFQI6q9pu8T4u9VwInDzWg2nkEJsZKrYpV+Pl\n"
"PZuf8AJdAgMBAAECggEBAKVxP1m3FzHBUe2NZ3fYCc0Qa2zjK7xl1KPFp2xKYr9E\n"
"eFQRZ8uoEQtQWHhsmx3AhxNxTH/uM7Zgc3Hy5xV7u3QiTmZZgI8lYWVJn5CpN7ry\n"
"GlPcz2yWMu1xmQGfhUyLEgfqB/PNWFXIGRzsCmkedALkBW0LyYGd5Hv9/SyZQI7o\n"
"QJ0UfQ1eVCIJOTe3D3jRX+6jKLCIxnpJC6Xwt6MAapRSgbQcjIX/SQQgxkLUzQKK\n"
"U8ddOMwxFgFbEQURgKcRf1Z5bOLNY4GwmA9XcEzOlp5w5JYGVAiBw0D5UM4RvJBi\n"
"QEUNdJ0ItfiohjvPmgUrvCKK1+8vO4vRKBX9nJpYTUECgYEA+hESMhrAuGPtrZc8\n"
"fC1eEz0uFWiGbsS9QFJk2qEPxMRQxJw0wL7hWjUz9iQjwpBE4zE0YIrR8+Fn5kIW\n"
"1XxADECgUwlcnADXBYbPOCUUYhdBl3qMWnMPRvYqrPJzm8T8HOc1k4QVQmWnN+zw\n"
"9v9GmcUKy8wVb0LYO5KHhSB1YFECgYEA5QkUJM/QUmE8pcSLGBrJxEKlCEjQp9bx\n"
"g9jgAEBZP5bAl8SQky5SglyHWMcTsHFEMUL7P8UQsKX+8xCOu5iqCpRCRHjRhUXB\n"
"ZFgKRpIQV9YfPQ3GIs3OJRIbZ3+nrRlYTMXnUGpRn3fU+RbEWbIkqXeOo+K7RLCU\n"
"I7DvMYEIrY0CgYEAzwOOsz8U6BYgUZXY5Uf0dhXsjQrTpUOz3j5rpAYzNhBY3mHp\n"
"rNlzI7Ga6hgaYx3zOrGtTtBLgpzKFXiJJsWdCCvtTW1GzR0OOJr8wVudqGt8PeXY\n"
"tM9dbA9xKLJ3J7LboyFKXNHIKQ3JbKdTRWwWlVFvvm/uhOiPJYoqHSUEFRECgYAX\n"
"5+PRFQ0D1UJWZfj7Ic8+4nH0Ytzv9XeKNkuRkHcXbNJQUoM5RA8ZGhZRswmEPTjL\n"
"5VBgxDSIZLxUzKUYfyWa+L2vEZmMHJJ9bYpgUvMZS8FxnxuBG2FXBxuPob8MHJVS\n"
"W6KKZaQhXOCyCJBT6HGY9TjlX5bqJNmH1tQ3+/+UNQKBgGxU8hbOxk3vK8x3RQCQ\n"
"f8IHpZ5BvTUcLUHLYNnYR6Bf8knuaKvKYJHjx4tFYA5QKg3a3n4lR5Lq4wQ4jnG4\n"
"v/+ZOdJiTZ0PRWBf+RG1JxcJQUQxYrHbadLAKBTWj5FPpKL4qQ/WQfYFXkQq8yJK\n"
"0GvA0QVoYaS9nGcCKzqIVOxI\n"
"-----END PRIVATE KEY-----\n";

/* Forward declarations of static functions */
static void prvCommunicationInitTLS(void);
static err_t prvTcpConnectCallback(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t prvTcpRecvCallback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t prvTcpSentCallback(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void prvTcpErrorCallback(void *arg, err_t err);
static BaseType_t prvSendMessage(uint8_t ucMessageType, const void *pvData, uint16_t usDataLength);
static BaseType_t prvProcessMessage(const uint8_t *pucData, uint16_t usLength);
static BaseType_t prvSendAuthMessage(void);
static BaseType_t prvSendHeartbeat(void);
static BaseType_t prvSendDetectionMessage(DetectionResult_t *pxDetection);
static BaseType_t prvSendStatusMessage(void);
static BaseType_t prvSendVideoFrameMessage(CameraFrame_t *pxFrame);
static BaseType_t prvProcessAuthResponse(const uint8_t *pucData, uint16_t usLength);
static BaseType_t prvProcessHeartbeatResponse(const uint8_t *pucData, uint16_t usLength);
static BaseType_t prvProcessConfigMessage(const uint8_t *pucData, uint16_t usLength);
static BaseType_t prvProcessVideoRequestMessage(const uint8_t *pucData, uint16_t usLength);
static BaseType_t prvProcessCommandMessage(const uint8_t *pucData, uint16_t usLength);
static BaseType_t prvEncodeH264Frame(CameraFrame_t *pxFrame, uint8_t *pucOutput, uint32_t *pulOutputLength);

/**
 * @brief Initialize the communication module
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationInit(void)
{
    /* Initialize Wi-Fi */
    if (cyw43_arch_init() != 0)
    {
        printf("Failed to initialize Wi-Fi\n");
        return pdFAIL;
    }
    
    /* Initialize TLS */
    prvCommunicationInitTLS();
    
    /* Create command queue */
    xCommandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(CommandMessage_t));
    if (xCommandQueue == NULL)
    {
        printf("Failed to create command queue\n");
        return pdFAIL;
    }
    
    /* Initialize connection status */
    eConnectionStatus = CONNECTION_STATUS_DISCONNECTED;
    
    /* Initialize message ID */
    usNextMessageId = 0;
    
    printf("Communication module initialized\n");
    return pdPASS;
}

/**
 * @brief Connect to Homebase
 * 
 * @param pcHomebaseIP Homebase IP address
 * @param usPort Homebase port
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationConnect(const char *pcHomebaseIP, uint16_t usPort)
{
    ip_addr_t xHomebaseAddr;
    err_t err;
    
    /* Check if already connected */
    if (eConnectionStatus == CONNECTION_STATUS_CONNECTED)
    {
        return pdPASS;
    }
    
    /* Check if already connecting */
    if (eConnectionStatus == CONNECTION_STATUS_CONNECTING || 
        eConnectionStatus == CONNECTION_STATUS_AUTHENTICATING)
    {
        return pdFAIL;
    }
    
    /* Check if reconnect interval has passed */
    uint32_t ulCurrentTime = xTaskGetTickCount();
    if (ulCurrentTime - ulLastReconnectTime < RECONNECT_INTERVAL)
    {
        return pdFAIL;
    }
    
    /* Update reconnect time */
    ulLastReconnectTime = ulCurrentTime;
    
    /* Resolve hostname if needed */
    if (ipaddr_aton(pcHomebaseIP, &xHomebaseAddr) == 0)
    {
        /* IP address is not in numeric format, try to resolve hostname */
        err = dns_gethostbyname(pcHomebaseIP, &xHomebaseAddr, NULL, NULL);
        if (err != ERR_OK)
        {
            printf("Failed to resolve hostname: %s\n", pcHomebaseIP);
            eConnectionStatus = CONNECTION_STATUS_ERROR;
            return pdFAIL;
        }
    }
    
    /* Create new TCP PCB */
    pxTcpPcb = tcp_new();
    if (pxTcpPcb == NULL)
    {
        printf("Failed to create TCP PCB\n");
        eConnectionStatus = CONNECTION_STATUS_ERROR;
        return pdFAIL;
    }
    
    /* Set TCP callbacks */
    tcp_arg(pxTcpPcb, NULL);
    tcp_recv(pxTcpPcb, prvTcpRecvCallback);
    tcp_sent(pxTcpPcb, prvTcpSentCallback);
    tcp_err(pxTcpPcb, prvTcpErrorCallback);
    
    /* Connect to Homebase */
    eConnectionStatus = CONNECTION_STATUS_CONNECTING;
    err = tcp_connect(pxTcpPcb, &xHomebaseAddr, usPort, prvTcpConnectCallback);
    if (err != ERR_OK)
    {
        printf("Failed to connect to Homebase: %d\n", err);
        tcp_close(pxTcpPcb);
        pxTcpPcb = NULL;
        eConnectionStatus = CONNECTION_STATUS_ERROR;
        return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Disconnect from Homebase
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationDisconnect(void)
{
    /* Check if connected */
    if (eConnectionStatus != CONNECTION_STATUS_CONNECTED && 
        eConnectionStatus != CONNECTION_STATUS_CONNECTING && 
        eConnectionStatus != CONNECTION_STATUS_AUTHENTICATING)
    {
        return pdPASS;
    }
    
    /* Close TCP connection */
    if (pxTcpPcb != NULL)
    {
        tcp_close(pxTcpPcb);
        pxTcpPcb = NULL;
    }
    
    /* Update connection status */
    eConnectionStatus = CONNECTION_STATUS_DISCONNECTED;
    
    return pdPASS;
}

/**
 * @brief Get connection status
 * 
 * @return ConnectionStatus_t Current connection status
 */
ConnectionStatus_t xCommunicationGetStatus(void)
{
    return eConnectionStatus;
}

/**
 * @brief Send detection result to Homebase
 * 
 * @param pxDetection Pointer to detection result
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationSendDetection(DetectionResult_t *pxDetection)
{
    /* Check if connected */
    if (eConnectionStatus != CONNECTION_STATUS_CONNECTED || pxDetection == NULL)
    {
        return pdFAIL;
    }
    
    /* Send detection message */
    return prvSendDetectionMessage(pxDetection);
}

/**
 * @brief Send status to Homebase
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationSendStatus(void)
{
    /* Check if connected */
    if (eConnectionStatus != CONNECTION_STATUS_CONNECTED)
    {
        return pdFAIL;
    }
    
    /* Send status message */
    return prvSendStatusMessage();
}

/**
 * @brief Send video frame to Homebase
 * 
 * @param pxFrame Pointer to camera frame
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationSendVideoFrame(CameraFrame_t *pxFrame)
{
    /* Check if connected and video streaming is enabled */
    if (eConnectionStatus != CONNECTION_STATUS_CONNECTED || 
        !ucVideoStreamEnabled || pxFrame == NULL)
    {
        return pdFAIL;
    }
    
    /* Send video frame message */
    return prvSendVideoFrameMessage(pxFrame);
}

/**
 * @brief Process command from Homebase
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationProcessCommand(void)
{
    CommandMessage_t xCommand;
    
    /* Check if there are commands in the queue */
    if (xQueueReceive(xCommandQueue, &xCommand, 0) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Process command */
    /* This would be expanded based on the specific commands */
    printf("Processing command type: %d\n", xCommand.header.message_type);
    
    return pdPASS;
}

/**
 * @brief Set device ID
 * 
 * @param pucDeviceID Device ID (16 bytes)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationSetDeviceID(const uint8_t *pucDeviceID)
{
    if (pucDeviceID == NULL)
    {
        return pdFAIL;
    }
    
    /* Copy device ID */
    memcpy(ucDeviceID, pucDeviceID, 16);
    
    return pdPASS;
}

/**
 * @brief Set authentication token
 * 
 * @param pucAuthToken Authentication token (32 bytes)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xCommunicationSetAuthToken(const uint8_t *pucAuthToken)
{
    if (pucAuthToken == NULL)
    {
        return pdFAIL;
    }
    
    /* Copy authentication token */
    memcpy(ucAuthToken, pucAuthToken, 32);
    
    return pdPASS;
}

/**
 * @brief Initialize TLS
 */
static void prvCommunicationInitTLS(void)
{
    /* Initialize mbedTLS structures */
    mbedtls_ssl_init(&xSslContext);
    mbedtls_ssl_config_init(&xSslConfig);
    mbedtls_entropy_init(&xEntropy);
    mbedtls_ctr_drbg_init(&xCtrDrbg);
    mbedtls_x509_crt_init(&xCaCert);
    mbedtls_x509_crt_init(&xClientCert);
    mbedtls_pk_init(&xClientKey);
    
    /* Seed the random number generator */
    mbedtls_ctr_drbg_seed(&xCtrDrbg, mbedtls_entropy_func, &xEntropy,
                          (const unsigned char *)"Viztron Camera Module", 21);
    
    /* Load certificates */
    mbedtls_x509_crt_parse(&xCaCert, (const unsigned char *)cServerCert, strlen(cServerCert) + 1);
    mbedtls_x509_crt_parse(&xClientCert, (const unsigned char *)cClientCert, strlen(cClientCert) + 1);
    mbedtls_pk_parse_key(&xClientKey, (const unsigned char *)cClientKey, strlen(cClientKey) + 1, NULL, 0);
    
    /* Set up SSL configuration */
    mbedtls_ssl_config_defaults(&xSslConfig, MBEDTLS_SSL_IS_CLIENT,
                               MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
    
    mbedtls_ssl_conf_authmode(&xSslConfig, MBEDTLS_SSL_VERIFY_REQUIRED);
    mbedtls_ssl_conf_ca_chain(&xSslConfig, &xCaCert, NULL);
    mbedtls_ssl_conf_rng(&xSslConfig, mbedtls_ctr_drbg_random, &xCtrDrbg);
    
    /* Set up client certificate */
    mbedtls_ssl_conf_own_cert(&xSslConfig, &xClientCert, &xClientKey);
    
    /* Set up SSL context */
    mbedtls_ssl_setup(&xSslContext, &xSslConfig);
}

/**
 * @brief TCP connect callback
 * 
 * @param arg User argument
 * @param tpcb TCP PCB
 * @param err Error code
 * @return err_t ERR_OK if successful, error code otherwise
 */
static err_t prvTcpConnectCallback(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    if (err != ERR_OK)
    {
        printf("TCP connect failed: %d\n", err);
        eConnectionStatus = CONNECTION_STATUS_ERROR;
        return err;
    }
    
    printf("TCP connected\n");
    
    /* Set up TLS connection */
    mbedtls_ssl_set_bio(&xSslContext, tpcb, 
                        (mbedtls_ssl_send_t *)tcp_write, 
                        (mbedtls_ssl_recv_t *)tcp_recv, 
                        NULL);
    
    /* Perform TLS handshake */
    int ret = mbedtls_ssl_handshake(&xSslContext);
    if (ret != 0)
    {
        printf("TLS handshake failed: %d\n", ret);
        eConnectionStatus = CONNECTION_STATUS_ERROR;
        return ERR_CONN;
    }
    
    printf("TLS handshake successful\n");
    
    /* Send authentication message */
    eConnectionStatus = CONNECTION_STATUS_AUTHENTICATING;
    prvSendAuthMessage();
    
    return ERR_OK;
}

/**
 * @brief TCP receive callback
 * 
 * @param arg User argument
 * @param tpcb TCP PCB
 * @param p Packet buffer
 * @param err Error code
 * @return err_t ERR_OK if successful, error code otherwise
 */
static err_t prvTcpRecvCallback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (err != ERR_OK)
    {
        printf("TCP receive error: %d\n", err);
        return err;
    }
    
    if (p == NULL)
    {
        /* Connection closed */
        printf("TCP connection closed by remote host\n");
        eConnectionStatus = CONNECTION_STATUS_DISCONNECTED;
        tcp_close(pxTcpPcb);
        pxTcpPcb = NULL;
        return ERR_OK;
    }
    
    /* Copy data to receive buffer */
    if (p->tot_len > 0)
    {
        /* Check if buffer has enough space */
        if (usRxBufferPos + p->tot_len > MAX_PACKET_SIZE)
        {
            /* Buffer overflow, reset buffer */
            usRxBufferPos = 0;
        }
        
        /* Copy data to buffer */
        pbuf_copy_partial(p, ucRxBuffer + usRxBufferPos, p->tot_len, 0);
        usRxBufferPos += p->tot_len;
        
        /* Process received data */
        prvProcessMessage(ucRxBuffer, usRxBufferPos);
        
        /* Reset buffer position */
        usRxBufferPos = 0;
    }
    
    /* Free packet buffer */
    pbuf_free(p);
    
    return ERR_OK;
}

/**
 * @brief TCP sent callback
 * 
 * @param arg User argument
 * @param tpcb TCP PCB
 * @param len Length of data sent
 * @return err_t ERR_OK if successful, error code otherwise
 */
static err_t prvTcpSentCallback(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    /* Data sent successfully */
    return ERR_OK;
}

/**
 * @brief TCP error callback
 * 
 * @param arg User argument
 * @param err Error code
 */
static void prvTcpErrorCallback(void *arg, err_t err)
{
    printf("TCP error: %d\n", err);
    
    /* Update connection status */
    eConnectionStatus = CONNECTION_STATUS_ERROR;
    
    /* Clear TCP PCB */
    pxTcpPcb = NULL;
}

/**
 * @brief Send message to Homebase
 * 
 * @param ucMessageType Message type
 * @param pvData Pointer to message data
 * @param usDataLength Length of message data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSendMessage(uint8_t ucMessageType, const void *pvData, uint16_t usDataLength)
{
    MessageHeader_t *pxHeader;
    err_t err;
    
    /* Check if connected */
    if (pxTcpPcb == NULL || (eConnectionStatus != CONNECTION_STATUS_CONNECTED && 
                            eConnectionStatus != CONNECTION_STATUS_AUTHENTICATING))
    {
        return pdFAIL;
    }
    
    /* Check if data fits in buffer */
    if (usDataLength > MAX_PACKET_SIZE)
    {
        return pdFAIL;
    }
    
    /* Copy data to transmit buffer */
    memcpy(ucTxBuffer, pvData, usDataLength);
    
    /* Update header */
    pxHeader = (MessageHeader_t *)ucTxBuffer;
    pxHeader->protocol_version = PROTOCOL_VERSION;
    pxHeader->message_type = ucMessageType;
    pxHeader->message_id = usNextMessageId++;
    pxHeader->payload_length = usDataLength - sizeof(MessageHeader_t);
    pxHeader->timestamp = xTaskGetTickCount();
    
    /* Send data over TLS */
    int ret = mbedtls_ssl_write(&xSslContext, ucTxBuffer, usDataLength);
    if (ret < 0)
    {
        printf("TLS write failed: %d\n", ret);
        return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Process received message
 * 
 * @param pucData Pointer to message data
 * @param usLength Length of message data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvProcessMessage(const uint8_t *pucData, uint16_t usLength)
{
    MessageHeader_t *pxHeader;
    
    /* Check if data is valid */
    if (pucData == NULL || usLength < sizeof(MessageHeader_t))
    {
        return pdFAIL;
    }
    
    /* Get message header */
    pxHeader = (MessageHeader_t *)pucData;
    
    /* Check protocol version */
    if (pxHeader->protocol_version != PROTOCOL_VERSION)
    {
        printf("Invalid protocol version: %d\n", pxHeader->protocol_version);
        return pdFAIL;
    }
    
    /* Check payload length */
    if (pxHeader->payload_length != usLength - sizeof(MessageHeader_t))
    {
        printf("Invalid payload length: %d\n", pxHeader->payload_length);
        return pdFAIL;
    }
    
    /* Process message based on type */
    switch (pxHeader->message_type)
    {
        case MSG_TYPE_AUTH_RESPONSE:
            return prvProcessAuthResponse(pucData, usLength);
            
        case MSG_TYPE_HEARTBEAT_RESPONSE:
            return prvProcessHeartbeatResponse(pucData, usLength);
            
        case MSG_TYPE_CONFIG:
            return prvProcessConfigMessage(pucData, usLength);
            
        case MSG_TYPE_VIDEO_REQUEST:
            return prvProcessVideoRequestMessage(pucData, usLength);
            
        case MSG_TYPE_COMMAND:
            return prvProcessCommandMessage(pucData, usLength);
            
        case MSG_TYPE_ERROR:
            printf("Received error message\n");
            return pdFAIL;
            
        default:
            printf("Unknown message type: %d\n", pxHeader->message_type);
            return pdFAIL;
    }
}

/**
 * @brief Send authentication message
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSendAuthMessage(void)
{
    AuthMessage_t xAuthMessage;
    
    /* Initialize message */
    memset(&xAuthMessage, 0, sizeof(AuthMessage_t));
    
    /* Set device ID and authentication token */
    memcpy(xAuthMessage.device_id, ucDeviceID, 16);
    memcpy(xAuthMessage.auth_token, ucAuthToken, 32);
    
    /* Send message */
    return prvSendMessage(MSG_TYPE_AUTH, &xAuthMessage, sizeof(AuthMessage_t));
}

/**
 * @brief Send heartbeat message
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSendHeartbeat(void)
{
    HeartbeatMessage_t xHeartbeatMessage;
    BatteryStatus_t xBatteryStatus;
    
    /* Initialize message */
    memset(&xHeartbeatMessage, 0, sizeof(HeartbeatMessage_t));
    
    /* Get battery status */
    xPowerGetBatteryStatus(&xBatteryStatus);
    
    /* Set uptime and battery level */
    xHeartbeatMessage.uptime = xTaskGetTickCount() / 1000;  /* Convert to seconds */
    xHeartbeatMessage.battery_level = xBatteryStatus.percentage;
    
    /* Send message */
    return prvSendMessage(MSG_TYPE_HEARTBEAT, &xHeartbeatMessage, sizeof(HeartbeatMessage_t));
}

/**
 * @brief Send detection message
 * 
 * @param pxDetection Pointer to detection result
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSendDetectionMessage(DetectionResult_t *pxDetection)
{
    uint8_t ucBuffer[MAX_PACKET_SIZE];
    DetectionMessage_t *pxMessage = (DetectionMessage_t *)ucBuffer;
    uint8_t *pucData = pxMessage->data;
    uint16_t usDataLength = 0;
    
    /* Initialize message */
    memset(pxMessage, 0, sizeof(DetectionMessage_t));
    
    /* Set frame ID and number of objects */
    pxMessage->frame_id = pxDetection->frame_id;
    pxMessage->num_objects = pxDetection->num_objects;
    
    /* Add detection objects */
    for (int i = 0; i < pxDetection->num_objects; i++)
    {
        /* Check if buffer has enough space */
        if (usDataLength + sizeof(DetectionObject_t) > MAX_PACKET_SIZE - sizeof(DetectionMessage_t))
        {
            break;
        }
        
        /* Copy object data */
        memcpy(pucData + usDataLength, &pxDetection->objects[i], sizeof(DetectionObject_t));
        usDataLength += sizeof(DetectionObject_t);
    }
    
    /* Send message */
    return prvSendMessage(MSG_TYPE_DETECTION, pxMessage, sizeof(DetectionMessage_t) + usDataLength);
}

/**
 * @brief Send status message
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSendStatusMessage(void)
{
    StatusMessage_t xStatusMessage;
    BatteryStatus_t xBatteryStatus;
    GPSPosition_t xGPSPosition;
    
    /* Initialize message */
    memset(&xStatusMessage, 0, sizeof(StatusMessage_t));
    
    /* Get battery status */
    xPowerGetBatteryStatus(&xBatteryStatus);
    
    /* Get GPS position */
    xGPSGetPosition(&xGPSPosition);
    
    /* Set status fields */
    xStatusMessage.system_state = (uint8_t)xPowerGetState();
    xStatusMessage.battery_level = xBatteryStatus.percentage;
    xStatusMessage.battery_voltage = xBatteryStatus.voltage;
    xStatusMessage.remaining_time = xBatteryStatus.remaining_time;
    xStatusMessage.latitude = xGPSPosition.latitude;
    xStatusMessage.longitude = xGPSPosition.longitude;
    xStatusMessage.gps_fix = xGPSPosition.fix_type;
    xStatusMessage.satellites = xGPSPosition.satellites;
    xStatusMessage.temperature = 25.0f;  /* Placeholder, would be read from sensor */
    xStatusMessage.signal_strength = 80;  /* Placeholder, would be read from Wi-Fi */
    
    /* Send message */
    return prvSendMessage(MSG_TYPE_STATUS, &xStatusMessage, sizeof(StatusMessage_t));
}

/**
 * @brief Send video frame message
 * 
 * @param pxFrame Pointer to camera frame
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSendVideoFrameMessage(CameraFrame_t *pxFrame)
{
    uint8_t ucBuffer[MAX_PACKET_SIZE];
    VideoFrameMessage_t *pxMessage = (VideoFrameMessage_t *)ucBuffer;
    uint32_t ulEncodedLength = 0;
    
    /* Initialize message */
    memset(pxMessage, 0, sizeof(VideoFrameMessage_t));
    
    /* Set frame information */
    pxMessage->frame_id = pxFrame->timestamp;
    pxMessage->width = pxFrame->width;
    pxMessage->height = pxFrame->height;
    pxMessage->format = 1;  /* H.264 encoded */
    
    /* Encode frame to H.264 */
    if (!prvEncodeH264Frame(pxFrame, pxMessage->data, &ulEncodedLength))
    {
        return pdFAIL;
    }
    
    /* Send message */
    return prvSendMessage(MSG_TYPE_VIDEO_FRAME, pxMessage, sizeof(VideoFrameMessage_t) + ulEncodedLength);
}

/**
 * @brief Process authentication response message
 * 
 * @param pucData Pointer to message data
 * @param usLength Length of message data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvProcessAuthResponse(const uint8_t *pucData, uint16_t usLength)
{
    AuthResponseMessage_t *pxResponse;
    
    /* Check if data is valid */
    if (pucData == NULL || usLength < sizeof(AuthResponseMessage_t))
    {
        return pdFAIL;
    }
    
    /* Get response */
    pxResponse = (AuthResponseMessage_t *)pucData;
    
    /* Check status */
    if (pxResponse->status != 0)
    {
        printf("Authentication failed: %d\n", pxResponse->status);
        eConnectionStatus = CONNECTION_STATUS_ERROR;
        return pdFAIL;
    }
    
    /* Store session token */
    memcpy(ucSessionToken, pxResponse->session_token, 32);
    
    /* Update connection status */
    eConnectionStatus = CONNECTION_STATUS_CONNECTED;
    
    /* Send initial status */
    prvSendStatusMessage();
    
    /* Update heartbeat time */
    ulLastHeartbeatTime = xTaskGetTickCount();
    
    printf("Authentication successful\n");
    
    return pdPASS;
}

/**
 * @brief Process heartbeat response message
 * 
 * @param pucData Pointer to message data
 * @param usLength Length of message data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvProcessHeartbeatResponse(const uint8_t *pucData, uint16_t usLength)
{
    HeartbeatResponseMessage_t *pxResponse;
    
    /* Check if data is valid */
    if (pucData == NULL || usLength < sizeof(HeartbeatResponseMessage_t))
    {
        return pdFAIL;
    }
    
    /* Get response */
    pxResponse = (HeartbeatResponseMessage_t *)pucData;
    
    /* Update server time if needed */
    /* This could be used for time synchronization */
    
    return pdPASS;
}

/**
 * @brief Process configuration message
 * 
 * @param pucData Pointer to message data
 * @param usLength Length of message data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvProcessConfigMessage(const uint8_t *pucData, uint16_t usLength)
{
    ConfigMessage_t *pxConfig;
    ConfigResponseMessage_t xResponse;
    
    /* Check if data is valid */
    if (pucData == NULL || usLength < sizeof(ConfigMessage_t))
    {
        return pdFAIL;
    }
    
    /* Get configuration */
    pxConfig = (ConfigMessage_t *)pucData;
    
    /* Initialize response */
    memset(&xResponse, 0, sizeof(ConfigResponseMessage_t));
    xResponse.config_type = pxConfig->config_type;
    xResponse.status = 0;  /* Success by default */
    
    /* Process configuration based on type */
    switch (pxConfig->config_type)
    {
        case 1:  /* Camera configuration */
            /* Process camera configuration */
            break;
            
        case 2:  /* AI configuration */
            /* Process AI configuration */
            break;
            
        case 3:  /* Power configuration */
            /* Process power configuration */
            break;
            
        case 4:  /* GPS configuration */
            /* Process GPS configuration */
            break;
            
        default:
            printf("Unknown configuration type: %d\n", pxConfig->config_type);
            xResponse.status = 1;  /* Error */
            break;
    }
    
    /* Send response */
    prvSendMessage(MSG_TYPE_CONFIG_RESPONSE, &xResponse, sizeof(ConfigResponseMessage_t));
    
    return pdPASS;
}

/**
 * @brief Process video request message
 * 
 * @param pucData Pointer to message data
 * @param usLength Length of message data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvProcessVideoRequestMessage(const uint8_t *pucData, uint16_t usLength)
{
    VideoRequestMessage_t *pxRequest;
    
    /* Check if data is valid */
    if (pucData == NULL || usLength < sizeof(VideoRequestMessage_t))
    {
        return pdFAIL;
    }
    
    /* Get request */
    pxRequest = (VideoRequestMessage_t *)pucData;
    
    /* Process request */
    switch (pxRequest->request_type)
    {
        case 0:  /* Stop streaming */
            ucVideoStreamEnabled = 0;
            break;
            
        case 1:  /* Start streaming */
            ucVideoStreamEnabled = 1;
            ucVideoQuality = pxRequest->quality;
            ucVideoFps = pxRequest->fps;
            break;
            
        case 2:  /* Single frame */
            /* Send a single frame */
            /* This would be handled by the camera task */
            break;
            
        default:
            printf("Unknown video request type: %d\n", pxRequest->request_type);
            return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Process command message
 * 
 * @param pucData Pointer to message data
 * @param usLength Length of message data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvProcessCommandMessage(const uint8_t *pucData, uint16_t usLength)
{
    CommandMessage_t *pxCommand;
    CommandResponseMessage_t xResponse;
    
    /* Check if data is valid */
    if (pucData == NULL || usLength < sizeof(CommandMessage_t))
    {
        return pdFAIL;
    }
    
    /* Get command */
    pxCommand = (CommandMessage_t *)pucData;
    
    /* Add command to queue for processing by main task */
    if (xQueueSend(xCommandQueue, pxCommand, 0) != pdPASS)
    {
        printf("Command queue full\n");
        
        /* Send error response */
        memset(&xResponse, 0, sizeof(CommandResponseMessage_t));
        xResponse.command_type = pxCommand->command_type;
        xResponse.status = 2;  /* Queue full */
        
        prvSendMessage(MSG_TYPE_COMMAND_RESPONSE, &xResponse, sizeof(CommandResponseMessage_t));
        
        return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Encode frame to H.264
 * 
 * @param pxFrame Pointer to camera frame
 * @param pucOutput Pointer to output buffer
 * @param pulOutputLength Pointer to output length
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvEncodeH264Frame(CameraFrame_t *pxFrame, uint8_t *pucOutput, uint32_t *pulOutputLength)
{
    /* This is a placeholder for actual H.264 encoding */
    /* In a real implementation, this would use a hardware encoder or software library */
    
    /* For now, just copy the raw frame data */
    if (pxFrame->size > MAX_PACKET_SIZE - sizeof(VideoFrameMessage_t))
    {
        *pulOutputLength = 0;
        return pdFAIL;
    }
    
    memcpy(pucOutput, pxFrame->data, pxFrame->size);
    *pulOutputLength = pxFrame->size;
    
    return pdPASS;
}
