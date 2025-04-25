#ifndef SECURITY_H
#define SECURITY_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* Security status codes */
#define SECURITY_STATUS_SUCCESS      0
#define SECURITY_STATUS_ERROR        1
#define SECURITY_STATUS_AUTH_FAILED  2
#define SECURITY_STATUS_EXPIRED      3
#define SECURITY_STATUS_INVALID      4

/* Encryption algorithms */
#define ENCRYPTION_NONE              0
#define ENCRYPTION_AES_128_GCM       1
#define ENCRYPTION_AES_256_GCM       2
#define ENCRYPTION_CHACHA20_POLY1305 3

/* Hash algorithms */
#define HASH_NONE                    0
#define HASH_SHA256                  1
#define HASH_SHA512                  2

/* Key types */
#define KEY_TYPE_SYMMETRIC           0
#define KEY_TYPE_ASYMMETRIC          1
#define KEY_TYPE_CERTIFICATE         2

/* Security function prototypes */
BaseType_t xSecurityInit(void);
BaseType_t xSecurityGenerateDeviceID(uint8_t *pucDeviceID, uint16_t usSize);
BaseType_t xSecurityGenerateAuthToken(uint8_t *pucAuthToken, uint16_t usSize);
BaseType_t xSecurityVerifyAuthToken(const uint8_t *pucAuthToken, uint16_t usSize);
BaseType_t xSecurityEncryptData(const uint8_t *pucInput, uint16_t usInputSize, 
                               uint8_t *pucOutput, uint16_t *pusOutputSize, 
                               uint8_t ucAlgorithm);
BaseType_t xSecurityDecryptData(const uint8_t *pucInput, uint16_t usInputSize, 
                               uint8_t *pucOutput, uint16_t *pusOutputSize, 
                               uint8_t ucAlgorithm);
BaseType_t xSecurityHashData(const uint8_t *pucInput, uint16_t usInputSize, 
                            uint8_t *pucOutput, uint16_t *pusOutputSize, 
                            uint8_t ucAlgorithm);
BaseType_t xSecuritySignData(const uint8_t *pucInput, uint16_t usInputSize, 
                            uint8_t *pucSignature, uint16_t *pusSignatureSize);
BaseType_t xSecurityVerifySignature(const uint8_t *pucInput, uint16_t usInputSize, 
                                   const uint8_t *pucSignature, uint16_t usSignatureSize);
BaseType_t xSecurityStoreKey(uint8_t ucKeyType, const uint8_t *pucKey, uint16_t usKeySize, 
                            const char *pcKeyName);
BaseType_t xSecurityLoadKey(uint8_t ucKeyType, uint8_t *pucKey, uint16_t *pusKeySize, 
                           const char *pcKeyName);
BaseType_t xSecurityVerifyFirmware(const uint8_t *pucFirmware, uint32_t ulFirmwareSize);
BaseType_t xSecuritySecureBoot(void);
BaseType_t xSecurityUpdateFirmware(const uint8_t *pucFirmware, uint32_t ulFirmwareSize);

#endif /* SECURITY_H */
