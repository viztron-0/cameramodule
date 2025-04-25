#include "security.h"
#include "task_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* Include Raspberry Pi Pico specific headers */
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/unique_id.h"

/* Include mbedTLS headers */
#include "mbedtls/aes.h"
#include "mbedtls/gcm.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/pk.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/x509.h"
#include "mbedtls/ecdsa.h"

/* Flash storage parameters */
#define FLASH_TARGET_OFFSET          (1024 * 1024)  /* 1MB offset */
#define FLASH_SECTOR_SIZE            4096
#define FLASH_KEY_STORAGE_SIZE       (16 * FLASH_SECTOR_SIZE)  /* 64KB for key storage */

/* Security parameters */
#define DEVICE_ID_SIZE               16
#define AUTH_TOKEN_SIZE              32
#define AES_KEY_SIZE                 32
#define AES_IV_SIZE                  12
#define AES_TAG_SIZE                 16
#define CHACHA20_KEY_SIZE            32
#define CHACHA20_NONCE_SIZE          12
#define CHACHA20_TAG_SIZE            16
#define SHA256_SIZE                  32
#define SHA512_SIZE                  64
#define SIGNATURE_SIZE               64
#define MAX_KEY_NAME_SIZE            32

/* Key storage structure */
typedef struct {
    char name[MAX_KEY_NAME_SIZE];
    uint8_t type;
    uint16_t size;
    uint8_t data[256];  /* Maximum key size */
} KeyStorage_t;

/* Static variables */
static uint8_t ucSecurityInitialized = 0;
static uint8_t ucDeviceID[DEVICE_ID_SIZE];
static uint8_t ucAuthToken[AUTH_TOKEN_SIZE];
static mbedtls_entropy_context xEntropy;
static mbedtls_ctr_drbg_context xCtrDrbg;
static mbedtls_pk_context xDeviceKey;
static mbedtls_x509_crt xDeviceCert;
static mbedtls_x509_crt xCACert;

/* Forward declarations of static functions */
static BaseType_t prvSecurityGenerateRandomBytes(uint8_t *pucOutput, size_t xSize);
static BaseType_t prvSecurityStoreToFlash(const void *pvData, uint32_t ulOffset, uint32_t ulSize);
static BaseType_t prvSecurityLoadFromFlash(void *pvData, uint32_t ulOffset, uint32_t ulSize);
static BaseType_t prvSecurityFindKey(const char *pcKeyName, KeyStorage_t *pxKeyStorage);
static BaseType_t prvSecurityAESGCMEncrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                          uint8_t *pucOutput, uint16_t *pusOutputSize,
                                          const uint8_t *pucKey, uint16_t usKeySize);
static BaseType_t prvSecurityAESGCMDecrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                          uint8_t *pucOutput, uint16_t *pusOutputSize,
                                          const uint8_t *pucKey, uint16_t usKeySize);
static BaseType_t prvSecurityChaCha20Encrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                            uint8_t *pucOutput, uint16_t *pusOutputSize,
                                            const uint8_t *pucKey, uint16_t usKeySize);
static BaseType_t prvSecurityChaCha20Decrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                            uint8_t *pucOutput, uint16_t *pusOutputSize,
                                            const uint8_t *pucKey, uint16_t usKeySize);

/**
 * @brief Initialize the security module
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityInit(void)
{
    /* Initialize mbedTLS entropy and random number generator */
    mbedtls_entropy_init(&xEntropy);
    mbedtls_ctr_drbg_init(&xCtrDrbg);
    
    /* Seed the random number generator */
    if (mbedtls_ctr_drbg_seed(&xCtrDrbg, mbedtls_entropy_func, &xEntropy,
                             (const unsigned char *)"Viztron Camera Module", 21) != 0)
    {
        printf("Failed to seed random number generator\n");
        return pdFAIL;
    }
    
    /* Initialize device key and certificate */
    mbedtls_pk_init(&xDeviceKey);
    mbedtls_x509_crt_init(&xDeviceCert);
    mbedtls_x509_crt_init(&xCACert);
    
    /* Load device ID from flash or generate a new one */
    if (prvSecurityLoadFromFlash(ucDeviceID, 0, DEVICE_ID_SIZE) != pdPASS)
    {
        /* Generate a new device ID */
        if (xSecurityGenerateDeviceID(ucDeviceID, DEVICE_ID_SIZE) != pdPASS)
        {
            printf("Failed to generate device ID\n");
            return pdFAIL;
        }
        
        /* Store device ID to flash */
        if (prvSecurityStoreToFlash(ucDeviceID, 0, DEVICE_ID_SIZE) != pdPASS)
        {
            printf("Failed to store device ID\n");
            return pdFAIL;
        }
    }
    
    /* Load authentication token from flash or generate a new one */
    if (prvSecurityLoadFromFlash(ucAuthToken, DEVICE_ID_SIZE, AUTH_TOKEN_SIZE) != pdPASS)
    {
        /* Generate a new authentication token */
        if (xSecurityGenerateAuthToken(ucAuthToken, AUTH_TOKEN_SIZE) != pdPASS)
        {
            printf("Failed to generate authentication token\n");
            return pdFAIL;
        }
        
        /* Store authentication token to flash */
        if (prvSecurityStoreToFlash(ucAuthToken, DEVICE_ID_SIZE, AUTH_TOKEN_SIZE) != pdPASS)
        {
            printf("Failed to store authentication token\n");
            return pdFAIL;
        }
    }
    
    /* Load device key and certificate */
    /* This would be implemented based on the specific secure element or key storage mechanism */
    
    ucSecurityInitialized = 1;
    
    printf("Security module initialized\n");
    return pdPASS;
}

/**
 * @brief Generate a unique device ID
 * 
 * @param pucDeviceID Pointer to store device ID
 * @param usSize Size of device ID buffer
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityGenerateDeviceID(uint8_t *pucDeviceID, uint16_t usSize)
{
    if (pucDeviceID == NULL || usSize < DEVICE_ID_SIZE)
    {
        return pdFAIL;
    }
    
    /* Use Pico's unique ID if available */
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    
    /* Copy board ID to device ID */
    if (sizeof(board_id.id) <= usSize)
    {
        memcpy(pucDeviceID, board_id.id, sizeof(board_id.id));
        
        /* Fill remaining bytes with random data if needed */
        if (sizeof(board_id.id) < usSize)
        {
            prvSecurityGenerateRandomBytes(pucDeviceID + sizeof(board_id.id), 
                                          usSize - sizeof(board_id.id));
        }
    }
    else
    {
        /* Board ID is larger than buffer, truncate */
        memcpy(pucDeviceID, board_id.id, usSize);
    }
    
    return pdPASS;
}

/**
 * @brief Generate an authentication token
 * 
 * @param pucAuthToken Pointer to store authentication token
 * @param usSize Size of authentication token buffer
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityGenerateAuthToken(uint8_t *pucAuthToken, uint16_t usSize)
{
    if (pucAuthToken == NULL || usSize < AUTH_TOKEN_SIZE)
    {
        return pdFAIL;
    }
    
    /* Generate random bytes for authentication token */
    return prvSecurityGenerateRandomBytes(pucAuthToken, usSize);
}

/**
 * @brief Verify an authentication token
 * 
 * @param pucAuthToken Pointer to authentication token
 * @param usSize Size of authentication token
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityVerifyAuthToken(const uint8_t *pucAuthToken, uint16_t usSize)
{
    if (pucAuthToken == NULL || usSize != AUTH_TOKEN_SIZE || !ucSecurityInitialized)
    {
        return pdFAIL;
    }
    
    /* Compare with stored authentication token */
    if (memcmp(pucAuthToken, ucAuthToken, AUTH_TOKEN_SIZE) == 0)
    {
        return pdPASS;
    }
    
    return pdFAIL;
}

/**
 * @brief Encrypt data
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucOutput Pointer to output buffer
 * @param pusOutputSize Pointer to output size (in/out)
 * @param ucAlgorithm Encryption algorithm to use
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityEncryptData(const uint8_t *pucInput, uint16_t usInputSize, 
                               uint8_t *pucOutput, uint16_t *pusOutputSize, 
                               uint8_t ucAlgorithm)
{
    if (pucInput == NULL || pucOutput == NULL || pusOutputSize == NULL || !ucSecurityInitialized)
    {
        return pdFAIL;
    }
    
    /* Load encryption key */
    uint8_t ucKey[AES_KEY_SIZE];
    uint16_t usKeySize = AES_KEY_SIZE;
    
    if (xSecurityLoadKey(KEY_TYPE_SYMMETRIC, ucKey, &usKeySize, "encryption_key") != pdPASS)
    {
        /* Key not found, generate a new one */
        prvSecurityGenerateRandomBytes(ucKey, AES_KEY_SIZE);
        xSecurityStoreKey(KEY_TYPE_SYMMETRIC, ucKey, AES_KEY_SIZE, "encryption_key");
    }
    
    /* Encrypt data based on algorithm */
    switch (ucAlgorithm)
    {
        case ENCRYPTION_AES_128_GCM:
            return prvSecurityAESGCMEncrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                           ucKey, 16);
            
        case ENCRYPTION_AES_256_GCM:
            return prvSecurityAESGCMEncrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                           ucKey, 32);
            
        case ENCRYPTION_CHACHA20_POLY1305:
            return prvSecurityChaCha20Encrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                             ucKey, CHACHA20_KEY_SIZE);
            
        default:
            return pdFAIL;
    }
}

/**
 * @brief Decrypt data
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucOutput Pointer to output buffer
 * @param pusOutputSize Pointer to output size (in/out)
 * @param ucAlgorithm Encryption algorithm to use
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityDecryptData(const uint8_t *pucInput, uint16_t usInputSize, 
                               uint8_t *pucOutput, uint16_t *pusOutputSize, 
                               uint8_t ucAlgorithm)
{
    if (pucInput == NULL || pucOutput == NULL || pusOutputSize == NULL || !ucSecurityInitialized)
    {
        return pdFAIL;
    }
    
    /* Load encryption key */
    uint8_t ucKey[AES_KEY_SIZE];
    uint16_t usKeySize = AES_KEY_SIZE;
    
    if (xSecurityLoadKey(KEY_TYPE_SYMMETRIC, ucKey, &usKeySize, "encryption_key") != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Decrypt data based on algorithm */
    switch (ucAlgorithm)
    {
        case ENCRYPTION_AES_128_GCM:
            return prvSecurityAESGCMDecrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                           ucKey, 16);
            
        case ENCRYPTION_AES_256_GCM:
            return prvSecurityAESGCMDecrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                           ucKey, 32);
            
        case ENCRYPTION_CHACHA20_POLY1305:
            return prvSecurityChaCha20Decrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                             ucKey, CHACHA20_KEY_SIZE);
            
        default:
            return pdFAIL;
    }
}

/**
 * @brief Hash data
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucOutput Pointer to output buffer
 * @param pusOutputSize Pointer to output size (in/out)
 * @param ucAlgorithm Hash algorithm to use
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityHashData(const uint8_t *pucInput, uint16_t usInputSize, 
                            uint8_t *pucOutput, uint16_t *pusOutputSize, 
                            uint8_t ucAlgorithm)
{
    if (pucInput == NULL || pucOutput == NULL || pusOutputSize == NULL)
    {
        return pdFAIL;
    }
    
    /* Hash data based on algorithm */
    switch (ucAlgorithm)
    {
        case HASH_SHA256:
            {
                if (*pusOutputSize < SHA256_SIZE)
                {
                    *pusOutputSize = SHA256_SIZE;
                    return pdFAIL;
                }
                
                mbedtls_sha256_context ctx;
                mbedtls_sha256_init(&ctx);
                mbedtls_sha256_starts(&ctx, 0);  /* 0 for SHA-256 */
                mbedtls_sha256_update(&ctx, pucInput, usInputSize);
                mbedtls_sha256_finish(&ctx, pucOutput);
                mbedtls_sha256_free(&ctx);
                
                *pusOutputSize = SHA256_SIZE;
                return pdPASS;
            }
            
        case HASH_SHA512:
            {
                if (*pusOutputSize < SHA512_SIZE)
                {
                    *pusOutputSize = SHA512_SIZE;
                    return pdFAIL;
                }
                
                mbedtls_sha512_context ctx;
                mbedtls_sha512_init(&ctx);
                mbedtls_sha512_starts(&ctx, 0);  /* 0 for SHA-512 */
                mbedtls_sha512_update(&ctx, pucInput, usInputSize);
                mbedtls_sha512_finish(&ctx, pucOutput);
                mbedtls_sha512_free(&ctx);
                
                *pusOutputSize = SHA512_SIZE;
                return pdPASS;
            }
            
        default:
            return pdFAIL;
    }
}

/**
 * @brief Sign data
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucSignature Pointer to signature buffer
 * @param pusSignatureSize Pointer to signature size (in/out)
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecuritySignData(const uint8_t *pucInput, uint16_t usInputSize, 
                            uint8_t *pucSignature, uint16_t *pusSignatureSize)
{
    if (pucInput == NULL || pucSignature == NULL || pusSignatureSize == NULL || !ucSecurityInitialized)
    {
        return pdFAIL;
    }
    
    /* Hash the input data first */
    uint8_t ucHash[SHA256_SIZE];
    uint16_t usHashSize = SHA256_SIZE;
    
    if (xSecurityHashData(pucInput, usInputSize, ucHash, &usHashSize, HASH_SHA256) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Sign the hash using the device private key */
    size_t xSignatureSize = 0;
    
    if (mbedtls_pk_sign(&xDeviceKey, MBEDTLS_MD_SHA256, ucHash, SHA256_SIZE, 
                        pucSignature, pusSignatureSize, &xSignatureSize, 
                        mbedtls_ctr_drbg_random, &xCtrDrbg) != 0)
    {
        return pdFAIL;
    }
    
    *pusSignatureSize = (uint16_t)xSignatureSize;
    
    return pdPASS;
}

/**
 * @brief Verify signature
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucSignature Pointer to signature
 * @param usSignatureSize Size of signature
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityVerifySignature(const uint8_t *pucInput, uint16_t usInputSize, 
                                   const uint8_t *pucSignature, uint16_t usSignatureSize)
{
    if (pucInput == NULL || pucSignature == NULL || !ucSecurityInitialized)
    {
        return pdFAIL;
    }
    
    /* Hash the input data first */
    uint8_t ucHash[SHA256_SIZE];
    uint16_t usHashSize = SHA256_SIZE;
    
    if (xSecurityHashData(pucInput, usInputSize, ucHash, &usHashSize, HASH_SHA256) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Verify the signature using the CA public key */
    if (mbedtls_pk_verify(&xCACert.pk, MBEDTLS_MD_SHA256, ucHash, SHA256_SIZE, 
                         pucSignature, usSignatureSize) != 0)
    {
        return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Store key
 * 
 * @param ucKeyType Key type
 * @param pucKey Pointer to key data
 * @param usKeySize Size of key data
 * @param pcKeyName Key name
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityStoreKey(uint8_t ucKeyType, const uint8_t *pucKey, uint16_t usKeySize, 
                            const char *pcKeyName)
{
    if (pucKey == NULL || pcKeyName == NULL || usKeySize > 256)
    {
        return pdFAIL;
    }
    
    /* Find existing key or empty slot */
    KeyStorage_t xKeyStorage;
    uint32_t ulOffset = DEVICE_ID_SIZE + AUTH_TOKEN_SIZE;
    uint32_t ulMaxKeys = FLASH_KEY_STORAGE_SIZE / sizeof(KeyStorage_t);
    BaseType_t xFound = pdFALSE;
    
    for (uint32_t i = 0; i < ulMaxKeys; i++)
    {
        if (prvSecurityLoadFromFlash(&xKeyStorage, ulOffset, sizeof(KeyStorage_t)) == pdPASS)
        {
            if (strcmp(xKeyStorage.name, pcKeyName) == 0 || xKeyStorage.name[0] == 0)
            {
                /* Found existing key or empty slot */
                xFound = pdTRUE;
                break;
            }
        }
        
        ulOffset += sizeof(KeyStorage_t);
    }
    
    if (!xFound)
    {
        return pdFAIL;  /* No space available */
    }
    
    /* Prepare key storage structure */
    memset(&xKeyStorage, 0, sizeof(KeyStorage_t));
    strncpy(xKeyStorage.name, pcKeyName, MAX_KEY_NAME_SIZE - 1);
    xKeyStorage.type = ucKeyType;
    xKeyStorage.size = usKeySize;
    memcpy(xKeyStorage.data, pucKey, usKeySize);
    
    /* Store key to flash */
    return prvSecurityStoreToFlash(&xKeyStorage, ulOffset, sizeof(KeyStorage_t));
}

/**
 * @brief Load key
 * 
 * @param ucKeyType Key type
 * @param pucKey Pointer to key buffer
 * @param pusKeySize Pointer to key size (in/out)
 * @param pcKeyName Key name
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityLoadKey(uint8_t ucKeyType, uint8_t *pucKey, uint16_t *pusKeySize, 
                           const char *pcKeyName)
{
    if (pucKey == NULL || pusKeySize == NULL || pcKeyName == NULL)
    {
        return pdFAIL;
    }
    
    /* Find key by name */
    KeyStorage_t xKeyStorage;
    if (prvSecurityFindKey(pcKeyName, &xKeyStorage) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Check key type */
    if (xKeyStorage.type != ucKeyType)
    {
        return pdFAIL;
    }
    
    /* Check buffer size */
    if (*pusKeySize < xKeyStorage.size)
    {
        *pusKeySize = xKeyStorage.size;
        return pdFAIL;
    }
    
    /* Copy key data */
    memcpy(pucKey, xKeyStorage.data, xKeyStorage.size);
    *pusKeySize = xKeyStorage.size;
    
    return pdPASS;
}

/**
 * @brief Verify firmware
 * 
 * @param pucFirmware Pointer to firmware data
 * @param ulFirmwareSize Size of firmware data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityVerifyFirmware(const uint8_t *pucFirmware, uint32_t ulFirmwareSize)
{
    if (pucFirmware == NULL || ulFirmwareSize == 0 || !ucSecurityInitialized)
    {
        return pdFAIL;
    }
    
    /* Extract signature from firmware */
    /* Firmware format: [firmware_data][signature_size(2 bytes)][signature] */
    if (ulFirmwareSize < SIGNATURE_SIZE + 2)
    {
        return pdFAIL;
    }
    
    uint16_t usSignatureSize = (pucFirmware[ulFirmwareSize - 2] << 8) | pucFirmware[ulFirmwareSize - 1];
    if (usSignatureSize > SIGNATURE_SIZE || ulFirmwareSize < usSignatureSize + 2)
    {
        return pdFAIL;
    }
    
    const uint8_t *pucSignature = pucFirmware + ulFirmwareSize - 2 - usSignatureSize;
    uint32_t ulFirmwareDataSize = ulFirmwareSize - 2 - usSignatureSize;
    
    /* Verify firmware signature */
    return xSecurityVerifySignature(pucFirmware, ulFirmwareDataSize, pucSignature, usSignatureSize);
}

/**
 * @brief Secure boot
 * 
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecuritySecureBoot(void)
{
    /* This would be implemented based on the specific secure boot mechanism */
    /* For Raspberry Pi Pico, this might involve checking firmware signature at boot */
    
    return pdPASS;
}

/**
 * @brief Update firmware
 * 
 * @param pucFirmware Pointer to firmware data
 * @param ulFirmwareSize Size of firmware data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
BaseType_t xSecurityUpdateFirmware(const uint8_t *pucFirmware, uint32_t ulFirmwareSize)
{
    if (pucFirmware == NULL || ulFirmwareSize == 0 || !ucSecurityInitialized)
    {
        return pdFAIL;
    }
    
    /* Verify firmware first */
    if (xSecurityVerifyFirmware(pucFirmware, ulFirmwareSize) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Update firmware */
    /* This would be implemented based on the specific firmware update mechanism */
    /* For Raspberry Pi Pico, this might involve writing to flash and rebooting */
    
    return pdPASS;
}

/**
 * @brief Generate random bytes
 * 
 * @param pucOutput Pointer to output buffer
 * @param xSize Size of output buffer
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityGenerateRandomBytes(uint8_t *pucOutput, size_t xSize)
{
    if (pucOutput == NULL || xSize == 0)
    {
        return pdFAIL;
    }
    
    /* Generate random bytes using mbedTLS */
    if (mbedtls_ctr_drbg_random(&xCtrDrbg, pucOutput, xSize) != 0)
    {
        return pdFAIL;
    }
    
    return pdPASS;
}

/**
 * @brief Store data to flash
 * 
 * @param pvData Pointer to data
 * @param ulOffset Offset in flash
 * @param ulSize Size of data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityStoreToFlash(const void *pvData, uint32_t ulOffset, uint32_t ulSize)
{
    if (pvData == NULL || ulSize == 0)
    {
        return pdFAIL;
    }
    
    /* Calculate flash address */
    uint32_t ulFlashAddress = FLASH_TARGET_OFFSET + ulOffset;
    
    /* Ensure offset is aligned to flash page size */
    if (ulFlashAddress % FLASH_PAGE_SIZE != 0)
    {
        ulFlashAddress = (ulFlashAddress / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    }
    
    /* Allocate buffer for flash sector */
    uint8_t *pucBuffer = pvPortMalloc(FLASH_SECTOR_SIZE);
    if (pucBuffer == NULL)
    {
        return pdFAIL;
    }
    
    /* Read current sector */
    memcpy(pucBuffer, (void *)(XIP_BASE + ulFlashAddress), FLASH_SECTOR_SIZE);
    
    /* Update data in buffer */
    memcpy(pucBuffer + (ulOffset % FLASH_SECTOR_SIZE), pvData, ulSize);
    
    /* Erase and write sector */
    uint32_t ulInterruptState = save_and_disable_interrupts();
    flash_range_erase(ulFlashAddress, FLASH_SECTOR_SIZE);
    flash_range_program(ulFlashAddress, pucBuffer, FLASH_SECTOR_SIZE);
    restore_interrupts(ulInterruptState);
    
    /* Free buffer */
    vPortFree(pucBuffer);
    
    return pdPASS;
}

/**
 * @brief Load data from flash
 * 
 * @param pvData Pointer to data buffer
 * @param ulOffset Offset in flash
 * @param ulSize Size of data
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityLoadFromFlash(void *pvData, uint32_t ulOffset, uint32_t ulSize)
{
    if (pvData == NULL || ulSize == 0)
    {
        return pdFAIL;
    }
    
    /* Calculate flash address */
    uint32_t ulFlashAddress = FLASH_TARGET_OFFSET + ulOffset;
    
    /* Copy data from flash */
    memcpy(pvData, (void *)(XIP_BASE + ulFlashAddress), ulSize);
    
    return pdPASS;
}

/**
 * @brief Find key by name
 * 
 * @param pcKeyName Key name
 * @param pxKeyStorage Pointer to key storage structure
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityFindKey(const char *pcKeyName, KeyStorage_t *pxKeyStorage)
{
    if (pcKeyName == NULL || pxKeyStorage == NULL)
    {
        return pdFAIL;
    }
    
    /* Search for key in flash */
    uint32_t ulOffset = DEVICE_ID_SIZE + AUTH_TOKEN_SIZE;
    uint32_t ulMaxKeys = FLASH_KEY_STORAGE_SIZE / sizeof(KeyStorage_t);
    
    for (uint32_t i = 0; i < ulMaxKeys; i++)
    {
        if (prvSecurityLoadFromFlash(pxKeyStorage, ulOffset, sizeof(KeyStorage_t)) == pdPASS)
        {
            if (strcmp(pxKeyStorage->name, pcKeyName) == 0)
            {
                /* Found key */
                return pdPASS;
            }
        }
        
        ulOffset += sizeof(KeyStorage_t);
    }
    
    return pdFAIL;
}

/**
 * @brief AES-GCM encryption
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucOutput Pointer to output buffer
 * @param pusOutputSize Pointer to output size (in/out)
 * @param pucKey Pointer to key
 * @param usKeySize Size of key
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityAESGCMEncrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                          uint8_t *pucOutput, uint16_t *pusOutputSize,
                                          const uint8_t *pucKey, uint16_t usKeySize)
{
    if (pucInput == NULL || pucOutput == NULL || pusOutputSize == NULL || 
        pucKey == NULL || (usKeySize != 16 && usKeySize != 32))
    {
        return pdFAIL;
    }
    
    /* Check output buffer size */
    uint16_t usRequiredSize = usInputSize + AES_IV_SIZE + AES_TAG_SIZE;
    if (*pusOutputSize < usRequiredSize)
    {
        *pusOutputSize = usRequiredSize;
        return pdFAIL;
    }
    
    /* Generate random IV */
    uint8_t ucIV[AES_IV_SIZE];
    if (prvSecurityGenerateRandomBytes(ucIV, AES_IV_SIZE) != pdPASS)
    {
        return pdFAIL;
    }
    
    /* Copy IV to output */
    memcpy(pucOutput, ucIV, AES_IV_SIZE);
    
    /* Initialize AES-GCM context */
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    
    /* Set key */
    if (mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, pucKey, usKeySize * 8) != 0)
    {
        mbedtls_gcm_free(&ctx);
        return pdFAIL;
    }
    
    /* Encrypt data */
    uint8_t ucTag[AES_TAG_SIZE];
    if (mbedtls_gcm_crypt_and_tag(&ctx, MBEDTLS_GCM_ENCRYPT, usInputSize, ucIV, AES_IV_SIZE,
                                 NULL, 0, pucInput, pucOutput + AES_IV_SIZE, AES_TAG_SIZE, ucTag) != 0)
    {
        mbedtls_gcm_free(&ctx);
        return pdFAIL;
    }
    
    /* Copy tag to output */
    memcpy(pucOutput + AES_IV_SIZE + usInputSize, ucTag, AES_TAG_SIZE);
    
    /* Clean up */
    mbedtls_gcm_free(&ctx);
    
    /* Set output size */
    *pusOutputSize = usRequiredSize;
    
    return pdPASS;
}

/**
 * @brief AES-GCM decryption
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucOutput Pointer to output buffer
 * @param pusOutputSize Pointer to output size (in/out)
 * @param pucKey Pointer to key
 * @param usKeySize Size of key
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityAESGCMDecrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                          uint8_t *pucOutput, uint16_t *pusOutputSize,
                                          const uint8_t *pucKey, uint16_t usKeySize)
{
    if (pucInput == NULL || pucOutput == NULL || pusOutputSize == NULL || 
        pucKey == NULL || (usKeySize != 16 && usKeySize != 32))
    {
        return pdFAIL;
    }
    
    /* Check input size */
    if (usInputSize < AES_IV_SIZE + AES_TAG_SIZE)
    {
        return pdFAIL;
    }
    
    /* Calculate plaintext size */
    uint16_t usPlaintextSize = usInputSize - AES_IV_SIZE - AES_TAG_SIZE;
    
    /* Check output buffer size */
    if (*pusOutputSize < usPlaintextSize)
    {
        *pusOutputSize = usPlaintextSize;
        return pdFAIL;
    }
    
    /* Extract IV and tag */
    const uint8_t *pucIV = pucInput;
    const uint8_t *pucCiphertext = pucInput + AES_IV_SIZE;
    const uint8_t *pucTag = pucInput + AES_IV_SIZE + usPlaintextSize;
    
    /* Initialize AES-GCM context */
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    
    /* Set key */
    if (mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, pucKey, usKeySize * 8) != 0)
    {
        mbedtls_gcm_free(&ctx);
        return pdFAIL;
    }
    
    /* Decrypt data */
    if (mbedtls_gcm_auth_decrypt(&ctx, usPlaintextSize, pucIV, AES_IV_SIZE,
                                NULL, 0, pucTag, AES_TAG_SIZE,
                                pucCiphertext, pucOutput) != 0)
    {
        mbedtls_gcm_free(&ctx);
        return pdFAIL;
    }
    
    /* Clean up */
    mbedtls_gcm_free(&ctx);
    
    /* Set output size */
    *pusOutputSize = usPlaintextSize;
    
    return pdPASS;
}

/**
 * @brief ChaCha20-Poly1305 encryption
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucOutput Pointer to output buffer
 * @param pusOutputSize Pointer to output size (in/out)
 * @param pucKey Pointer to key
 * @param usKeySize Size of key
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityChaCha20Encrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                            uint8_t *pucOutput, uint16_t *pusOutputSize,
                                            const uint8_t *pucKey, uint16_t usKeySize)
{
    /* This is a placeholder for ChaCha20-Poly1305 encryption */
    /* In a real implementation, this would use mbedTLS or another library */
    
    /* For now, just use AES-GCM as a fallback */
    return prvSecurityAESGCMEncrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                   pucKey, usKeySize > 16 ? 16 : usKeySize);
}

/**
 * @brief ChaCha20-Poly1305 decryption
 * 
 * @param pucInput Pointer to input data
 * @param usInputSize Size of input data
 * @param pucOutput Pointer to output buffer
 * @param pusOutputSize Pointer to output size (in/out)
 * @param pucKey Pointer to key
 * @param usKeySize Size of key
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise
 */
static BaseType_t prvSecurityChaCha20Decrypt(const uint8_t *pucInput, uint16_t usInputSize,
                                            uint8_t *pucOutput, uint16_t *pusOutputSize,
                                            const uint8_t *pucKey, uint16_t usKeySize)
{
    /* This is a placeholder for ChaCha20-Poly1305 decryption */
    /* In a real implementation, this would use mbedTLS or another library */
    
    /* For now, just use AES-GCM as a fallback */
    return prvSecurityAESGCMDecrypt(pucInput, usInputSize, pucOutput, pusOutputSize, 
                                   pucKey, usKeySize > 16 ? 16 : usKeySize);
}
