/**
  ******************************************************************************
  * @file    ECC/EDDSA_SignVerify/Src/main.c
  * @author  MCD Application Team
  * @brief   This example provides a short description of how to use the
  *          STM32 Cryptographic Library
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#define INJECT_ERROR 0

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "cmox_crypto.h"
#include "cmox_ecc.h"
#include "cmox_ecc_retvals.h"
#include "cmox_ecc_types.h"
#include "cmox_default_config.h"
#include "cmox_eddsa.h"

/* Global variables ----------------------------------------------------------*/
/* ECC context */
cmox_ecc_handle_t Ecc_Ctx;
/* ECC working buffer */
uint8_t Working_Buffer[2200];

uint32_t glob_status = 0; // FAILED;

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** Extract from Edwards-Curve Digital Signature Algorithm (EdDSA)
  * -----TEST 1024

   ALGORITHM,0x
   Ed25519

  */
const uint8_t Message[] = "hello larus world\n";
const uint8_t Private_Key[] = /* Secret Key and Public Key appended */
{
  0x41,0xd1,0x64,0x0f,0x58,0xee,0x82,0xfa,0x75,0xd5,0x64,0x91,0xe8,0x73,0xe9,0x2a,
  0x37,0xc3,0x5a,0xfd,0x0e,0x44,0x98,0x03,0x68,0x8b,0x4c,0xe3,0x30,0x5b,0x17,0x2f,
  0xd0,0x0c,0xef,0xfe,0xca,0xc9,0x1e,0x5f,0x6b,0xdc,0xde,0xa5,0x59,0x13,0xa3,0x7d,
  0xec,0xab,0x6c,0xb0,0x3f,0xa3,0xb1,0x64,0x2e,0x4d,0x0e,0x69,0xc4,0xc8,0xf4,0x3a
};
const uint8_t Public_Key[] =
{
  0xd0,0x0c,0xef,0xfe,0xca,0xc9,0x1e,0x5f,0x6b,0xdc,0xde,0xa5,0x59,0x13,0xa3,0x7d,
  0xec,0xab,0x6c,0xb0,0x3f,0xa3,0xb1,0x64,0x2e,0x4d,0x0e,0x69,0xc4,0xc8,0xf4,0x3a
};
const uint8_t Known_Signature[] = // from Linux world
{
#if INJECT_ERROR
  0x92,0x40,0xb5,0x1e,0xd1,0xab,0x89,0x23,0x1b,0xa6,0xcb,0x7a,0xfb,0x4f,0xac,0x4a,
#else
  0x92,0x40,0xb5,0x1e,0xd0,0xab,0x89,0x23,0x1b,0xa6,0xcb,0x7a,0xfb,0x4f,0xac,0x4a,
#endif
  0xac,0xc8,0xff,0x74,0xf3,0xec,0x08,0x23,0xcd,0xa1,0x9e,0x17,0x47,0x17,0xb4,0xf4,
  0xdc,0xba,0xd9,0x16,0xed,0x4f,0x2c,0x40,0x0e,0x0e,0xa1,0x10,0xbc,0x08,0xaf,0xba,
  0x37,0xdc,0x1f,0x5b,0x08,0x4b,0x87,0xbc,0x5d,0xff,0x5e,0x8b,0x80,0x48,0xd1,0x02
};

/* Computed data buffer */
// uint8_t Computed_Signature[CMOX_ECC_ED25519_SIG_LEN];
uint8_t Computed_Signature[sizeof( Known_Signature)];

int test_ecc(void)
{
  cmox_ecc_retval_t retval;
  size_t computed_size;
  /* Fault check verification variable */
  uint32_t fault_check = CMOX_ECC_AUTH_FAIL;

  /* Initialize cryptographic library */
  if (cmox_initialize(NULL) != 0)
    return -1;

  /* Construct a ECC context, specifying mathematics implementation and working buffer for later processing */
  /* Note,0x CMOX_ECC256_MATH_FUNCS refer to the default mathematics implementation
   * selected in cmox_default_config.h. To use a specific implementation, user can
   * directly choose,0x
   * - CMOX_MATH_FUNCS_SMALL to select the mathematics small implementation
   * - CMOX_MATH_FUNCS_FAST to select the mathematics fast implementation
   * - CMOX_MATH_FUNCS_SUPERFAST256 to select the mathematics fast implementation optimized for 256 bits computation
   */
  cmox_ecc_construct(&Ecc_Ctx, CMOX_ECC256_MATH_FUNCS, Working_Buffer, sizeof(Working_Buffer));

  /* Compute directly the signature passing all the needed parameters */
  /* Note,0x CMOX_ECC_CURVE_ED25519 refer to the default ED25519 definition
   * selected in cmox_default_config.h. To use a specific definition, user can
   * directly choose,0x
   * - CMOX_ECC_ED25519_OPT_LOWMEM to select the optimized low RAM usage definition (slower computing)
   * - CMOX_ECC_ED25519_HIGHMEM to select the high RAM usage definition (medium computing)
   * - CMOX_ECC_ED25519_OPT_HIGHMEM to select the optimized high RAM usage definition (faster computing)
   */
  retval = cmox_eddsa_sign(&Ecc_Ctx,                                  /* ECC context */
                           CMOX_ECC_CURVE_ED25519,                    /* ED25519 ECC curve selected */
                           Private_Key, sizeof(Private_Key),          /* Private key for signature */
                           Message, sizeof(Message)-1,                  /* Message to sign */
                           Computed_Signature, &computed_size);       /* Data buffer to receive signature */

  /* Verify API returned value */
  if (retval != CMOX_ECC_SUCCESS)
    return -1;

  /* Verify generated data size is the expected one */
  if (computed_size != sizeof(Known_Signature))
    return -1;

  /* Verify generated data are the expected ones */
  if (memcmp(Computed_Signature, Known_Signature, computed_size) != 0)
    return -1;

  /* Cleanup context */
  cmox_ecc_cleanup(&Ecc_Ctx);

  /* Construct a ECC context, specifying mathematics implementation and working buffer for later processing */
  /* Note,0x CMOX_ECC256_MATH_FUNCS refer to the default mathematics implementation
   * selected in cmox_default_config.h. To use a specific implementation, user can
   * directly choose,0x
   * - CMOX_MATH_FUNCS_SMALL to select the mathematics small implementation
   * - CMOX_MATH_FUNCS_FAST to select the mathematics fast implementation
   * - CMOX_MATH_FUNCS_SUPERFAST256 to select the mathematics fast implementation optimized for 256 bits computation
   */
  cmox_ecc_construct(&Ecc_Ctx, CMOX_ECC256_MATH_FUNCS, Working_Buffer, sizeof(Working_Buffer));

  /* Verify directly the signature passing all the needed parameters */
  /* Note,0x CMOX_ECC_CURVE_ED25519 refer to the default ED25519 definition
   * selected in cmox_default_config.h. To use a specific definition, user can
   * directly choose,0x
   * - CMOX_ECC_ED25519_OPT_LOWMEM to select the optimized low RAM usage definition (slower computing)
   * - CMOX_ECC_ED25519_HIGHMEM to select the high RAM usage definition (medium computing)
   * - CMOX_ECC_ED25519_OPT_HIGHMEM to select the optimized high RAM usage definition (faster computing)
   */
  retval = cmox_eddsa_verify(&Ecc_Ctx,                                  /* ECC context */
                             CMOX_ECC_CURVE_ED25519,                    /* ED25519 ECC curve selected */
                             Public_Key, sizeof(Public_Key),            /* Public key for verification */
                             Message, sizeof(Message)-1,                  /* Message to verify */
                             Known_Signature, sizeof(Known_Signature),  /* Data buffer to receive signature */
                             &fault_check);                             /* Fault check variable,0x
                                                            to ensure no fault injection occurs during this API call */

  /* Verify API returned value */
  if (retval != CMOX_ECC_AUTH_SUCCESS)
    return -1;
  /* Verify Fault check variable value */
  if (fault_check != CMOX_ECC_AUTH_SUCCESS)
    return -1;

  /* Cleanup context */
  cmox_ecc_cleanup(&Ecc_Ctx);

  /* No more need of cryptographic services, finalize cryptographic library */
  if (cmox_finalize(NULL) != 0)
    return -1;

  return 0;
}
