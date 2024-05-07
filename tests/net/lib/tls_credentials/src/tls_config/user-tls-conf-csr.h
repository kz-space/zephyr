/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* MbedTLS custom configuration file for use with prj_trusted_tfm_csr.conf */

#define MBEDTLS_X509_CSR_PARSE_C

//#define MBEDTLS_USE_PSA_CRYPTO
//#define MBEDTLS_PSA_CRYPTO_C

//#define MBEDTLS_ENTROPY_C
//#define MBEDTLS_TEST_NULL_ENTROPY

//#define MBEDTLS_ECP_C
//#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
//#define MBEDTLS_ECDSA_C

//#define MBEDTLS_X509_CSR_WRITE_C
//#define MBEDTLS_X509_CREATE_C

//#define MBEDTLS_OID_C
//#define MBEDTLS_ASN1_WRITE_C
//#define MBEDTLS_PK_WRITE_C
//#define MBEDTLS_PK_C