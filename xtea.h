/*
 * xtea.h
 *
 *  Created on: Mar 16, 2015
 *      Author: walmis
 */

#ifndef XTEA_H_
#define XTEA_H_

#include <stdint.h>

typedef struct AVXTEA {
    uint32_t key[16];
} AVXTEA;

void xtea_crypt_ecb(AVXTEA *ctx, uint8_t *dst, const uint8_t *src,
                           int decrypt, uint8_t *iv);

#endif /* XTEA_H_ */
