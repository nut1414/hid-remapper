#ifndef _AES128_H_
#define _AES128_H_

#include <stdint.h>

// Encrypts one 16-byte block with AES-128 (ECB).
void aes128_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

#endif
