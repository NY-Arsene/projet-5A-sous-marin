/** @file crc16.h  CRC-16/CCITT-FALSE (polynôme 0x1021, valeur initiale 0xFFFF) : détecte les octets altérés sur la liaison série. */
#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stddef.h>

/** CRC d'un bloc. Vecteur de contrôle : crc16_ccitt("123456789", 9) == 0x29B1. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#endif
