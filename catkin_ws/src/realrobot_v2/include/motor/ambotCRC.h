#ifndef __AMBOT_CRC_H__
#define __AMBOT_CRC_H__

#include <stdio.h>
#include <iostream>

uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen );
void generateCRC(uint8_t *buffer, uint8_t length);

#endif