#ifndef I2C_USER_H
#define I2C_USER_H

#include "i2c.h"
#include "stdio.h"
#include "string.h"
#include "stm32f1xx_hal.h"

void WriteDataToEEPROM(double W1, double W2, double B, uint8_t BIG_WEIGHT_ID1, uint8_t BIG_WEIGHT_ID2);
void ReadDataFromEEPROM(double* W1, double* W2, double* B, uint8_t* BIG_WEIGHT_ID1, uint8_t* BIG_WEIGHT_ID2);

#endif