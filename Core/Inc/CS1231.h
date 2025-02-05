#ifndef DATA_H
#define DATA_H

#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "spi.h"
#include <stdlib.h> // 包含 abs 函数的声明
#include <math.h>

double calculate_weight(double ad_value1, double ad_value2);
uint32_t SPI_Receive24BitsAndConvertToVoltage(SPI_HandleTypeDef *hspi,GPIO_TypeDef * AD1231_DRDY_PORT,uint16_t AD1231_DRDY_PIN);
double kalman_update(double Z1, double Z2, double *P, double *W_est);
double calculate_weight2(double ad_value1, double ad_value2);

#endif