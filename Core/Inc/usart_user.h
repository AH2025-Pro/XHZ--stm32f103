#ifndef USART_USER_H
#define USART_USER_H

#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "spi.h"
#include "string.h"
#include "i2c_user.h"

void usart_nvit_init();
void process_command(uint8_t *command1, uint8_t *command2);
uint16_t crc16_modbus(uint8_t *data, uint16_t length);
void send_data_based_on_flags(uint32_t weighing_data);

#endif