#include "usart_user.h"

// 全局变量定义
double W1 = 0;  // 传感器权重1
double W2 = 0;  // 传感器权重2
double B = 0;   // 偏置值
uint8_t BIG_WEIGHT_SID = 0;  // 设备SID（动态区分左右）
uint8_t BIG_WEIGHT_DID = 0;  // 设备SID（动态区分左右）

int SP485 = 0;               // 通用SP485标志
int SP485_modbus1 = 0;       // Modbus1标志
int SP485_modbus2 = 0;       // Modbus2标志

int SP485_Receive = 0;               // 通用SP485标志
int SP485_modbus_Receive = 0;       // Modbus1标志


uint8_t rx_data_usart1[100];  // USART1接收数据
uint8_t rx_data_usart2[100];  // USART2接收数据

//uint8_t rx1_buffer[100] = {0};  // USART1接收缓冲区
//uint8_t rx2_buffer[100] = {0};  // USART2接收缓冲区
//uint16_t rx1_index = 0;         // USART1缓冲区索引
//uint16_t rx2_index = 0;         // USART2缓冲区索引

int debug_mode = 0;             // 调试模式标志
int big_weight_flag = 0;        // 大桶称重标志

// 数据处理标志位
volatile uint8_t data_ready_usart1 = 0;  // USART1数据准备好标志
volatile uint8_t data_ready_usart2 = 0;  // USART2数据准备好标志

uint16_t crc16_modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;  // Modbus CRC-16 polynomial
            } else {
                crc >>= 1;
            }
        }
    }
    // 交换高低字节，以符合 Modbus 协议的传输顺序
    return (crc >> 8) | (crc << 8);
}

// 串口接收初始化
void usart_nvit_init() {
    // 初始化USART1和USART2的中断接收
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_data_usart1, sizeof(rx_data_usart1));
//	HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_data_usart2, sizeof(rx_data_usart2));
}

// 串口接收中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {  // 检查是否为USART1
		// 重新启动接收，以便继续接收新的数据
        HAL_UARTEx_ReceiveToIdle_IT(huart, rx_data_usart1, sizeof(rx_data_usart1));
    } else if (huart->Instance == USART2) {  // 检查是否为USART2
		// 重新启动接收，以便继续接收新的数据
		if(Size == 12){SP485_Receive = 1;}
		if(Size == 8){SP485_modbus_Receive = 1;}
        HAL_UARTEx_ReceiveToIdle_IT(huart, rx_data_usart2, sizeof(rx_data_usart2));
    }
}

// 命令处理函数
void process_command(uint8_t *command1, uint8_t *command2) {
    /*********** USART1 数据处理 *******************/
    if (command1 != NULL) {
        // 判断命令是否是 "ENTER_DEBUG_MODE"
        if (strncmp((char *)command1, "ENTER_DEBUG_MODE", strlen("ENTER_DEBUG_MODE")) == 0) {
            debug_mode = 1;
            printf("进入调试模式\n");	
			memset(command1, 0, strlen((char *)command1)); // 清空 command1 缓冲区			
        }
        // 在调试模式下设置大桶称重参数
        else if (debug_mode && strncmp((char *)command1, "SET_PARAM_big_weight", strlen("SET_PARAM_big_weight")) == 0) {
            double w1_new, w2_new, b_new;
            if (sscanf((char *)(command1 + strlen("SET_PARAM_big_weight")), "%lf %lf %lf", &w1_new, &w2_new, &b_new) == 3) {
                W1 = w1_new;
                W2 = w2_new;
                B = b_new;
                printf("大桶称重参数已更新\r\n");
                debug_mode = 0;
                printf("退出调试模式\n");
                big_weight_flag = 1;
				memset(command1, 0, strlen((char *)command1)); // 清空 command1 缓冲区
            } else {
                char msg[] = "命令格式错误";
                HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
				memset(command1, 0, strlen((char *)command1)); // 清空 command1 缓冲区
            }
        }
		// 在调试模式下设置大桶ID参数
		else if (debug_mode && strncmp((char *)command1, "SET_ID_big_weight", strlen("SET_ID_big_weight")) == 0) {
			uint8_t SID_new, DID_new;
			if (sscanf((char *)(command1 + strlen("SET_ID_big_weight")), "%hhu %hhu", &SID_new, &DID_new) == 2) {
				BIG_WEIGHT_SID = SID_new;
				BIG_WEIGHT_DID = DID_new;
				printf("大桶ID参数已更新为：SID=%d, DID=%d\r\n", BIG_WEIGHT_SID, BIG_WEIGHT_DID);
				debug_mode = 0;
				printf("退出调试模式\n");
				big_weight_flag = 1;
				memset(command1, 0, strlen((char *)command1)); // 清空 command1 缓冲区
			} else {
				printf("命令格式错误");
				memset(command1, 0, strlen((char *)command1)); // 清空 command1 缓冲区
			}
		}
        // 在调试模式下退出调试模式
        else if (debug_mode && strncmp((char *)command1, "EXIT_DEBUG_MODE", strlen("EXIT_DEBUG_MODE")) == 0) {
            debug_mode = 0;
            printf("退出调试模式\n");
			memset(command1, 0, strlen((char *)command1)); // 清空 command1 缓冲区
        }
    }

    /*********** USART2 数据处理 *******************/
    if (SP485_modbus_Receive== 1 || SP485_Receive == 1 ) {
        // 检查是否符合 XHZ_GSINF=xx 格式
		if (SP485_Receive == 1){ 		
			if (strncmp((char *)command2, "XHZ_GSINF=", 10) == 0) {
            uint8_t cmd_sid = (command2[10] - '0') * 10 + (command2[11] - '0');
			printf("cmd_sid = %d,BIG_WEIGHT_SID = %d\r\n",cmd_sid,BIG_WEIGHT_SID);
            if (cmd_sid > 0 && cmd_sid == BIG_WEIGHT_SID) {
                SP485 = 1;
                printf("命令解析成功，设备地址匹配\r\n");
            }
			else {
				printf("命令解析成功，设备地址匹配失败\r\n");
			}
			memset(command2, 0, strlen((char *)command2)); // 清空 command1 缓冲区
        }
			SP485_Receive = 0;
		}
        // 检查是否为Modbus帧
        else if (SP485_modbus_Receive== 1) {
			printf("command2 = %x\r\n",command2[6]);
			printf("addr = %d\r\n",BIG_WEIGHT_SID);
            // 提取设备地址和功能码
            uint8_t addr = command2[0];
            uint8_t func_code = command2[1];

            // 提取CRC校验码
            uint16_t crc_received = (command2[6] << 8) | command2[7];
			printf("crc_received = %x\r\n",crc_received);
            // 计算CRC校验
            uint16_t crc_calculated = crc16_modbus(command2, 6);
			printf("crc_calculated = %x\r\n",crc_calculated);
            // 校验CRC
            if (crc_received == crc_calculated) {
                printf("Modbus数据帧校验通过！\n");
                printf("设备地址: 0x%X\n", addr);
                printf("功能码: 0x%X\n", func_code);

                // 进一步处理Modbus帧
                uint16_t start_address = (command2[2] << 8) | command2[3];
                uint16_t reg_count = (command2[4] << 8) | command2[5];
                printf("起始地址: 0x%X\n", start_address);
                printf("寄存器数量: 0x%X\n", reg_count);

                // 根据设备地址和起始地址设置标志变量
                if (addr == BIG_WEIGHT_SID && start_address == 0x0000) {
                    SP485_modbus1 = 1;
                    printf("设置 SP485_modbus1 = 1\n");
                } else if (addr == BIG_WEIGHT_SID && start_address == 0x0002) {
                    SP485_modbus2 = 1;
                    printf("设置 SP485_modbus2 = 1\n");
                }
            } else {
                printf("Modbus数据帧校验失败！\n");
            }
			SP485_modbus_Receive = 0;
			memset(command2, 0, strlen((char *)command2)); // 清空 command1 缓冲区
			}
    }
}

// 发送数据函数
void send_data_based_on_flags(uint32_t weighing_data) {
    uint8_t data_buffer[256] = {0};  // 数据发送缓冲区
    uint16_t data_length = 0;       // 数据长度

    // 根据SP485标志状态选择发送的数据格式
    if (SP485) {
        // SP485标志为1，发送普通数据格式
        char message[256];
        char weighing_data_str[9];
        sprintf(weighing_data_str, "%08X", weighing_data);  // 以8位十六进制格式化称重数据
        snprintf(message, sizeof(message), "XHZ_AGSINF=%02X,%02X,01,01,WSED,%s\r\n", 
                 BIG_WEIGHT_SID, BIG_WEIGHT_DID, weighing_data_str);  // 动态生成数据
        data_length = strlen(message);
        memcpy(data_buffer, message, data_length);
    } else if (SP485_modbus1) {
        // SP485_modbus1标志为1，发送Modbus1格式数据
        data_buffer[0] = BIG_WEIGHT_SID;  // 设备地址
        data_buffer[1] = 0x03;           // 功能码
        data_buffer[2] = 0x00;           // 数据长度高字节
        data_buffer[3] = 0x04;           // 数据长度低字节

        // 数据内容：4字节重量值（假设为小端格式）
        data_buffer[4] = weighing_data & 0xFF;
        data_buffer[5] = (weighing_data >> 8) & 0xFF;
        data_buffer[6] = (weighing_data >> 16) & 0xFF;
        data_buffer[7] = (weighing_data >> 24) & 0xFF;

        // 计算CRC校验
        uint16_t crc = crc16_modbus(data_buffer, 8);
        data_buffer[8] = (crc >> 8) & 0xFF;     // CRC低字节
        data_buffer[9] = crc & 0xFF;  // CRC高字节

        data_length = 10;
    } else if (SP485_modbus2) {
        // SP485_modbus2标志为1，发送Modbus2格式数据
        data_buffer[0] = BIG_WEIGHT_SID;  // 设备地址
        data_buffer[1] = 0x03;           // 功能码
        data_buffer[2] = 0x00;           // 数据长度高字节
        data_buffer[3] = 0x04;           // 数据长度低字节

        // 数据内容：千克（2字节）+ 克（2字节）
        uint16_t kilograms = weighing_data / 1000;  // 千克部分
        uint16_t grams = weighing_data % 1000;      // 克部分

        data_buffer[4] = kilograms & 0xFF;         // 千克低字节
        data_buffer[5] = (kilograms >> 8) & 0xFF;  // 千克高字节
        data_buffer[6] = grams & 0xFF;             // 克低字节
        data_buffer[7] = (grams >> 8) & 0xFF;      // 克高字节

        // 计算CRC校验
        uint16_t crc = crc16_modbus(data_buffer, 8);
        data_buffer[8] = (crc >> 8) & 0xFF;     // CRC低字节
        data_buffer[9] = crc & 0xFF;  // CRC高字节

        data_length = 10;
    }

    // 通过USART2发送数据
    if (data_length > 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart2, data_buffer, data_length, 1000);  // 发送数据，超时时间为1000ms
		//清零
		SP485 = 0;               // 通用SP485标志
		SP485_modbus1 = 0;       // Modbus1标志
		SP485_modbus2 = 0;       // Modbus2标志
    }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}
