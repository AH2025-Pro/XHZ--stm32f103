#include "i2c_user.h"  // ����������� I2C ����ͷ�ļ�
#include <string.h>
#include <stdio.h>

// ��Щ��ȫ�ֱ������洢��Ҫд��EEPROM������
extern double W1, W2, B;
extern uint8_t BIG_WEIGHT_SID, BIG_WEIGHT_DID;

// EEPROM ������
uint8_t eepromBuffer[32];  // ���ڴ洢�ֽ����飨24�ֽ� double + 2�ֽ� uint8_t��

// EEPROM ����
uint16_t memAddr = 0x00;  // EEPROM �ڴ��ַ
uint16_t DevAddress = 0xA0;  // EEPROM �� I2C ��ַ
uint32_t Timeout = 100;  // ��ʱʱ��

//д�����ݵ� EEPROM
void WriteDataToEEPROM(double W1, double W2, double B, uint8_t BIG_WEIGHT_SID, uint8_t BIG_WEIGHT_DID) {
    // ����������ת��Ϊ�ֽ�����
    memcpy(&eepromBuffer[0], &W1, sizeof(W1));  // W1
    memcpy(&eepromBuffer[8], &W2, sizeof(W2));  // W2
    memcpy(&eepromBuffer[16], &B, sizeof(B));   // B
    memcpy(&eepromBuffer[24], &BIG_WEIGHT_SID, sizeof(BIG_WEIGHT_SID)); // ID1
    memcpy(&eepromBuffer[25], &BIG_WEIGHT_DID, sizeof(BIG_WEIGHT_DID)); // ID2

    // �ֶ�д��EEPROM
    HAL_StatusTypeDef status1 = HAL_I2C_Mem_Write(&hi2c1, DevAddress, memAddr, I2C_MEMADD_SIZE_8BIT, &eepromBuffer[0], sizeof(W1), Timeout);
    HAL_Delay(5);  // ��ʱ�ȴ�EEPROM���д��

    HAL_StatusTypeDef status2 = HAL_I2C_Mem_Write(&hi2c1, DevAddress, memAddr + sizeof(W1), I2C_MEMADD_SIZE_8BIT, &eepromBuffer[8], sizeof(W2), Timeout);
    HAL_Delay(5);  // ��ʱ�ȴ�EEPROM���д��

    HAL_StatusTypeDef status3 = HAL_I2C_Mem_Write(&hi2c1, DevAddress, memAddr + sizeof(W1) + sizeof(W2), I2C_MEMADD_SIZE_8BIT, &eepromBuffer[16], sizeof(B), Timeout);
    HAL_Delay(5);  // ��ʱ�ȴ�EEPROM���д��

    HAL_StatusTypeDef status4 = HAL_I2C_Mem_Write(&hi2c1, DevAddress, memAddr + sizeof(W1) + sizeof(W2) + sizeof(B), I2C_MEMADD_SIZE_8BIT, &eepromBuffer[24], sizeof(BIG_WEIGHT_SID), Timeout);
    HAL_Delay(5);  // ��ʱ�ȴ�EEPROM���д��

    HAL_StatusTypeDef status5 = HAL_I2C_Mem_Write(&hi2c1, DevAddress, memAddr + sizeof(W1) + sizeof(W2) + sizeof(B) + sizeof(BIG_WEIGHT_SID), I2C_MEMADD_SIZE_8BIT, &eepromBuffer[25], sizeof(BIG_WEIGHT_DID), Timeout);
    HAL_Delay(5);  // ��ʱ�ȴ�EEPROM���д��

    // ���д��״̬
    if (status1 == HAL_OK && status2 == HAL_OK && status3 == HAL_OK && status4 == HAL_OK && status5 == HAL_OK) {
        printf("EEPROM Write successful\n");
    } else {
        printf("Write failed\n");
    }
}

// �� EEPROM ��ȡ 24 �ֽڵ����ݲ��ָ��� 3 �� double ���͵���ֵ
void ReadDataFromEEPROM(double* W1, double* W2, double* B, uint8_t* BIG_WEIGHT_SID, uint8_t* BIG_WEIGHT_DID) {
    // ��EEPROM��ȡ����
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, DevAddress, memAddr, I2C_MEMADD_SIZE_8BIT, eepromBuffer, sizeof(eepromBuffer), Timeout);
    if (status == HAL_OK) {
        // ���ֽ�����ָ�Ϊԭʼ����
        memcpy(W1, &eepromBuffer[0], sizeof(*W1));  // W1
        memcpy(W2, &eepromBuffer[8], sizeof(*W2));  // W2
        memcpy(B, &eepromBuffer[16], sizeof(*B));   // B
        memcpy(BIG_WEIGHT_SID, &eepromBuffer[24], sizeof(*BIG_WEIGHT_SID)); // SID
        memcpy(BIG_WEIGHT_DID, &eepromBuffer[25], sizeof(*BIG_WEIGHT_DID)); // DID

        // ��ӡ��ȡ������
        printf("Success: W1 = %.9f, W2 = %.9f, B = %.9f, BIG_WEIGHT_SID = %u, BIG_WEIGHT_DID = %u\n", *W1, *W2, *B, *BIG_WEIGHT_SID, *BIG_WEIGHT_DID);
    } else {
        printf("Read failed\n");
    }
}

