#include "CS1231.h"

#define FILTER_SIZE 10  // 滑动平均滤波器的窗口大小
// 指数平滑滤波参数
double alpha = 0.1;  // 平滑系数，取值范围为 [0, 1]，值越小表示平滑效果越强
double weight_buffer[FILTER_SIZE] = {0};  // 用于存储最近的重量值
int buffer_index = 0;  // 当前缓冲区索引
extern double W1 ,W2,B;

uint32_t SPI_Receive24BitsAndConvertToVoltage(SPI_HandleTypeDef *hspi, GPIO_TypeDef *AD1231_DRDY_PORT, uint16_t AD1231_DRDY_PIN) 
{
    uint8_t txBuffer[3] = {0x00, 0x00, 0x00}; // 发送缓冲区
    uint8_t rxBuffer[3]; // 接收缓冲区
    uint32_t rawData = 0; // 存储原始的24位数据
    int32_t signedData = 0;   // 存储有符号32位整数
    int result = 0; // 初始化结构体变量
    static int32_t lastValidData = 0; // 静态变量存储上一次的有效值

    // 等待 DRDY/DOUT 引脚被拉低，表示数据准备好
    while (HAL_GPIO_ReadPin(AD1231_DRDY_PORT, AD1231_DRDY_PIN) == GPIO_PIN_SET)
    {
        HAL_SPI_Transmit(hspi, txBuffer, 3, HAL_MAX_DELAY);
    }

    // 接收3字节数据
    if (HAL_SPI_TransmitReceive(hspi, txBuffer, rxBuffer, 3, HAL_MAX_DELAY) == HAL_OK) {
        // 将3字节数据组合为24位数据
        rawData = (rxBuffer[0] << 16) | (rxBuffer[1] << 8) | rxBuffer[2];

        // 检查是否为负数或数据变动是否过大
        if (rawData & 0x800000) {
            printf("Error: Negative data detected (rawData = 0x%06X)\n", rawData);
            // 使用上一次的有效值
            signedData = lastValidData;
        } else {
            // 正数且数据变动在允许范围内，直接转换为32位整数
            signedData = (int32_t)rawData;
            // 更新上一次的有效值
            lastValidData = signedData;
        }

        // 直接使用原始数据计算重量
        result = signedData; // 保存原始数据
    }
    return result;
}

// 函数：根据输入传感器值计算桶内的重量（单位：克）(滑动平均滤波)
double calculate_weight(double ad_value1, double ad_value2) {
    // 计算当前重量
    double current_weight = (W1 * ad_value1 + W2 * ad_value2 + B);

    // 将当前重量值存入缓冲区
    weight_buffer[buffer_index] = current_weight;

    // 更新缓冲区索引
    buffer_index = (buffer_index + 1) % FILTER_SIZE;

    // 计算滑动平均值
    double sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += weight_buffer[i];
    }
    double average_weight = sum / FILTER_SIZE;

    return average_weight;
}
// 函数：根据输入传感器值计算桶内的重量（单位：克）（指数平滑滤波）
double calculate_weight2(double ad_value1, double ad_value2) {
    // 计算当前重量
    double current_weight = (W1 * ad_value1 + W2 * ad_value2 + B);

    // 初始化上一次的平滑值（如果是第一次调用，则直接使用当前值）
    static double previous_smoothed_weight = 0;  // 静态变量，保存上一次的平滑值

    // 如果是第一次调用，直接使用当前值
    if (previous_smoothed_weight == 0) {
        previous_smoothed_weight = current_weight;
    }

    // 计算指数平滑值
    double smoothed_weight = alpha * current_weight + (1 - alpha) * previous_smoothed_weight;

    // 更新上一次的平滑值
    previous_smoothed_weight = smoothed_weight;

    return smoothed_weight;
}
// 改进后的卡尔曼滤波函数
double kalman_update(double Z1, double Z2, double *P, double *W_est) {
    // 内部参数设置
    double Q = 5;   // 过程噪声协方差
    double R = 100.0; // 测量噪声协方差
    double d = 500.0;   // 野值判断的门限系数（通常取3，表示3倍方差根）

    // 计算测量值
    double W_meas = W1 * Z1 + W2 * Z2 + B;

    // 预测步骤
    double W_pred = *W_est; // 预测的重量（假设物品重量不变）
    double P_pred = *P + Q; // 预测协方差

    // 隐含信息计算
    double Z_hat = W_meas - W_pred; // 隐含信息（测量值与预测值的差异）
    double C = P_pred + R;          // 隐含信息的方差

    // 野值判断
    double sigma = sqrt(C); // 隐含信息的标准差
    double m = 1.0;         // 卡尔曼增益调整系数，默认为1（不调整）

    if (fabs(Z_hat) > d * sigma) {
        // 如果隐含信息超过门限值，认为是野值，调整卡尔曼增益
        m = 0.0; // 令 m = 0，剔除野值点
    }

    // 更新步骤
    double K = m * (P_pred / (P_pred + R)); // 调整卡尔曼增益
    *W_est = W_pred + K * Z_hat;            // 更新估计值
    *P = (1 - K) * P_pred;                  // 更新协方差

    return *W_est; // 返回当前估计值
}