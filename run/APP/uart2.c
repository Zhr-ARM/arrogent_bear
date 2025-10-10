#include "uart2.h"

uint8_t uart2_tx_buf[100]= {0};
volatile uint8_t uart2_tx_busy = 0;

void u2printf(const char *str)
{
    while(uart2_tx_busy); // 等待上一次发送完成
    uart2_tx_busy = 1;
    memset(uart2_tx_buf, 0, sizeof(uart2_tx_buf));
    snprintf((char *)uart2_tx_buf, sizeof(uart2_tx_buf), "%s", str);
    HAL_UART_Transmit_IT(&huart2, uart2_tx_buf, strlen((char*)uart2_tx_buf)); // 确保参数正确
}

// 在 stm32f4xx_it.c 里
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        uart2_tx_busy = 0;
    }
}