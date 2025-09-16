#include "uart2.h"

uint8_t uart2_tx_buf[100]= {0};
void u2printf(const char *str)
{
    memset(uart2_tx_buf, 0, sizeof(uart2_tx_buf));
    snprintf((char *)uart2_tx_buf, sizeof(uart2_tx_buf), "%s", str);
    HAL_UART_Transmit_IT(&huart2, uart2_tx_buf, sizeof(uart2_tx_buf));
}