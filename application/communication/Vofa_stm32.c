/*
 * VOFA 硬件适配 - USART1 (串口)
 * 数据通过 USART1 (PA9 TX, 115200) 发送到上位机 Vofa+
 */

#include "Vofa.h"
#include "usart.h"

/*============================================================
 * Vofa_SendDataCallBack
 * 通过 USART1 阻塞发送
 * Vofa_JustFloat 分两次回调：先发浮点数据，再发帧尾 {0x00,0x00,0x80,0x7f}
 * 阻塞发送可保证两次回调数据连续发出，不丢失帧尾
 *============================================================*/
void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length)
{
    (void)handle;
    HAL_UART_Transmit(&huart1, data, length, 100);
}

/*============================================================
 * Vofa_GetDataCallBack
 * 从 USB CDC 读取一个字节（上位机下发命令时使用）
 * 当前仅用于 Vofa_JustFloat 单向发送，接收暂不实现
 *============================================================*/
uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
    (void)handle;
    return 0;  // USB CDC 接收需单独实现（中断/DMA方式）
}
