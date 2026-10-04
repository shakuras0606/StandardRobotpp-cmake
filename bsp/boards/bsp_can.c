#include "bsp_can.h"
#include "bsp_delay.h"

uint32_t CAN_TX_FAIL_COUNT = 0;

#define CAN_TX_RETRY_TIMES    8U
#define CAN_TX_RETRY_DELAY_US 50U

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief          发送控制数据
 * @param[in]      can_ctrl_data 待发送的数据
 * @return         HAL_OK: 已写入发送邮箱, 本帧会由硬件发送;
 *                 其他: 等待超时或总线异常, 本帧被丢弃
 * @note           邮箱满时每 50us 重试一次, 最多 8 次(Bus-Off 立即放弃),
 *                 丢弃的帧计数在 CAN_TX_FAIL_COUNT，便于调试观察。
 */
HAL_StatusTypeDef CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data)
{
    if (can_ctrl_data == NULL || can_ctrl_data->hcan == NULL) {
        CAN_TX_FAIL_COUNT++;
        return HAL_ERROR;
    }

    uint32_t send_mail_box = 0;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
        can_ctrl_data->hcan, &can_ctrl_data->tx_header, can_ctrl_data->tx_data, &send_mail_box);

    for (uint8_t retry = 0; status != HAL_OK && retry < CAN_TX_RETRY_TIMES; retry++) {
        if ((can_ctrl_data->hcan->Instance->ESR & CAN_ESR_BOFF) != 0U) {
            break;
        }
        delay_us(CAN_TX_RETRY_DELAY_US);
        status = HAL_CAN_AddTxMessage(
            can_ctrl_data->hcan, &can_ctrl_data->tx_header, can_ctrl_data->tx_data, &send_mail_box);
    }

    if (status != HAL_OK) {
        CAN_TX_FAIL_COUNT++;
    }

    return status;
}
