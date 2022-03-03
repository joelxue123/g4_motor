/**
 * @file communication.cpp
 * @brief 
 * @author htkang (kanght@jodell.cn)
 * @version 1.0
 * @date 2021-06-23
 * 
 * @copyright Copyright (c) 2021  苏州钧舵机器人有限公司
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-06-23 <td>1.0     <td>htkang616     <td>内容
 * </table>
 */
#include "communication.hpp"
#include "modbus_slave.h"
#include "system_register.h"
#include "stm32g4xx_hal.h"
#include "usart.h"
#include <memory.h>
#include <stdint.h>
#include "clamper.h"

extern "C"
{
    #include "eeprom_emul.h"
    #include "TinyFrame.h"
}

Communication * Communication::_instance = nullptr;
clamper_ctrl_t g_clamper_ctrl = {0};
clamper_status_t g_clamper_status = {0};
TinyFrame g_tiny_frame = {0};

void HAL_UART_ErrorCallback(UART_HandleTypeDef* phuart_)
{
  if(phuart_ == &huart1)
  {
      Communication::getInstance()->on_data_error();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* phuart_, uint16_t Size)
{
  if(phuart_ == &huart1)
  {
    Communication::getInstance()->on_data_recv(Size);
  }
}

TF_Result replyListener(TinyFrame *tf, TF_Msg *msg)
{
   memcpy(&g_clamper_ctrl, msg->data, msg->len);
   msg->data = (const uint8_t *)&g_clamper_status;
   msg->len = sizeof(clamper_status_t);
   TF_Respond(tf, msg);
   return TF_STAY;
}

bool Communication::on_init(void)
{
    TF_InitStatic(&g_tiny_frame, TF_SLAVE);
    TF_AddTypeListener(&g_tiny_frame, 1, replyListener);
    memset(RX485_buf_Size, 0, UART_RX_BUFFER_NUM * 2);
    RX485_buf_Write_prt = 0;
    RX485_buf_Read_prt = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);

    return true;
}

extern uint8_t g_need_bootloader;

bool Communication::on_high_realtime_update(uint32_t _tick)
{
    g_clamper_status.StatusWord = clamper_get_status();
    g_clamper_status.AbsPosition = clamper_spi_get_pos();
    g_clamper_status.AbsVelocity = clamper_spi_get_vel();
    g_clamper_status.AbsTorque = clamper_get_torque();
    clamper_set_status(g_clamper_ctrl.ControlWord);
    clamper_spi_set_vel(g_clamper_ctrl.AbsVelocity);
    clamper_spi_set_torque(g_clamper_ctrl.AbsTorque);
    clamper_spi_set_pos(g_clamper_ctrl.AbsPosition);

    if(g_clamper_ctrl.boot_flag == 1)
    {
        g_clamper_status.is_boot = 1;
        g_need_bootloader = 1;
    }

    return true;
}

bool Communication::on_realtime_update(uint32_t _tick)
{
    return true;
}

bool Communication::on_none_realtime_update(uint32_t _tick)
{
    if(RX485_buf_Write_prt != RX485_buf_Read_prt)
    {
        uint8_t * recv_data_ptr = RX485_buf[RX485_buf_Read_prt];
        TF_Accept(&g_tiny_frame, recv_data_ptr, Size);
        RX485_buf_Read_prt++;
        RX485_buf_Read_prt = RX485_buf_Read_prt % UART_RX_BUFFER_NUM;
    }
    return true;
}

void Communication::on_data_recv(uint16_t Size)
{ 
    RX485_buf_Size[RX485_buf_Write_prt] = Size;
    RX485_buf_Write_prt++;
    RX485_buf_Write_prt = RX485_buf_Write_prt % UART_RX_BUFFER_NUM;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);
}

void Communication::on_data_error(void)
{
    HAL_UART_AbortReceive(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);
}