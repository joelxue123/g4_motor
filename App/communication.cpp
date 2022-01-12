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
}

Communication * Communication::_instance = nullptr;
clamper_ctrl_t g_clamper_ctrl = {0};
clamper_status_t g_clamper_status = {0};

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* phuart_)
{
  if(phuart_ == &huart1)
  {

  }
}

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

bool Communication::on_init(void)
{
    memset(RX485_buf_Size, 0, UART_RX_BUFFER_NUM * 2);
    RX485_buf_Write_prt = 0;
    RX485_buf_Read_prt = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);

    return true;
}

bool Communication::on_high_realtime_update(uint32_t _tick)
{
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
        memcpy((void*)&g_clamper_ctrl, (void*)recv_data_ptr, sizeof(clamper_ctrl_t));
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&g_clamper_status, sizeof(clamper_status_t));
        RX485_buf_Read_prt++;
        RX485_buf_Read_prt = RX485_buf_Read_prt % UART_RX_BUFFER_NUM;
    }

    g_clamper_status.StatusWord = clamper_get_status();
    g_clamper_status.AbsPosition = clamper_spi_get_pos();
    g_clamper_status.AbsVelocity = clamper_spi_get_vel();
    g_clamper_status.AbsTorque = clamper_get_torque();
    clamper_set_status(g_clamper_ctrl.ControlWord);
    clamper_spi_set_vel(g_clamper_ctrl.AbsVelocity);
    clamper_spi_set_torque(g_clamper_ctrl.AbsTorque);
    clamper_spi_set_pos(g_clamper_ctrl.AbsPosition);
    return true;
}

void Communication::on_data_recv(uint16_t Size)
{
    if(Size != sizeof(clamper_ctrl_t))
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);
        return;
    }
  
    RX485_buf_Size[RX485_buf_Write_prt] = Size;
    RX485_buf_Write_prt++;
    RX485_buf_Write_prt = RX485_buf_Write_prt % UART_RX_BUFFER_NUM;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);
}

void Communication::on_data_error(void)
{
    HAL_UART_AbortReceive(&huart2);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);
}