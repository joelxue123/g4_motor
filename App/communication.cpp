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

extern "C"
{
    #include "eeprom_emul.h"
}

Communication * Communication::_instance = nullptr;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* phuart_)
{
  if(phuart_ == &huart2)
  {
    //HAL_GPIO_WritePin(PIN_UART2_RE_GPIO_Port, PIN_UART2_RE_Pin, GPIO_PIN_RESET);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* phuart_)
{
  if(phuart_ == &huart2)
  {
      Communication::getInstance()->on_data_error();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* phuart_, uint16_t Size)
{
  if(phuart_ == &huart2)
  {
    Communication::getInstance()->on_data_recv(Size);
  }
}

bool Communication::on_init(void)
{
    //Jodell_SN_init();
    memset(RX485_buf_Size, 0, UART_RX_BUFFER_NUM * 2);
    RX485_buf_Write_prt = 0;
    RX485_buf_Read_prt = 0;
    //HAL_GPIO_WritePin(PIN_UART2_RE_GPIO_Port, PIN_UART2_RE_Pin, GPIO_PIN_RESET);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);
    RX485_flag = 0;

    EE_ReadVariable32bits(0x2, &station_address);
    if(station_address == 0x00 || station_address >= 0xff)
    {
        station_address = MODS_DEFAULT_ADDRESS;
    } 
    MODS_SetAddress(station_address);
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
        uint32_t _data_offset = 0;

        g_tModS.TxCount = 0;
        if(MODS_Poll(recv_data_ptr, RX485_buf_Size[RX485_buf_Read_prt]))
        {
            if(g_tModS.TxCount != 0)
            {
                //HAL_GPIO_WritePin(PIN_UART2_RE_GPIO_Port, PIN_UART2_RE_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit_DMA(&huart2, g_tModS.TxBuf, g_tModS.TxCount);
            }
        }
        else
        {
            if(g_tModS.TxCount != 0)
            {
                //HAL_GPIO_WritePin(PIN_UART2_RE_GPIO_Port, PIN_UART2_RE_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit_DMA(&huart2, g_tModS.TxBuf, g_tModS.TxCount);
            }
        }
        RX485_buf_Read_prt++;
        RX485_buf_Read_prt = RX485_buf_Read_prt % UART_RX_BUFFER_NUM;
    }
    return true;
}

bool test_header(uint8_t * p_buffer)
{
    if(p_buffer[0] != g_tModS.g_station_address
      && p_buffer[0] != 0x00
      && p_buffer[0] != 0xff)
    {
        return true;
    }

    if(p_buffer[1] != 0x03
      && p_buffer[1] != 0x06
      && p_buffer[1] != 0x10
      && p_buffer[1] != 0x15)
    {
        return true;
    }

    return false;
}

void Communication::on_data_recv(uint16_t Size)
{
    if(Size < 8 || test_header(RX485_buf[RX485_buf_Write_prt]))
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
    //HAL_GPIO_WritePin(PIN_UART2_RE_GPIO_Port, PIN_UART2_RE_Pin, GPIO_PIN_RESET);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX485_buf[RX485_buf_Write_prt], UART_RX_BUFFER_SIZE);
}