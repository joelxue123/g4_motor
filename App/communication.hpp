/**
 * @file communication.hpp
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

#ifndef __COMMUNICATION_HPP
#define __COMMUNICATION_HPP

#include "taskmanager.hpp"

#define UART_RX_BUFFER_SIZE 512
#define UART_RX_BUFFER_NUM  8

class Communication : public TaskManager
{
    public:
        Communication(void){
            station_address = 0;
            RX485_flag = 0;
            _instance = this;
        };
        ~Communication(void){};

        static Communication* getInstance(void){return _instance;}

        virtual bool on_init(void);

        virtual bool on_high_realtime_update(uint32_t _tick);

        virtual bool on_realtime_update(uint32_t _tick);

        virtual bool on_none_realtime_update(uint32_t _tick); 

        void on_data_recv(uint16_t Size);

        void on_data_error(void);
        

    private:
        uint32_t  station_address = 0;
        uint8_t  RX485_flag = 0;
        uint8_t  RX485_buf[UART_RX_BUFFER_NUM][UART_RX_BUFFER_SIZE] = {0};
        uint16_t RX485_buf_Size[UART_RX_BUFFER_NUM] = {0};
        uint32_t RX485_buf_Write_prt = 0;
        uint32_t RX485_buf_Read_prt = 0;

	    static Communication * _instance;
};

#endif