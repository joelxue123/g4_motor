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

#define UART_RX_BUFFER_SIZE 64
#define UART_RX_BUFFER_NUM  2


typedef struct {
uint8_t ControlWord; /* Subindex1 - controlWord */
int32_t AbsPosition; /* Subindex3 - adbPosition */
uint8_t AbsVelocity; /* Subindex4 - absVelocity */
uint8_t AbsTorque; /* Subindex6 - absTorque */
uint8_t boot_flag;
} clamper_ctrl_t;

extern clamper_ctrl_t g_clamper_ctrl;

typedef struct{
uint8_t StatusWord; /* Subindex1 - statusWord */
uint8_t is_boot; /* Subindex2 - errCode */
int32_t AbsPosition; /* Subindex3 - absPosition */
uint8_t AbsVelocity; /* Subindex4 - absVelocity */
uint8_t AbsTorque; /* Subindex5 - absTorque */
} clamper_status_t;

extern clamper_status_t g_clamper_status;
class Communication : public TaskManager
{
    public:
        Communication(void){
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
        uint8_t  RX485_buf[UART_RX_BUFFER_NUM][UART_RX_BUFFER_SIZE] = {0};
        uint16_t RX485_buf_Size[UART_RX_BUFFER_NUM] = {0};
        uint32_t RX485_buf_Write_prt = 0;
        uint32_t RX485_buf_Read_prt = 0;

	    static Communication * _instance;
};

#endif