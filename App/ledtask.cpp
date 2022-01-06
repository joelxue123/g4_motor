/**
 * @file ledtask.cpp
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

#include "ledtask.hpp"
#include "interface.h"
#include "global.hpp"
#include "motor.hpp"
#include "optical_encoder.hpp"


bool LedTask::on_init(void)
{
    return true;
}

bool LedTask::on_high_realtime_update(uint32_t _tick)
{
    return true;
}

bool LedTask::is_err(void)
{
    if(g_motor.error_ != Motor::ERROR_NONE)
    {
        return true;
    }

    if(g_optical_encoder.error_ != OpticalEncoder::ERROR_NONE)
    {
        return true;
    }
    return false;
}

bool LedTask::is_fault(void)
{
    return g_fault_flag;
}

bool LedTask::on_realtime_update(uint32_t _tick)
{
    return true;
}

bool LedTask::on_none_realtime_update(uint32_t _tick)
{
    if(_tick % (SEC_IN_MILSEC /  none_reatime_period_us) == 1)
    {
        if(is_fault())
        {
            led_fault_set();
            led_state_clr();
        }
        else {
            if(is_err())
            {
                led_fault_toggle();
                led_state_clr();
            }
            else {
                led_fault_clr();
                led_state_toggle();
            }   
        }
    }
    
    return true;
}

