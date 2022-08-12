/**
 * @file motortask.cpp
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

#include "motortask.hpp"
#include "clamper.h"
#include "interface.h"
#include "motor.hpp"
#include "global.hpp"
#include "main.h"
extern "C"
{
    #include "stspin32g4.h"
}

extern STSPIN32G4_HandleTypeDef hdlG4;

bool MotorTask::on_init(void)
{
    clamper_init();
    return true;
}

bool MotorTask::on_high_realtime_update(uint32_t _tick)
{
    clamper_on_update();
    return true;
}

bool MotorTask::on_realtime_update(uint32_t _tick)
{
    return true;
}

bool MotorTask::on_none_realtime_update(uint32_t _tick)
{
    clamper_on_main();
    if(g_fault_flag == 1)
    {
        STSPIN32G4_clearFaults(&hdlG4);
    }
    return true;
}

void MotorTask::on_exit(void)
{
    g_motor.servo_off();
}