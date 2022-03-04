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
#include "main.h"


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
    return true;
}

void MotorTask::on_exit(void)
{
    clamper_set_torque(0);
}