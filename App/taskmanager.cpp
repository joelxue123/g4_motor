/**
 * @file taskmanager.cpp
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

#include "taskmanager.hpp"
#include "ledtask.hpp"
#include "motortask.hpp"
#include "communication.hpp"

uint32_t TaskManager::total_task = 0;
std::vector<TaskManager*> TaskManager::task_list;
uint32_t TaskManager::high_realtime_tick = 0;
uint32_t TaskManager::realtime_tick = 0;
uint32_t TaskManager::none_realtime_tick = 0;

MotorTask     g_motor_task;
//Communication g_communication_task;
//LedTask       g_led_task;

bool TaskManager::init()
{
    bool _ret = true;
    for(auto task_ptr : task_list)
    {
        _ret = _ret && task_ptr->on_init();
    }
    return _ret;
}

bool TaskManager::high_realtime_update(void)
{
    high_realtime_tick++;
    bool _ret = true;
    for(auto task_ptr : task_list)
    {
        _ret = _ret && task_ptr->on_high_realtime_update(high_realtime_tick);
    }
    return _ret;
}

bool TaskManager::realtime_update(void)
{
    realtime_tick++;
    bool _ret = true;
    for(auto task_ptr : task_list)
    {
        _ret = _ret && task_ptr->on_realtime_update(realtime_tick);
    }
    return _ret;
}

bool TaskManager::none_realtime_update(void)
{
    none_realtime_tick++;
    bool _ret = true;
    for(auto task_ptr : task_list)
    {
        _ret = _ret && task_ptr->on_none_realtime_update(none_realtime_tick);
    }
    return _ret;
}

void TaskManager::exit(void)
{
    for(auto task_ptr : task_list)
    {
        task_ptr->on_exit();
    }
}