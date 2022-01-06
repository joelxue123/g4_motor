/**
 * @file taskmanager.h
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

#ifndef __TASKMANAGER_H
#define __TASKMANAGER_H

#include <stdint.h>
#include <vector>

#define SEC_IN_MILSEC 500000

using namespace std;

class TaskManager;

/**
 * @brief Task接口类，所有Task继承此类
 */
class TaskManager
{
    public:
        TaskManager(void) 
        {
            m_is_alive = false;
            m_task_id = total_task;
            task_list.push_back(this);
            total_task++;
        };
        ~TaskManager(void){};

        virtual bool on_init(void) = 0;

        virtual bool on_high_realtime_update(uint32_t _tick) = 0;

        virtual bool on_realtime_update(uint32_t _tick) = 0;

        virtual bool on_none_realtime_update(uint32_t _tick) = 0;

        virtual void on_exit(void) {};

        uint32_t get_task_id(void) {return m_task_id;}

        bool is_alive(void) {return m_is_alive;}

        static bool high_realtime_update(void);

        static bool realtime_update(void);

        static bool none_realtime_update(void);

        static bool init(void);

        static void exit(void);

        //const static float high_reatime_period_us;
        const static uint32_t none_reatime_period_us = 1000;

    private:
        uint32_t m_task_id;
        bool     m_is_alive;

        static uint32_t total_task;
        static std::vector<TaskManager*> task_list;
        static uint32_t high_realtime_tick;
        static uint32_t realtime_tick;
        static uint32_t none_realtime_tick;
};

#endif