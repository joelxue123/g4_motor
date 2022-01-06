/**
 * @file ledtask.hpp
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

#ifndef __LED_TASK_HPP
#define __LED_TASK_HPP

#include "taskmanager.hpp"

class LedTask : public TaskManager
{
    public:
        LedTask(void){};
        ~LedTask(void){};

        virtual bool on_init(void);

        virtual bool on_high_realtime_update(uint32_t _tick);

        virtual bool on_realtime_update(uint32_t _tick);

        virtual bool on_none_realtime_update(uint32_t _tick); 

    private:
        bool is_err(void);
        bool is_fault(void);
};

#endif