#include "interface.h"
#include "taskmanager.hpp"

void main_setup(void)
{
    TaskManager::init();
    high_realtime_interrupt_init(); 
    system_delay_ms(100);
}

extern uint8_t g_need_bootloader;
uint8_t g_need_reboot = 0;

void main_loop(void)
{
    TaskManager::none_realtime_update();
    if(g_need_bootloader)
    {
        TaskManager::exit();
        system_delay_ms(300);
        system_jump_booloader();
    }

    if(g_need_reboot)
    {
        TaskManager::exit();
        system_delay_ms(300);
        system_restart();
    }
    system_delay_ms(1);
}

void high_realtime_interrupt(void)
{
    TaskManager::high_realtime_update();
}
