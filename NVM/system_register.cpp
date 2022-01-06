/*
*********************************************************************************************************
*
*    模块名称 : MODBUS从机模块
*    文件名称 : tcp_MODS_slave.c
*    版    本 : V1.0
*    说    明 : 头文件
*
*    Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "system_reg_addr.h"
#include "system_register.h"
#include "clamper.h"
#include "rotator.h"

extern "C"
{
    #include "eeprom_emul.h"
}

uint8_t fSaveReq_06H = 0;         /* 保存基本参数请求，用于06H和10H写寄存器函数 */
uint8_t fResetReq_06H = 0;     /* 需要复位CPU，因为网络参数变化 */
uint8_t fSaveCalibParam = 0; /* 保存校准参数请求，用于06H和10H写寄存器函数 */
extern uint8_t g_need_reboot;
uint8_t reg_high = 0;
uint8_t reg_low = 0;
jodell_devsn_t g_devsn = {0};

void Jodell_SN_init(void)
{
#if 0
    ee24_read(3, (uint8_t *)&g_devsn, sizeof(jodell_devsn_t), 100);
    if(g_devsn.magic != 0x5AA5)
    {
        g_devsn.magic = 0x5AA5;
        g_devsn.corpName[0] = 'J';
        g_devsn.corpName[1] = 'D';

        g_devsn.prodMod[0] = 'E';
        g_devsn.prodMod[1] = 'R';
        g_devsn.prodMod[2] = 'G';
        g_devsn.prodMod[3] = 0;
        g_devsn.prodMod[4] = 0;

        g_devsn.version = 0;
        g_devsn.custom[0] = 0;
        g_devsn.custom[1] = 0;

        g_devsn.securcode[0] = 0;
        g_devsn.securcode[1] = 0;
        g_devsn.year[0] = 0;
        g_devsn.year[1] = 0;
        g_devsn.week[0] = 0;
        g_devsn.week[1] = 0;
        g_devsn.order[0] = 0;
        ee24_write(3, (uint8_t *)&g_devsn, sizeof(jodell_devsn_t), 100);
    }
#endif
}

uint32_t uart_BaudRate = 0;

uint8_t WriteRegValue(uint16_t reg_addr, uint16_t reg_value)
{
    uint8_t _ret = 1;

    if(reg_addr > 0x03EF && reg_addr < 0x0400)
    {
        int flag = (reg_addr - 0x03F0) % 2;
        int i = (reg_addr - 0x03F0) / 2;
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff; 

        if(flag == 0)
        {
            clamper_set_param(0, i, reg_low);
        }
        else {
            clamper_set_param(1, i, reg_low);
            clamper_set_param(2, i, reg_high);
            clamper_set_param(3, i, 1);
        }

        return 1;
    }

    if(reg_addr > 0x03FF && reg_addr < 0x0410)
    {
        int flag = (reg_addr - 0x0400) % 2;
        int i = (reg_addr - 0x0400) / 2;
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff; 

        if(flag == 0)
        {
            int16_t* turn_set = (int16_t*)&reg_value;
            rotator_set_param(0, i, *turn_set);
        }
        else {
            rotator_set_param(1, i, reg_low);
            rotator_set_param(2, i, reg_high);
            rotator_set_param(3, i, 1);
        }

        return 1;
    }

    switch (reg_addr)
    {
    case 0x03E8:
        {
            reg_high = (reg_value & 0xff00) >> 8;
            reg_low = reg_value & 0x00ff; 
            clamper_set_mode(reg_high);
            clamper_set_status(reg_low);
            break;
        }
    case 0x03E9:
        {
            reg_high = (reg_value & 0xff00) >> 8;
            reg_low = reg_value & 0x00ff;
            rotator_set_mode(reg_high);
            rotator_set_status(reg_low);
            break;
        }
    case 0x03EA:
        {
            reg_high = (reg_value & 0xff00) >> 8;
            reg_low = reg_value & 0x00ff;
            clamper_set_pos(reg_low);
            clamper_set_vel(reg_high);   
            break;
        } 
    case 0x03EB:
        {
            reg_high = (reg_value & 0xff00) >> 8;
            reg_low = reg_value & 0x00ff;
            clamper_set_torque(reg_high);
            if(reg_low == 0x1)
            {
                clamper_set_start_move();
            }
            break;
        }
    case 0x03EC:
        {
            int16_t* turn_set = (int16_t*)&reg_value;
            rotator_set_turn(*turn_set);
            break;
        }
    case 0x03ED:
        {   
            reg_high = (reg_value & 0xff00) >> 8;
            reg_low = reg_value & 0x00ff;
            rotator_set_vel(reg_low);
            rotator_set_torque(reg_high);
            break;
        }
    case 0x03EE:
        {   
            int16_t* turn_set = (int16_t*)&reg_value;
            rotator_set_relative_turn(*turn_set);
            break;
        }
    case 0x03EF:
        {   
            int8_t _reg_high = (reg_value & 0xff00) >> 8;
            reg_low = reg_value & 0x00ff;
            rotator_set_pos(_reg_high);
            rotator_set_start_move(reg_low);
            break;
        }

    case 0x138a:
        {  
            reg_high = (reg_value & 0xff00) >> 8;
            reg_low = reg_value & 0x00ff;
            //ee24_write(2, &reg_low, 1, 20);
            HAL_FLASH_Unlock(); 
            EE_WriteVariable32bits(0x2, reg_low); 
            HAL_FLASH_Lock();
            g_need_reboot = 1;
            break;
        }

    case 0x138e:
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff;
        g_devsn.version = reg_high;
        g_devsn.custom[0] = reg_low;
        break;

    case 0x138f:
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff;
        g_devsn.custom[1] = reg_high;
        g_devsn.securcode[0] = reg_low;
        break;
    case 0x1390:
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff;
        g_devsn.securcode[1] = reg_high;
        g_devsn.year[0] = reg_low;
        break;
    case 0x1391:
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff;
        g_devsn.year[1] = reg_high;
        g_devsn.week[0] = reg_low;
        break;
    case 0x1392:
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff;
        g_devsn.week[1] = reg_high;
        g_devsn.order[0] = reg_low;
        break;
    case 0x1393:
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff;
        g_devsn.order[1] = reg_high;
        g_devsn.order[2] = reg_low;
        break;
    case 0x1394:
        reg_high = (reg_value & 0xff00) >> 8;
        reg_low = reg_value & 0x00ff;
        g_devsn.order[3] = reg_high;
        if(reg_low == 1)
        {
            //ee24_write(3, (uint8_t *)&g_devsn, sizeof(jodell_devsn_t), 100);
        }
        break;
    case 0x1395:
        if(reg_value >= 0 && reg_value <= 0x5)
        {
            uart_BaudRate = reg_value;
            //EE_Init(); 
            HAL_FLASH_Unlock(); 
            EE_WriteVariable32bits(0x1, uart_BaudRate); 
            HAL_FLASH_Lock();
            g_need_reboot = 1;
        }
        break;
    case 0x0410:
        // 将外界设置的寄存器值赋值给旋转偏置，并存入Eeprom
        rotator_set_offset(reg_value);
        break;

    default:
        _ret = 0;
        break;
    }
    return _ret;
}

uint8_t ReadRegValue(uint16_t _reg_addr, uint16_t *_reg_value)
{
    uint8_t _ret = 1;
    uint8_t reg_high = 0;
    uint8_t reg_low = 0;

    if(_reg_addr > 0x03EF && _reg_addr < 0x0400)
    {
        int flag = (_reg_addr - 0x03F0) % 2;
        int i = (_reg_addr - 0x03F0) / 2;

        if(flag == 0)
        {
            *_reg_value = clamper_get_param(0, i);
        }
        else {
            *_reg_value = (clamper_get_param(2, i) << 8) + clamper_get_param(1, i);
        }

        return 1;
    }

    if(_reg_addr > 0x03FF && _reg_addr < 0x0410)
    {
        int flag = (_reg_addr - 0x0400) % 2;
        int i = (_reg_addr - 0x0400) / 2;

        if(flag == 0)
        {
            *_reg_value = rotator_get_param(0, i);
        }
        else {
            *_reg_value = (rotator_get_param(2, i) << 8) + rotator_get_param(1, i);
        }

        return 1;
    }

    switch (_reg_addr)
    {
    case 0x07D0:
        reg_high = 0;
        reg_low = clamper_get_status();
        *_reg_value = (reg_high << 8) + reg_low;
        break;
    case 0x07D1:
        reg_high = 0;
        reg_low = rotator_get_status();
        *_reg_value = (reg_high << 8) + reg_low;
        break;
    case 0x07D2:
        reg_high = clamper_get_vel();
        reg_low = clamper_get_pos();
        *_reg_value = (reg_high << 8) + reg_low;
        break;
    case 0x07D3:
    {
        reg_high = clamper_get_torque();
        reg_low = 0;
        *_reg_value = (reg_high << 8) + reg_low;
        break;
    }
    case 0x07D4:
    {
        *_reg_value = rotator_get_turn();
        break;
    }

    case 0x07D5:
    {
        reg_high = rotator_get_torque();
        reg_low = rotator_get_vel();
        *_reg_value = (reg_high << 8) + reg_low;
        break;
    }

    case 0x07D6:
    {
        reg_high = rotator_get_pos();
        reg_low = rotator_get_relative_mode();
        *_reg_value = (reg_high << 8) + reg_low;
        break;
    }

    case 0x07D7:
    {
        *_reg_value = rotator_get_relative_turn();
        break;
    }

    case 0x07D8:
    {
        reg_high = 24;
        reg_low = 25;
        *_reg_value = (reg_high << 8) + reg_low;
        break;
    }

    case 0x1388:
        *_reg_value = 0x20;
        break;
    case 0x1389:
        *_reg_value = SOFT_VERSION;
        break;
    case 0x138a:
        *_reg_value = g_devsn.corpName[0];
        break;
    case 0x138b:
        *_reg_value = g_devsn.corpName[1] * 256 + g_devsn.prodMod[0];
        break;
    case 0x138c:
        *_reg_value = g_devsn.prodMod[1] * 256 + g_devsn.prodMod[2];
        break;
    case 0x138d:
        *_reg_value = g_devsn.prodMod[3] * 256 + g_devsn.prodMod[4];
        break;
    case 0x138e:
        *_reg_value = g_devsn.version * 256 + g_devsn.custom[0];
        break;
    case 0x138f:
        *_reg_value = g_devsn.custom[1] * 256 + g_devsn.securcode[0];
        break;
    case 0x1390:
        *_reg_value = g_devsn.securcode[1] * 256 + g_devsn.year[0];
        break;
    case 0x1391:
        *_reg_value = g_devsn.year[1] * 256 + g_devsn.week[0];
        break;
    case 0x1392:
        *_reg_value = g_devsn.week[1] * 256 + g_devsn.order[0]; 
        break; 
    case 0x1393: 
        *_reg_value = g_devsn.order[1] * 256 + g_devsn.order[2]; 
        break; 
    case 0x1394: 
        *_reg_value = g_devsn.order[3] * 256 + 0; 
        break; 

    case REG03_SYS_MAJOR_VESION: 
        *_reg_value = 0; 
        break;

    case REG03_SYS_MINOR_VESION: 
        *_reg_value = 3;
        break;

    case REG03_SYS_Build_VESION:
        *_reg_value = 19;
        break;

    default:
        _ret = 0;
        break;
    }
        
    return _ret;
}
/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
