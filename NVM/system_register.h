/*
*********************************************************************************************************
*
*    模块名称 : MODBUS从站寄存器操作
*    文件名称 : system_register.h
*    版    本 : V1.0
*    说    明 : 头文件
*
*    Copyright (C), 2019-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __MODBUS_REGISTER_H
#define __MODBUS_REGISTER_H
#include <stdint.h>

#define MAIN_VERSION          100
#define SUB_VERSION           001
#define SOFT_VERSION          (MAIN_VERSION << 8 | SUB_VERSION)

uint8_t WriteRegValue(uint16_t reg_addr, uint16_t reg_value);
uint8_t ReadRegValue(uint16_t _reg_addr, uint16_t *_reg_value);

void    Jodell_SN_init(void);

typedef struct 
{
    uint16_t    magic;
    char        corpName[2];
    char        prodMod[5];
    uint8_t     version;
    char        custom[2];
    char        securcode[2];
    char        year[2];
    char        week[2];
    char        order[4]; 
}jodell_devsn_t;

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
