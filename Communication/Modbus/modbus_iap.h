/*
*********************************************************************************************************
*
*    模块名称 : RS485 MODEBUS 通信模块
*    文件名称 : modbus_rs485.h
*    版    本 : V1.0
*    说    明 : 头文件
*
*    Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __MODBUS_IAP_H_
#define __MODBUS_IAP_H_

#include <stdint.h>

void MODS_15H(void);
uint8_t IAP_Write06H(uint16_t reg_addr, uint16_t reg_value);

#endif
