/*
*********************************************************************************************************
*
*    模块名称 : MODBUS 寄存器地址定义 （应用层）
*    文件名称 : system_reg_addr.h
*    版    本 : V1.0
*    说    明 : 头文件
*
*
*********************************************************************************************************
*/

#ifndef __MODBUS_REG_ADDR_H
#define __MODBUS_REG_ADDR_H

#define REG03_SET_CONTROL                   0x03E8
#define REG03_SET_RECONFINGER_POS           0x03EB
#define REG03_SET_RECONFINGER_VEL           0x03EC
#define REG03_SET_RECONFINGER_CUR           0x03ED
#define REG03_SET_LEFTFINGER_POS            0x03EE
#define REG03_SET_LEFTFINGER_VEL            0x03EF
#define REG03_SET_LEFTFINGER_CUR            0x03F0
#define REG03_SET_MIDDLEFINGER_POS          0x03F1
#define REG03_SET_MIDDLEFINGER_VEL          0x03F2
#define REG03_SET_MIDDLEFINGER_CUR          0x03F3
#define REG03_SET_RIGHTFINGER_POS           0x03F4
#define REG03_SET_RIGHTFINGER_VEL           0x03F5
#define REG03_SET_RIGHTFINGER_CUR           0x03F6

#define REG03_GET_STATUS                    0x07D0
#define REG03_GET_EROOR                     0x07D2
#define REG03_GET_RECONFINGER_TARGET        0x07D3
#define REG03_GET_RECONFINGER_POS           0x07D4
#define REG03_GET_RECONFINGER_CUR           0x07D5
#define REG03_GET_LEFTFINGER_TARGET         0x07D6
#define REG03_GET_LEFTFINGER_POS            0x07D7
#define REG03_GET_LEFTFINGER_CUR            0x07D8
#define REG03_GET_MIDDLEFINGER_TARGET       0x07D9
#define REG03_GET_MIDDLEFINGER_POS          0x07DA
#define REG03_GET_MIDDLEFINGER_CUR          0x07DB
#define REG03_GET_RIGHTFINGER_TARGET        0x07DC
#define REG03_GET_RIGHTFINGER_POS           0x07DD
#define REG03_GET_RIGHTFINGER_CUR           0x07DE

/* 06H boot波特率寄存器 和 程序升级寄存器 */
#define REG03_SYS_RESET 0x9100
#define REG03_SYS_MAJOR_VESION 0x9101
#define REG03_SYS_MINOR_VESION 0x9102
#define REG03_SYS_Build_VESION 0x9103
#define REG03_BOOT_UPGRADE_FLAG 0x9104
#endif

/***************************** (END OF FILE) *********************************/
