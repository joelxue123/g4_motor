/*
*********************************************************************************************************
*
*    模块名称 : 在线升级固件。
*    文件名称 : modbus_iap.c
*    版    本 : V1.0
*    说    明 : 
*
*    修改记录 :
*        版本号  日期        作者     说明
*        V1.0    2013-02-01 armfly  正式发布
*
*    Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "main.h"
#include "modbus_iap.h"
#include "modbus_slave.h"
#include "system_reg_addr.h"
#include "eeprom_emul.h"
#include <string.h>

uint32_t g_FlashAddr = 0;                /* CPU当前写入的地址 */
uint8_t s_Databuf[1024] = {0}; /* 升级程序buf。接收到1024个字节一次性写入cpu flash */
uint32_t g_DataLen = 0;                    /* 最后一包包长 */
uint32_t g_File_CRC = 0;

uint8_t g_Upgrade = 0;
uint8_t g_Erase = 0; /* APP应用区代码擦除标志 */
uint8_t g_need_bootloader = 0;

uint8_t IAP_Write06H(uint16_t reg_addr, uint16_t reg_value)
{
    switch (reg_addr)
    {
    case REG03_BOOT_UPGRADE_FLAG: /* 程序升级标志寄存器 */
        if(reg_value == 1)
        {
            g_need_bootloader = 1;
        }
        break;
        
    default:
        return 0; /* 参数异常，返回 0 */
    }

    return 1; /* 成功 */
}
    
uint16_t RecordID;                /* 记录号 */
/*
*********************************************************************************************************
*    函 数 名: MODS_15H
*    功能说明: 写文件
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void MODS_15H(void)
{
    /*
        主机发送:
            11 从机地址
            15 功能码
            00 请求数据长度
            01 子请求x，参考类型
            00 子请求x，文件号
            01 子请求x，记录号
            9A 子请求x，记录长度
            9B 子请求x，记录数据
            18 校验高字节
            FC 校验低字节
    
        从机响应:
            11 从机地址
            15 功能码
            00 请求数据长度
            01 子请求x，参考类型
            00 子请求x，文件号
            01 子请求x，记录号
            9A 子请求x，记录长度
            9B 子请求x，记录数据
            18 校验高字节
            FC 校验低字节
*/
#if 0
    uint8_t i;

    uint32_t RecordLen;                /* 记录长度 */
    uint32_t Packet;                    /* 第几包数据 */
    static uint32_t s_LenBak; /* 记录之前的数据长度，如果长度与之前的不同，则认为是最后一包数据,需要写入 */

    g_tModS.RspCode = RSP_OK;
    RecordID = BEBufToUint16(&g_tModS.RxBuf[6]);    /* 子请求x，记录号 */
    RecordLen = BEBufToUint16(&g_tModS.RxBuf[8]) * 2; /* 子请求x，记录长度 */

    if (RecordID == 0) /* 第一包数据，就把flash写入地址设为基地址,同时擦除应用区代码 */
    {
        g_FlashAddr = APPLICATION_BAK_ADDRESS;
        FLASH_If_Erase_App_Bak();
        g_File_CRC = 0;
        g_Erase = 1;
        s_LenBak = RecordLen; /* 第1包的数据长度，认为是收到每包的长度 */
        g_Upgrade = 1;
    }

    Packet = RecordID + 1;
    uint16_t cur_ID = (g_FlashAddr - APPLICATION_BAK_ADDRESS) / 128;

    if(Packet > cur_ID)
    {
        memcpy(&s_Databuf[(RecordID * s_LenBak) % 1024], &g_tModS.RxBuf[10], RecordLen); /* 组成1K数据再写入CPU flash */

        if ((Packet * s_LenBak) % 1024 != 0) /* 判断当前数据包是否满足1K的整数倍 */
        {
            g_DataLen = ((RecordID * s_LenBak) % 1024) + RecordLen; /* 记录当前需要写入的包长 */
        }
        else /* 满足1K的整数倍，此时才开始将1K数据写入CPU flash */
        {
            if (FLASH_If_Write(g_FlashAddr, (uint32_t *)s_Databuf, 256) == 0) /* 每次写入1024个字节 */
            {
                g_FlashAddr = g_FlashAddr + 1024;                /* 下一包数据写入的位置，1024的整数倍 */
                g_tModS.RspCode = RSP_OK;
            }
            else
            {
                g_tModS.RspCode = RSP_ERR_WRITE; /* 写入失败 */
                goto err_ret;
            }
        }
    }
    else
    {
        g_tModS.RspCode = RSP_OK;
    }

err_ret:
    if (g_tModS.RspCode == RSP_OK) /* 正确应答 */
    {
        g_tModS.TxCount = 0;
        for (i = 0; i < 10; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[i]; /* 应答数据包 */
        }
        g_tModS.TxBuf[2] = 7;
        MODS_SendWithCRC();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode); /* 告诉主机命令错误 */
    }
#endif
    g_tModS.RspCode = RSP_ERR_WRITE; /* 写入失败 */
    MODS_SendAckErr(g_tModS.RspCode); /* 告诉主机命令错误 */
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
