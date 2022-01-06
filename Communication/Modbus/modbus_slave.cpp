/*
*********************************************************************************************************
*
*    模块名称 : MODBUS从机模块
*    文件名称 : modbus_slave.c
*    版    本 : V1.0
*    说    明 : 头文件
*
*    Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "usart.h"
#include "main.h"
#include "modbus_slave.h"
#include "system_reg_addr.h"
#include "system_register.h"
#include "modbus_iap.h"

/*
*********************************************************************************************************
*    函 数 名: LEBufToUint16
*    功能说明: 将2字节数组(小端Little Endian，低字节在前)转换为16位整数
*    形    参: _pBuf : 数组
*    返 回 值: 16位整数值
*********************************************************************************************************
*/
uint16_t LEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[1] << 8) | _pBuf[0]);
}

/*
*********************************************************************************************************
*    函 数 名: BEBufToUint16
*    功能说明: 将2字节数组(大端Big Endian次序，高字节在前)转换为16位整数
*    形    参: _pBuf : 数组
*    返 回 值: 16位整数值
*
*   大端(Big Endian)与小端(Little Endian)
*********************************************************************************************************
*/
uint16_t BEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}

// CRC 高位字节值表
static const uint8_t s_CRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

// CRC 低位字节值表
const uint8_t s_CRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
        0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
        0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
        0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
        0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
        0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
        0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
        0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
        0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
        0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
        0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
        0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
        0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
        0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
        0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
        0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
        0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
        0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

void MODS_SetAddress(uint8_t addr_)
{
    g_tModS.g_station_address = addr_;
}

/*
*********************************************************************************************************
*    函 数 名: CRC16_Modbus
*    功能说明: 计算CRC。 用于Modbus协议。
*    形    参: _pBuf : 参与校验的数据
*              _usLen : 数据长度
*    返 回 值: 16位整数值。 对于Modbus ，此结果高字节先传送，低字节后传送。
*
*   所有可能的CRC值都被预装在两个数组当中，当计算报文内容时可以简单的索引即可；
*   一个数组包含有16位CRC域的所有256个可能的高位字节，另一个数组含有低位字节的值；
*   这种索引访问CRC的方式提供了比对报文缓冲区的每一个新字符都计算新的CRC更快的方法；
*
*  注意：此程序内部执行高/低CRC字节的交换。此函数返回的是已经经过交换的CRC值；也就是说，该函数的返回值可以直接放置
*        于报文用于发送；
*********************************************************************************************************
*/
uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
    uint8_t ucCRCHi = 0xFF; /* 高CRC字节初始化 */
    uint8_t ucCRCLo = 0xFF; /* 低CRC 字节初始化 */
    uint16_t usIndex;                /* CRC循环中的索引 */

    while (_usLen--)
    {
        usIndex = ucCRCHi ^ *_pBuf++; /* 计算CRC */
        ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
        ucCRCLo = s_CRCLo[usIndex];
    }
    return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

static void MODS_AnalyzeApp(void);
static void MODS_03H(void);
static void MODS_06H(void);
static void MODS_10H(void);

MODS_T g_tModS;

/*
*********************************************************************************************************
*    函 数 名: MODS_Poll
*    功能说明: 解析数据包. 在主程序中轮流调用。
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
uint8_t MODS_Poll(uint8_t *_buf, uint16_t _len)
{
    uint16_t addr = 0;
    uint16_t crc1 = 0;

    g_tModS.RxBuf = _buf;
    g_tModS.RxCount = _len;

    g_tModS.TxCount = 0;
    //*_AckBuf = g_tModS.TxBuf;

    if (g_tModS.RxCount < 4)
    {
        goto err_ret;
    }

    /* 计算CRC校验和 */
    crc1 = CRC16_Modbus(g_tModS.RxBuf, g_tModS.RxCount);
    if (crc1 != 0)
    {
        MODS_SendAckErr(ERR_PACKAGE); /* 发送连包应答 */
        goto err_ret;
    }

    /* 站地址 (1字节） */
    addr = g_tModS.RxBuf[0]; /* 第1字节 站号 */
    if (addr != g_tModS.g_station_address && addr != 0xFF && addr != 0x00)
    {
        goto err_ret;
    }

    /* 分析应用层协议 */
    MODS_AnalyzeApp();
    g_tModS.RxCount = 0; /* 必须清零计数器，方便下次帧同步 */
    return 1;

err_ret:
    g_tModS.RxCount = 0; /* 必须清零计数器，方便下次帧同步 */
    return 0;
}

/*
*********************************************************************************************************
*    函 数 名: MODS_SendWithCRC
*    功能说明: 发送一串数据, 自动追加2字节CRC。数据在全局变量: g_tModS.TxBuf, g_tModS.TxCount
*    形    参: 无
*              _ucLen 数据长度（不带CRC）
*    返 回 值: 无
*********************************************************************************************************
*/
void MODS_SendWithCRC(void)
{
    uint16_t crc = 0;

    crc = CRC16_Modbus(g_tModS.TxBuf, g_tModS.TxCount);
    g_tModS.TxBuf[g_tModS.TxCount++] = crc >> 8;
    g_tModS.TxBuf[g_tModS.TxCount++] = crc;
}

void MODS_SendTimeOut(uint8_t _FCode)
{
    g_tModS.TxCount = 0;
    g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.g_station_address;                /* 485地址 */
    g_tModS.TxBuf[g_tModS.TxCount++] = _FCode | 0x80; /* 异常的功能码 */
    g_tModS.TxBuf[g_tModS.TxCount++] = ERR_TIMEOUT;                            /* 错误代码(01,02,03,04) */

    MODS_SendWithCRC();    
}
/*
*********************************************************************************************************
*    函 数 名: MODS_SendAckErr
*    功能说明: 发送错误应答
*    形    参: _ucErrCode : 错误代码
*    返 回 值: 无
*********************************************************************************************************
*/
void MODS_SendAckErr(uint8_t _ucErrCode)
{
    g_tModS.TxCount = 0;
    g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.g_station_address;                /* 485地址 */
    g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1] | 0x80; /* 异常的功能码 */
    g_tModS.TxBuf[g_tModS.TxCount++] = _ucErrCode;                            /* 错误代码(01,02,03,04) */

    MODS_SendWithCRC();
}

/*
*********************************************************************************************************
*    函 数 名: MODS_SendAckOk
*    功能说明: 发送正确的应答.
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void MODS_SendAckOk(void)
{
    uint8_t i = 0;

    g_tModS.TxCount = 1;
    g_tModS.TxBuf[0] = g_tModS.g_station_address;
    for (i = 1; i < 6; i++)
    {
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[i];
    }
    MODS_SendWithCRC();
}

/*
*********************************************************************************************************
*    函 数 名: MODS_AnalyzeApp
*    功能说明: 分析应用层协议
*    形    参:
*             _DispBuf  存储解析到的显示数据ASCII字符串，0结束
*    返 回 值: 无
*********************************************************************************************************
*/
static void MODS_AnalyzeApp(void)
{
    switch (g_tModS.RxBuf[1]) /* 第2个字节 功能码 */
    {

        case 0x03: /* 读取1个或多个参数保持寄存器 在一个或多个保持寄存器中取得当前的二进制值*/
            MODS_03H();
            break;

        case 0x06: /* 写单个寄存器 */
            MODS_06H();
            break;

        case 0x10: /* 写多个参数保持寄存器 (存储在CPU的FLASH中，或EEPROM中的参数)*/
            MODS_10H();
            break;

        case 0x15: /* 写文件*/
            MODS_15H();
            break;

        default:
            g_tModS.RspCode = RSP_ERR_CMD;
            MODS_SendAckErr(g_tModS.RspCode); /* 告诉主机命令错误 */
            break;
    }
}

/*
*********************************************************************************************************
*    函 数 名: MODS_03H
*    功能说明: 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void MODS_03H(void)
{
    uint16_t regaddr = 0;
    uint16_t num = 0;
    uint16_t value = 0;
    uint16_t i = 0;

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE; /* 数据值域错误 */
        goto err_ret;
    }

    regaddr = BEBufToUint16(&g_tModS.RxBuf[2]);
    num = BEBufToUint16(&g_tModS.RxBuf[4]);
    if (num > (TX_BUF_SIZE - 5) / 2)
    {
        g_tModS.RspCode = RSP_ERR_VALUE; /* 数据值域错误 */
        goto err_ret;
    }

err_ret:
    if (g_tModS.RxBuf[0] != 0x00) /* 00广播地址不应答, FF地址应答g_tParam.Addr485 */
    {
        if (g_tModS.RspCode == RSP_OK) /* 正确应答 */
        {
            g_tModS.TxCount = 0;
            g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.g_station_address;
            g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
            g_tModS.TxBuf[g_tModS.TxCount++] = num * 2;

            for (i = 0; i < num; i++)
            {
                if (ReadRegValue(regaddr++, &value) == 0)
                {
                    g_tModS.RspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
                    goto err_ret;
                }
                g_tModS.TxBuf[g_tModS.TxCount++] = value >> 8;
                g_tModS.TxBuf[g_tModS.TxCount++] = value;
            }

            MODS_SendWithCRC();
        }
        else
        {
            MODS_SendAckErr(g_tModS.RspCode); /* 告诉主机命令错误 */
        }
    }
}

/*
*********************************************************************************************************
*    函 数 名: MODS_06H
*    功能说明: 写单个寄存器
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void MODS_06H(void)
{

    /*
        写保持寄存器。注意06指令只能操作单个保持寄存器，16指令可以设置单个或多个保持寄存器

        主机发送:
            11 从机地址
            06 功能码
            00 寄存器地址高字节
            01 寄存器地址低字节
            00 数据1高字节
            01 数据1低字节
            9A CRC校验高字节
            9B CRC校验低字节

        从机响应:
            11 从机地址
            06 功能码
            00 寄存器地址高字节
            01 寄存器地址低字节
            00 数据1高字节
            01 数据1低字节
            1B CRC校验高字节
            5A    CRC校验低字节

        例子:
            01 06 30 06 00 25  A710    ---- 触发电流设置为 2.5
            01 06 30 06 00 10  6707    ---- 触发电流设置为 1.0


            01 06 30 1B 00 00  F6CD    ---- SMA 滤波系数 = 0 关闭滤波
            01 06 30 1B 00 01  370D    ---- SMA 滤波系数 = 1
            01 06 30 1B 00 02  770C    ---- SMA 滤波系数 = 2
            01 06 30 1B 00 05  36CE    ---- SMA 滤波系数 = 5

            01 06 30 07 00 01  F6CB    ---- 测试模式修改为 T1
            01 06 30 07 00 02  B6CA    ---- 测试模式修改为 T2

            01 06 31 00 00 00  8736    ---- 擦除浪涌记录区
            01 06 31 01 00 00  D6F6    ---- 擦除告警记录区

*/

    uint16_t reg = 0;
    uint16_t value = 0;
    //    uint8_t i;

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE; /* 数据值域错误 */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]);        /* 寄存器号 */
    value = BEBufToUint16(&g_tModS.RxBuf[4]); /* 寄存器值 */

    if(reg >= 0x9100)
    {
        if(IAP_Write06H(reg, value) == 0)
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
        }
    }
    else {
        if (WriteRegValue(reg, value) == 0)
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
        }
    }

err_ret:
    if (g_tModS.RxBuf[0] != 0x00) /* 00广播地址不应答, FF地址应答g_tParam.Addr485 */
    {
        if (g_tModS.RspCode == RSP_OK) /* 正确应答 */
        {
            MODS_SendAckOk();
        }
        else
        {
            MODS_SendAckErr(g_tModS.RspCode); /* 告诉主机命令错误 */
        }
    }
    else
    {
        g_tModS.TxCount = 0;
    }
}

/*
*********************************************************************************************************
*    函 数 名: MODS_10H
*    功能说明: 连续写多个寄存器.  进用于改写时钟
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void MODS_10H(void)
{
    /*
        从机地址为11H。保持寄存器的其实地址为0001H，寄存器的结束地址为0002H。总共访问2个寄存器。
        保持寄存器0001H的内容为000AH，保持寄存器0002H的内容为0102H。

        主机发送:
            11 从机地址
            10 功能码
            00 寄存器起始地址高字节
            01 寄存器起始地址低字节
            00 寄存器数量高字节
            02 寄存器数量低字节
            04 字节数
            00 数据1高字节
            0A 数据1低字节
            01 数据2高字节
            02 数据2低字节
            C6 CRC校验高字节
            F0 CRC校验低字节

        从机响应:
            11 从机地址
            06 功能码
            00 寄存器地址高字节
            01 寄存器地址低字节
            00 数据1高字节
            01 数据1低字节
            1B CRC校验高字节
            5A    CRC校验低字节

        例子:
            01 10 30 00 00 06 0C  07 DE  00 0A  00 01  00 08  00 0C  00 00     389A    ---- 写时钟 2014-10-01 08:12:00
            01 10 30 00 00 06 0C  07 DF  00 01  00 1F  00 17  00 3B  00 39     5549    ---- 写时钟 2015-01-31 23:59:57

    */
    uint16_t reg_addr = 0;
    uint16_t reg_num = 0;
    //    uint8_t byte_num;
    uint16_t value = 0;
    uint8_t i = 0;
    uint8_t *_pBuf = NULL;

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount < 11)
    {
        g_tModS.RspCode = RSP_ERR_VALUE; /* 数据值域错误 */
        goto err_ret;
    }

    //fSaveReq_06H = 0;
    //fSaveCalibParam = 0;
    //fResetReq_06H = 0;

    reg_addr = BEBufToUint16(&g_tModS.RxBuf[2]); /* 寄存器号 */
    reg_num = BEBufToUint16(&g_tModS.RxBuf[4]);    /* 寄存器个数 */
                                                                                             //    byte_num = g_tModS.RxBuf[6];    /* 后面的数据体字节数 */
    _pBuf = &g_tModS.RxBuf[7];

    if(reg_addr >= 0x9100)
    {
        for (i = 0; i < reg_num; i++)
        {
            value = BEBufToUint16(_pBuf);

            if (IAP_Write06H(reg_addr + i, value) == 0)
            {
                g_tModS.RspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
                break;
            }

            _pBuf += 2;
        }
    }
    else {
        for (i = 0; i < reg_num; i++)
        {
            value = BEBufToUint16(_pBuf);

            if (WriteRegValue(reg_addr + i, value) == 0)
            {
                g_tModS.RspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
                break;
            }

            _pBuf += 2;
        }
    }


err_ret:
    if (g_tModS.RxBuf[0] != 0x00) /* 00广播地址不应答, FF地址应答g_tParam.Addr485 */
    {
        if (g_tModS.RspCode == RSP_OK) /* 正确应答 */
        {
            MODS_SendAckOk();
        }
        else
        {
            MODS_SendAckErr(g_tModS.RspCode); /* 告诉主机命令错误 */
        }
    }
    else
    {
        g_tModS.TxCount = 0;
    }
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
