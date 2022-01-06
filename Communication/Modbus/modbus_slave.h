/*
*********************************************************************************************************
*
*    模块名称 : MODBUS从站通信模块
*    文件名称 : modbus_slave.h
*    版    本 : V1.0
*    说    明 : 头文件
*
*    Copyright (C), 2019-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H
#include <stdint.h>
/* RTU 应答代码 */
#define RSP_OK              0           /* 成功 */
#define RSP_ERR_CMD         0x01        /* 不支持的功能码 */
#define RSP_ERR_REG_ADDR    0x02        /* 寄存器地址错误 */
#define RSP_ERR_VALUE       0x03        /* 数据值域错误 */
#define RSP_ERR_WRITE       0x04        /* 写入失败 */

#define ERR_PACKAGE         0x05        /* 自己定义错误包应答 */
#define ERR_TIMEOUT         0x06        /* 超时 自定义的 */

#define RX_BUF_SIZE (256)
#define TX_BUF_SIZE (256)

typedef struct
{
    uint8_t *RxBuf;
    uint16_t RxCount;
    uint8_t RxStatus;
    uint8_t RxNewFlag;

    uint8_t RspCode;

    uint8_t TxBuf[TX_BUF_SIZE];
    uint16_t TxCount;

    uint8_t g_station_address;
} MODS_T;

uint16_t LEBufToUint16(uint8_t *_pBuf);
uint16_t BEBufToUint16(uint8_t *_pBuf);

void MODS_SetAddress(uint8_t addr_);
uint8_t MODS_Poll(uint8_t *_buf, uint16_t _len);
void MODS_SendAckErr(uint8_t _ucErrCode);
void MODS_SendWithCRC(void);
void MODS_SendAckOk(void);
void MODS_SendTimeOut(uint8_t _FCode);
extern MODS_T g_tModS;

#define MODS_DEFAULT_ADDRESS 0x09
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
