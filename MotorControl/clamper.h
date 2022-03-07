//
// Created by htkan on 2021/4/26.
//

#ifndef STM32_BOARD_CLAMPER_H
#define STM32_BOARD_CLAMPER_H
#include <stdint.h>
/*
*** 夹取管理类
*/

void clamper_init(void);

void clamper_on_update(void);



void clamper_on_main(void);

uint8_t clamper_get_vel(void);
uint8_t clamper_get_torque(void);
uint8_t clamper_get_pos(void);

uint8_t clamper_spi_get_vel(void);
uint8_t clamper_spi_get_torque(void);
uint8_t clamper_spi_get_pos(void);

uint8_t clamper_get_status(void);


void clamper_spi_set_vel(uint8_t vel);
void clamper_spi_set_torque(uint8_t torque);
void clamper_spi_set_pos(int32_t pos);

void clamper_set_status(uint8_t status);
void clamper_set_vel(uint8_t vel);
void clamper_set_torque(uint8_t torque);
void clamper_set_pos(uint8_t pos);
void clamper_set_start_move(void);
void clamper_set_mode(uint8_t mode);

typedef struct 
{
    uint8_t is_used;
    uint8_t pos_;
    uint8_t vel_;
    uint8_t tor_;
} clamper_param_t; 

void clamper_set_param(char type, uint16_t address, uint8_t data);
uint16_t clamper_get_param(char type, uint16_t address);

#endif //STM32_BOARD_CLAMPER_H
