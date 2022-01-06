//
// Created by htkan on 2021/4/26.
//

#ifndef STM32_BOARD_ROTATOR_H
#define STM32_BOARD_ROTATOR_H
#include <stdint.h>
/*
*** 旋转管理类
*/
void rotator_init(void);

void rotator_on_update(void);

uint8_t rotator_get_status(void);

void rotator_on_main(void);

int8_t rotator_spi_get_vel(void);
int8_t rotator_spi_get_torque(void);
int32_t rotator_spi_get_pos(void);

void rotator_spi_set_vel(uint8_t vel);
void rotator_spi_set_torque(uint8_t torque);
void rotator_spi_set_pos(int32_t pos);

uint8_t rotator_get_vel(void);

uint8_t rotator_get_torque(void);

int8_t rotator_get_pos(void);

int16_t rotator_get_turn(void);
int16_t rotator_get_relative_turn(void);
uint8_t rotator_get_relative_mode();

void rotator_set_status(uint8_t status);
void rotator_set_vel(uint8_t vel);
void rotator_set_torque(uint8_t torque);
void rotator_set_turn(int16_t turn);
void rotator_set_pos(int8_t pos);
void rotator_set_mode(uint8_t mode);
void rotator_set_start_move(uint8_t mode);
void rotator_set_mode(uint8_t mode);
void rotator_set_relative_turn(int16_t turn);

typedef struct 
{
    uint8_t is_used;
    int16_t pos_;
    uint8_t vel_;
    uint8_t tor_;
} rotator_param_t; 

void rotator_set_param(char type, uint16_t address, int16_t data);
uint16_t rotator_get_param(char type, uint16_t address);

void  rotator_set_offset(uint16_t offset);
float rotator_get_offset(void);
#endif //STM32_BOARD_ROTATOR_H
