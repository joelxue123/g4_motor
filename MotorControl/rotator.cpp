//
// Created by htkan on 2021/4/26.
//

#include "rotator.h"
#include "interface.h"
#include "motor.hpp"
#include "controller.hpp"
#include "optical_encoder.hpp"
#include "global.hpp"
#include <math.h>
#include "stm32g431xx.h"

extern "C"
{
    #include "eeprom_emul.h"
}

#define ROTATOR_REDUCER_RATIO    (28728.0f)
#define NVM_ROTATOR_DATA_ADDRESS (33)

rotator_param_t rotator_params[8] = {0};

float _rotator_turn_len = ROTATOR_REDUCER_RATIO;
float _rotator_turn_pos = 0.0f;
float _rotator_pos_encode = 0.0f;
float _rotator_turn_err = 0.0f;
int _rotator_status = 0;
float _rotator_target_iq = 0.0f;
float _rotator_target_id = 0.0f;

float _rotator_zero_speed = 0.0f;
float left_rotator_zero_encoder = 0;
float right_rotator_zero_encoder = 0;
float zero_encoder_offset = 0;

float rotator_relative_zero = 0.0f;
int _rotator_tick_time = 0;

uint8_t rotator_vel_set = 0;
uint8_t rotator_vel_fb = 0;
int8_t rotator_pos_set = 0;
uint8_t rotator_pos_fb = 0;
uint8_t rotator_torque_set = 0;
uint8_t rotator_torque_fb = 0;
int16_t rotator_turn_set = 0;
int16_t rotator_turn_relative_set = 0;
int16_t rotator_turn_relative_get = 0;
int16_t rotator_turn_fb = 0;

bool    is_relative_mode = false;
uint8_t rotator_last_rACT = 0;
uint8_t rotator_last_rGTO = 0;

uint8_t rotator_gACT = 0;
uint8_t rotator_gSTA = 0;
uint8_t rotator_gOBJ = 0;
uint8_t rotator_dir = 0;

float rotator_point_falling = 0.0f;
bool  rotator_falling_triged = true;

int32_t rotator_real_turn = 0;



void rotator_set_turn(int16_t turn)
{
    rotator_turn_set = turn;
}

void rotator_param_init(void)
{
    int i =0;
    for(i = 0; i< 8; i++)
    {
        uint16_t read_address = NVM_ROTATOR_DATA_ADDRESS + i;
        EE_ReadVariable32bits(read_address, (uint32_t*)rotator_params);
    }
}

void rotator_init(void)
{
    g_motor.init();
    g_optical_encoder.init();
    g_ctrl.reset();

    _rotator_status = 0;
    _rotator_target_iq = 0.0f;
    _rotator_target_id = 0.0f;

    left_rotator_zero_encoder = 0;
    right_rotator_zero_encoder = 0;
    _rotator_tick_time = 0;

    rotator_vel_set = 255;
    rotator_vel_fb = 0;
    rotator_pos_set = 0;
    rotator_pos_fb = 0;
    rotator_turn_fb = 0;
    rotator_torque_set = 255;
    rotator_torque_fb = 0;
    rotator_turn_set = 0;

    //ee24_read(NVM_ROTATOR_DATA_ADDRESS, (uint8_t*)&rotator_params, 8 * sizeof(rotator_param_t), 100);
    rotator_param_init();
    // 将旋转零点位置偏差读出
    zero_encoder_offset = rotator_get_offset();
    _rotator_turn_len = ROTATOR_REDUCER_RATIO;
}

uint8_t rotator_get_status(void)
{
    return (rotator_gACT | (rotator_gSTA << 3) | (rotator_gOBJ << 5));
}

float _last_pos = 0.0f;
bool _last_zero_flag = true;

void rotator_on_update(void)
{
    g_motor.update();
    g_optical_encoder.update(g_motor.config_.pole_pairs, 1.0f);

    float pos_inc_ = g_optical_encoder.pos_estimate_degree - _last_pos;
    float cur_pos =  _rotator_turn_pos + pos_inc_;

    if(cur_pos < -(zero_encoder_offset / 360.0f * _rotator_turn_len))
    {
        rotator_real_turn--;
        cur_pos = _rotator_turn_len + cur_pos;
    }

    if(cur_pos > _rotator_turn_len - (zero_encoder_offset / 360.0f * _rotator_turn_len))
    {
        rotator_real_turn++;
        cur_pos = cur_pos - _rotator_turn_len;
    }
    
    _rotator_turn_pos = cur_pos;
    _last_pos = g_optical_encoder.pos_estimate_degree;
    _rotator_pos_encode = _rotator_turn_pos + rotator_real_turn * _rotator_turn_len;

    if(_rotator_status == 1 && g_motor.state_ == Motor::STATE_NORMAL)
    {
        if(g_optical_encoder.calibrate_offset_rotator(g_motor, 1.0f))
        {
            g_motor.servo_on();
            g_ctrl.reset();
            g_ctrl.set_pos_setpoint(0, 0, 0);
            g_ctrl.vel_ramp_target_ = 0.0f;
            _rotator_status = 2;
        }
    }

    if(_rotator_status >= 2)
    {
        float _phase = g_optical_encoder.phase_;       
        g_ctrl.update(g_optical_encoder, _rotator_pos_encode, _rotator_target_iq, g_motor.config_.requested_current_range);
        g_motor.FOC_current(_rotator_target_id, _rotator_target_iq, _phase, 0.0f);

        float abs_vel = fabsf(g_optical_encoder.vel_rpm_);
        if(abs_vel > 18000.0f)
        {
            rotator_vel_fb = 255;
        }
        else
        {
            rotator_vel_fb = (uint8_t)(abs_vel / 18000.0f * 255.0f);
        }

        if(g_motor.extern_torque > 1.1f)
        {
            rotator_torque_fb = 255;
        }
        else
        {
            rotator_torque_fb = (uint8_t)(g_motor.extern_torque / 1.1f * 255.0f);
        }
    }

    if(_rotator_status >= 6)
    {
        float rel_dis = _rotator_turn_pos;
        
        rotator_pos_fb = rotator_real_turn;

        rotator_turn_fb = (int16_t)((rel_dis / _rotator_turn_len + rotator_real_turn) * 360.0f);

        if(is_relative_mode)
        {
            float rel_dis_1 = (g_optical_encoder.pos_estimate_degree - rotator_relative_zero);
            rotator_turn_relative_get = (int16_t)((rel_dis_1 / _rotator_turn_len) * 360.0f);
        }
    }
}

void rotator_on_main(void)
{
    switch (_rotator_status)
    {
    case 0:
        rotator_real_turn = 0;
        _rotator_target_iq = 0.0f;
        _rotator_target_id = 0.0f;
        g_motor.init();
        g_motor.servo_off();
        g_optical_encoder.init();
        break;

    case 1:
        g_motor.config_.requested_current_range = 1.0f;
        g_ctrl.trap_.config_.vel_limit = 5000.0f * 6.0f;
        g_ctrl.config_.vel_limit = 5000.0f * 6.0f;
        g_ctrl.trap_.config_.accel_limit = 1000000.0f * 6.0f + 20000.0f * 6.0f;
        g_ctrl.trap_.config_.decel_limit = 1000000.0f * 6.0f + 20000.0f * 6.0f;

        g_motor.reset_current_control();
        _rotator_zero_speed = -6 * 5000.0f;
        break;

    case 2:
        g_ctrl.reset();
        g_ctrl.set_vel_setpoint(_rotator_zero_speed, 0.0f);
        _rotator_status = 5;
        break;

    case 3:
        break;

    case 4:
        break;

    case 5:
        g_ctrl.set_vel_setpoint(0.0f, 0.0f);     
        left_rotator_zero_encoder = 0; 
        right_rotator_zero_encoder = ROTATOR_REDUCER_RATIO;
        
        __disable_irq();
        rotator_real_turn = 0;
        _rotator_turn_pos =  -(zero_encoder_offset / 360.0f * _rotator_turn_len);
        g_ctrl.reset();
        g_motor.reset_current_control();
        g_ctrl.move_to_pos(0.0f, _rotator_turn_pos);
        _rotator_status = 6;
        __enable_irq();   
        break;
    case 6:
        break;
    case 7:
        if(rotator_dir)
        {
            rotator_gOBJ = 2;
        }
        else {
            rotator_gOBJ = 3;
        }
        break;
    case 9:
        if(rotator_dir)
        {
            rotator_gOBJ = 4;
        }
        else {
            rotator_gOBJ = 5;
        }
        break;
    case 8:
        rotator_gOBJ = 1;
        if(g_ctrl.mode_ == Controller::CTRL_MODE_POSITION_CONTROL
           && g_ctrl.pos_abs_err_ < 120.0f)
        {
            _rotator_status = 9;
        }

        if((g_motor.extern_torque > g_motor.config_.requested_current_range * 0.8f)
            && (g_optical_encoder.vel_abs_rpm_ < 60.0f))
        {
            g_ctrl.set_pos_setpoint(_rotator_pos_encode, 0.0f, 0.0f);
            _rotator_status = 7;
        }
        break;
    case 11:
        g_motor.error_ = Motor::ERROR_NONE;
        _rotator_status = 4;
        break;
    default:
        g_motor.servo_off();
        break;
    }
    
    if(g_motor.error_ != Motor::ERROR_NONE)
    {
        _rotator_status = 10;
    }

    if(_rotator_status < 7)
    {
        rotator_gOBJ = 0;
    }

    if(_rotator_status < 6 && _rotator_status > 0)
    {
        rotator_gSTA = 1;
    }
    else if(_rotator_status == 10 || _rotator_status == 0)
    {
        rotator_gSTA = 0;
    }
    else {
        rotator_gSTA = 3;
    }
}

uint8_t rotator_get_vel(void)
{
    return rotator_vel_fb;
}

uint8_t rotator_get_torque(void)
{
    return rotator_torque_fb;
}

int8_t rotator_get_pos(void)
{
    return rotator_pos_fb;
}

int16_t rotator_get_turn(void)
{
    return rotator_turn_fb;
}

void rotator_set_vel(uint8_t vel)
{
    rotator_vel_set = vel;
}

void rotator_set_torque(uint8_t torque)
{
    rotator_torque_set = torque;
}

void rotator_set_pos(int8_t pos)
{
    rotator_pos_set = pos;
}

void _rotator_stop_move(void)
{
    if( _rotator_status > 6)
    {
        __disable_irq();
        g_ctrl.reset();
        g_motor.reset_current_control();
        g_ctrl.set_vel_setpoint(0.0f, 0.0f);
        _rotator_status = 6;
        __enable_irq();
    }
}

void rotator_set_status(uint8_t status)
{
    uint8_t _rACT = status & 0x1;
    uint8_t _rGTO = (status & 0x8) >> 3;
    uint8_t _rstop = (status & 0x10) >> 4;

    if(rotator_last_rACT == 0 && _rACT == 1)
    {
        _rotator_status = 1;
    }

    if(_rACT == 0 && rotator_last_rACT == 1)
    {
        _rotator_status = 0;
    }

    if(_rGTO)
    {
        rotator_set_start_move(2);
    }

    if(_rstop)
    {
        _rotator_stop_move();
    }

    rotator_last_rACT = _rACT;
    rotator_last_rGTO = _rGTO;
}

void rotator_set_start_move(uint8_t mode)
{
    if(mode > 2 || mode < 1)
    {
        return;
    }

    if(_rotator_status >= 6)
    {
        rotator_gOBJ = 1;
        g_motor.config_.requested_current_range = (float)rotator_torque_set / 255.0f * 1.0f + 0.15f;
        g_ctrl.trap_.config_.vel_limit = (float)rotator_vel_set / 255.0f * 9500.0f * 6.0f + 500.0f * 6.0f;
        g_ctrl.config_.vel_limit = (float)rotator_vel_set / 255.0f * 9500.0f * 6.0f + 500.0f * 6.0f;
        g_ctrl.trap_.config_.accel_limit = (float)rotator_torque_set / 255.0f * 180000.0f * 6.0f + 20000.0f * 6.0f;
        g_ctrl.trap_.config_.decel_limit = (float)rotator_torque_set / 255.0f * 180000.0f * 6.0f + 20000.0f * 6.0f;
        //float dis_all = left_rotator_zero_encoder - right_rotator_zero_encoder;
        float pos = 0.0f;
        if(mode == 1)
        {
            float turn_pos = rotator_pos_set * _rotator_turn_len;
            pos = (float)rotator_turn_set / 360.0f * _rotator_turn_len + turn_pos;
            float cur = _rotator_pos_encode;
            if(cur > pos)
            {
                rotator_dir = 0;
            }
            else {
                rotator_dir = 1;
            }

            is_relative_mode = false;
        }
        else {
            rotator_relative_zero = _rotator_pos_encode;
            pos = rotator_relative_zero + (float)rotator_turn_relative_set / 360.0f * _rotator_turn_len;
            if(rotator_turn_relative_set < 0)
            {
                rotator_dir = 0;
            }
            else {
                rotator_dir = 1;
            }
            is_relative_mode = true;
        }

        __disable_irq();
        g_ctrl.reset();
        g_motor.reset_current_control();
        g_ctrl.move_to_pos(pos, _rotator_pos_encode);
        _rotator_status = 8;
        __enable_irq();
    }
}

void rotator_set_mode(uint8_t mode)
{
    if(mode > 0x7 && mode < 0x10)
    {
        if(rotator_params[mode - 8].is_used == 0x5A)
        {
            rotator_turn_relative_set = rotator_params[mode - 8].pos_;
            rotator_torque_set = rotator_params[mode - 8].tor_;
            rotator_vel_set = rotator_params[mode - 8].vel_;
        }
        return;
    }

    switch (mode) {
    case 1:
        rotator_turn_relative_set = 360;
        rotator_torque_set = 0x80;
        rotator_vel_set = 0x80;
        break;
    case 2:
        rotator_turn_relative_set = -360;
        rotator_torque_set = 0x80;
        rotator_vel_set = 0x80;
        break;
    case 3:
        rotator_turn_relative_set = 360;
        rotator_torque_set = 0xFF;
        rotator_vel_set = 0xFF;
        break;
    case 4:
        rotator_turn_relative_set = -360;
        rotator_torque_set = 0xFF;
        rotator_vel_set = 0xFF;
        break;
    case 5:
        rotator_turn_relative_set = 360;
        rotator_torque_set = 0x20;
        rotator_vel_set = 0x20;
        break;
    case 6:
        rotator_turn_relative_set = -360;
        rotator_torque_set = 0x20;
        rotator_vel_set = 0x20;
        break;
    default:
        break;;
    }
}

void rotator_set_relative_turn(int16_t turn)
{
    rotator_turn_relative_set = turn;
}

int16_t rotator_get_relative_turn(void)
{
    return rotator_turn_relative_get;
}

uint8_t rotator_get_relative_mode()
{
    if(is_relative_mode)
    {
        return 2;
    }
    else {
        return 1;
    }
}

void rotator_set_param(char type, uint16_t address, int16_t data)
{
    switch (type) {
    case 0:
        {
            rotator_params[address].pos_ = data;
            break;
        }
    case 1:
        {
            rotator_params[address].vel_ = data;
            break;
        }
    case 2:
        {
            rotator_params[address].tor_ = data;
            break;
        }
    case 3:
        {
            if(data == 1)
            {
                rotator_params[address].is_used = 0x5A;
                uint16_t write_address = NVM_ROTATOR_DATA_ADDRESS + address * sizeof(rotator_param_t);
                //ee24_write(write_address, (uint8_t *)&rotator_params[address], sizeof(rotator_param_t), 100);
                HAL_FLASH_Unlock(); 
                EE_WriteVariable32bits(write_address, (*(uint32_t *)rotator_params)); 
                HAL_FLASH_Lock();
            }
        }
    default:
        break;
    }
}

uint16_t rotator_get_param(char type, uint16_t address)
{
    switch (type) {
    case 0:
        {
            return rotator_params[address].pos_;
        }
    case 1:
        {
            return rotator_params[address].vel_;
        }
    case 2:
        {
            return rotator_params[address].tor_;
        }
    default:
        return 0;
    }
}

void  rotator_set_offset(uint16_t offset)
{
    zero_encoder_offset = 0.0f;
}

float rotator_get_offset(void)
{
    return 0.0f;
}

int8_t rotator_spi_get_vel(void)
{
    int8_t vel = 0;
    float _vel = g_optical_encoder.vel_rpm_;
    if(_vel > 18000.0f)
    {
        vel = 127;
    }
    else if(_vel < -18000.0f)
    {
        vel = -127;
    }
    else
    {
        vel = (int8_t)(_vel / 18000.0f * 127.0f);
    }
    return vel;
}

int8_t rotator_spi_get_torque(void)
{
    int8_t tor = 0;
    float _tor = g_motor.current_control_.Iq_measured;
    if(_tor > 1.2f)
    {
        tor = 127;
    }
    else if(_tor < -1.2f)
    {
        tor = -127;
    }
    else
    {
        tor = (int8_t)(_tor / 1.2f * 127.0f);
    }
    return tor;
}

int32_t rotator_spi_get_pos(void)
{
    int32_t _fb = (int32_t)((_rotator_turn_pos / _rotator_turn_len + rotator_real_turn) * 360.0f);
    return _fb;
}

void rotator_spi_set_vel(uint8_t vel)
{
    //g_ctrl.config_.vel_limit = 18000.0f;
    //if(_rotator_status >= 6)
    //{
    //    g_ctrl.config_.vel_limit = (float)vel / 255.0f * 17500.0f * 6.0f + 500.0f * 6.0f;
    //}
}

void rotator_spi_set_torque(uint8_t torque)
{
    //g_motor.config_.requested_current_range = 1.2f;
    //if(_rotator_status >= 6)
    //{
    //    g_motor.config_.requested_current_range = (float)torque / 255.0f * 1.0f + 0.15f;
    //}
}

void rotator_spi_set_pos(int32_t pos)
{
    if(_rotator_status >= 6)
    {
        float pos_ = (float)pos / 360.0f * _rotator_turn_len;
        g_ctrl.set_pos_setpoint(pos_, 0.0f, 0.0f);
    }
}
