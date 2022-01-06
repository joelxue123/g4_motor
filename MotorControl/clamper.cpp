//
// Created by htkan on 2021/4/26.
//

#include "clamper.h"
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

#define NVM_CLAMPER_DATA_ADDRESS (33)

clamper_param_t clamper_params[8] = {0};

uint8_t _clamper_status = 0;
float _clamper_target_iq = 0.0f;
float _clamper_target_id = 0.0f;

uint8_t _clamper_state = 0;
float _open_limit_point = 0.0f;
float _close_limit_point = 0.0f;

int _tick_time = 0;

uint8_t clamper_vel_set = 0;
uint8_t clamper_vel_fb = 0;
uint8_t clamper_pos_set = 0;
uint8_t clamper_pos_fb = 0;
uint8_t clamper_torque_set = 0;
uint8_t clamper_torque_fb = 0;

uint8_t clamper_last_rACT = 0;
uint8_t clamper_last_rGTO = 0;

uint8_t clamper_gACT = 0;
uint8_t clamper_gDrop = 0;
uint8_t clamper_gSTA = 1;
uint8_t clamper_gOBJ = 0;

void clamper_param_init(void)
{
    int i =0;
    for(i = 0; i< 8; i++)
    {
        uint16_t read_address = NVM_CLAMPER_DATA_ADDRESS + i;
        EE_ReadVariable32bits(read_address, (uint32_t*)clamper_params);
    }
}

void clamper_init(void)
{
    g_motor.init();
    g_optical_encoder.init();
    g_ctrl.reset();

    _clamper_status = 0;
    _clamper_target_iq = 0.0f;
    _clamper_target_id = 0.0f;

    _clamper_state = 0;
    _open_limit_point = 0.0f;
    _close_limit_point = 0.0f;

    _tick_time = 0;

    clamper_vel_set = 255;
    clamper_vel_fb = 0;
    clamper_pos_set = 0;
    clamper_pos_fb = 0;
    clamper_torque_set = 255;
    clamper_torque_fb = 0;

    clamper_param_init();
    //ee24_read(NVM_CLAMPER_DATA_ADDRESS, (uint8_t*)&clamper_params, 8 * sizeof(clamper_param_t), 100);
}

uint8_t clamper_get_status(void)
{
    uint8_t _ret_status = clamper_gACT | (clamper_gDrop << 1);
    _ret_status |= (clamper_gSTA << 3);
    _ret_status |= (clamper_gOBJ << 5);
    return _ret_status;
}

void clamper_on_update(void)
{
    g_motor.update();
    g_optical_encoder.update(g_motor.config_.pole_pairs, 1.0f);

    if(_clamper_status == 1 && g_motor.state_ == Motor::STATE_NORMAL)
    {
        if(g_optical_encoder.calibrate_offset_clamper(g_motor, 1.0f))
        {
            g_motor.servo_on();
            g_ctrl.reset();
            g_ctrl.set_pos_setpoint(0, 0, 0);
            g_ctrl.vel_ramp_target_ = 0.0f;
            _clamper_status = 2;
        }
    }

    if(_clamper_status >= 2)
    {
        float _phase = g_optical_encoder.phase_;
        float _pos_encoder = g_ctrl.pos_estimate;
        g_ctrl.update(g_optical_encoder, _pos_encoder, _clamper_target_iq, g_motor.config_.requested_current_range);
        g_motor.FOC_current(_clamper_target_id, _clamper_target_iq, _phase, 0.0f);

        float abs_vel = fabs(g_optical_encoder.vel_rpm_);
        if(abs_vel > 18500.0f)
        {
            clamper_vel_fb = 255;
        }
        else
        {
            clamper_vel_fb = (uint8_t)(abs_vel / 18500.0f * 255.0f);
        }

        float abs_torque = fabs(g_motor.current_control_.Iq_measured);
        if(abs_torque > 0.6f)
        {
            clamper_torque_fb = 255;
        }
        else
        {
            clamper_torque_fb = (uint8_t)(abs_torque / 0.6f * 255.0f);
        }
    }

    if(_clamper_status >= 6)
    {
        if(g_ctrl.pos_estimate > _open_limit_point)
        {
            clamper_pos_fb = 255;
        }
        else if(g_ctrl.pos_estimate < _close_limit_point)
        {
            clamper_pos_fb = 0;
        }
        else
        {
            clamper_pos_fb = (uint8_t)((g_ctrl.pos_estimate - _close_limit_point) / (_open_limit_point - _close_limit_point) * 255.0f);
        }
    }
}

void clamper_on_main(void)
{
    switch (_clamper_status)
    {
    case 0:
        _clamper_target_iq = 0.0f;
        _clamper_target_id = 0.0f;
        g_motor.init();
        g_optical_encoder.init();
        g_motor.servo_off();
        break;

    case 1:
        g_motor.config_.requested_current_range = 0.6f;
        g_ctrl.trap_.config_.vel_limit = 20000.0f * 6.0f;
        g_ctrl.config_.vel_limit = 20000.0f * 6.0f;
        g_ctrl.trap_.config_.accel_limit = 2000000.0f * 6.0f;
        g_ctrl.trap_.config_.decel_limit = 2000000.0f * 6.0f;

        g_motor.reset_current_control();
        break;

    case 2:
        g_ctrl.reset();
        _tick_time = 0;
        g_ctrl.set_current_setpoint(0.3f);
        _clamper_status = 3;
        break;

    case 3:
        if(g_motor.current_control_.Iq_measured >= 0.1f
           && g_optical_encoder.vel_rpm_ < 10.0f)
        {
            _tick_time++;
        }
        else
        {
            _tick_time = 0;
        }

        if(_tick_time >= 40)
        {
            g_ctrl.set_current_setpoint(0.0f);
            _open_limit_point = g_ctrl.pos_estimate - 800.0f;
            _clamper_status = 4;
        }
        break;
    
    case 4:
        g_ctrl.reset();
        _tick_time = 0;
        g_ctrl.set_current_setpoint(-0.3f);
        _clamper_status = 5;
        break;

    case 5:
        if(g_motor.current_control_.Iq_measured <= -0.1f
           && g_optical_encoder.vel_rpm_ > -10.0f)
        {
            _tick_time++;
        }
        else
        {
            _tick_time = 0;
        }

        if(_tick_time >= 40)
        {
            g_ctrl.set_current_setpoint(0.0f);
            _close_limit_point = g_ctrl.pos_estimate + 800.0f;
            __disable_irq();
            g_ctrl.reset();
            g_motor.reset_current_control();
            g_ctrl.move_to_pos(_open_limit_point,  g_ctrl.pos_estimate);
            _clamper_status = 6;
            __enable_irq();
        }
        break;

    case 6:
        _tick_time = 0;
        break;
    
    case 7:
        clamper_gOBJ = 3;
        if((g_ctrl.mode_ == Controller::CTRL_MODE_POSITION_CONTROL)
            && (g_optical_encoder.vel_abs_rpm_ > 60.0f))
        {
            _clamper_status = 8;
            clamper_gDrop = 1;
        }
        break;

    case 17:
        clamper_gOBJ = 2;
        if((g_ctrl.mode_ == Controller::CTRL_MODE_POSITION_CONTROL)
            && (g_optical_encoder.vel_abs_rpm_ > 60.0f))
        {
            _clamper_status = 18;
        }
        break;

    case 8:
        clamper_gOBJ = 1;
        if(g_ctrl.mode_ == Controller::CTRL_MODE_POSITION_CONTROL
           && g_ctrl.pos_abs_err_ < 120.0f)
        {
            _clamper_status = 9;
        }

        if((g_ctrl.mode_ == Controller::CTRL_MODE_POSITION_CONTROL
            && g_motor.extern_torque > g_motor.config_.requested_current_range * 0.8f)
            && (g_optical_encoder.vel_abs_rpm_ < 60.0f))
        {
            _clamper_status = 7;
            clamper_gDrop = 0;
        }
        break;

    case 18:
        clamper_gOBJ = 1;
        if(g_ctrl.mode_ == Controller::CTRL_MODE_POSITION_CONTROL
           && g_ctrl.pos_abs_err_ < 60.0f)
        {
            _clamper_status = 19;
        }

        if((g_ctrl.mode_ == Controller::CTRL_MODE_POSITION_CONTROL
            && g_motor.extern_torque > g_motor.config_.requested_current_range * 0.8f)
            && (g_optical_encoder.vel_abs_rpm_ < 60.0f))
        {
            _clamper_status = 17;
        }
        break;

    case 9:
        clamper_gOBJ = 5;
        break;

    case 19:
        clamper_gOBJ = 4;
        break;

    case 11:
        g_motor.error_ = Motor::ERROR_NONE;
        _clamper_status = 6;
        break;
    default:
        _clamper_target_iq = 0.0f;
        _clamper_target_id = 0.0f;
        g_motor.servo_off();
        break;
    }

    if(g_motor.error_ != Motor::ERROR_NONE)
    {
        _clamper_status = 10;
    }

    if(_clamper_status < 7)
    {
        clamper_gOBJ = 0;
    }

    if(_clamper_status < 6 && _clamper_status > 0)
    {
        clamper_gSTA = 1;
    }
    else if(_clamper_status == 10 || _clamper_status == 0)
    {
        clamper_gSTA = 0;
    }
    else {
        clamper_gSTA = 3;
    }
}

void _clamper_stop_move(void)
{
    if(_clamper_status > 6)
    {
        __disable_irq();
        g_ctrl.reset();
        g_motor.reset_current_control();
        g_ctrl.set_vel_setpoint(0.0f, 0.0f);
        _clamper_status = 6;
        __enable_irq();
    }
}

void clamper_set_status(uint8_t status)
{
    uint8_t _rACT = status & 0x1;

    uint8_t _rMODE = (status & 0x6) >> 1;
    uint8_t _rGTO = (status & 0x8) >> 3;
    uint8_t _rstop = (status & 0x10) >> 4;

    if(_rACT == 1 && clamper_last_rACT == 0)
    {
        _clamper_status = 1;
    }

    if(_rACT == 0 && clamper_last_rACT == 1)
    {
        _clamper_status = 0;
    }

    if(_rGTO == 1)
    {
        clamper_set_start_move();
    }

    if(_rstop == 1)
    {
        _clamper_stop_move();
    }

    clamper_last_rACT = _rACT;
    clamper_last_rGTO = _rGTO;
}

void clamper_set_vel(uint8_t vel)
{
    clamper_vel_set = vel;
}

void clamper_set_torque(uint8_t torque)
{
    clamper_torque_set = torque;
}

void clamper_set_pos(uint8_t pos)
{
    clamper_pos_set = 0xFF - pos;
}

void clamper_set_start_move(void)
{
    if(_clamper_status >= 6)
    {
        clamper_gDrop = 0;
        clamper_gOBJ = 1;
        g_motor.config_.requested_current_range = (float)clamper_torque_set / 255.0f * 0.45f + 0.15f;
        g_ctrl.trap_.config_.vel_limit = (float)clamper_vel_set / 255.0f * 17500.0f * 6.0f + 1000.0f * 6.0f;
        g_ctrl.config_.vel_limit = (float) clamper_vel_set / 255.0f * 17500.0f * 6.0f + 1000.0f * 6.0f;
        g_ctrl.trap_.config_.accel_limit = (float)clamper_torque_set / 255.0f * 1900000.0f * 6.0f + 100000.0f * 6.0f;
        g_ctrl.trap_.config_.decel_limit = (float)clamper_torque_set / 255.0f * 1900000.0f * 6.0f + 100000.0f * 6.0f;
        float pos_ = (float)clamper_pos_set / 255.0f * (_open_limit_point - _close_limit_point) + _close_limit_point;
        float cur_ = g_ctrl.pos_estimate;
        __disable_irq();
        g_ctrl.reset();
        g_motor.reset_current_control();
        g_ctrl.move_to_pos(pos_, g_ctrl.pos_estimate);
        if(cur_ > pos_)
        {
            _clamper_status = 8;
        }
        else {
            _clamper_status = 18;
        }
        __enable_irq();
    }
}

uint8_t clamper_get_vel(void)
{
    return clamper_vel_fb;
}

uint8_t clamper_get_torque(void)
{
    return clamper_torque_fb;
}

uint8_t clamper_get_pos(void)
{
    return (0xFF - clamper_pos_fb);
}

void clamper_set_mode(uint8_t mode)
{
    if(mode > 0x7 && mode < 0x10)
    {
        if(clamper_params[mode - 8].is_used == 0x5A)
        {
            clamper_pos_set = 0xFF - clamper_params[mode - 8].pos_;
            clamper_vel_set = clamper_params[mode - 8].vel_;
            clamper_torque_set = clamper_params[mode - 8].tor_;
        }
        return;
    }

    switch (mode) {
    case 0x1:
        clamper_pos_set = 0xff;
        clamper_vel_set = 0x80;
        clamper_torque_set = 0x80;
        break;
    case 0x2:
        clamper_pos_set = 0x00;
        clamper_vel_set = 0x80;
        clamper_torque_set = 0x80;
        break;
    case 0x3:
        clamper_pos_set = 0xff;
        clamper_vel_set = 0xFF;
        clamper_torque_set = 0xFF;
        break;
    case 0x4:
        clamper_pos_set = 0x00;
        clamper_vel_set = 0xFF;
        clamper_torque_set = 0xFF;
        break;
    case 0x5:
        clamper_pos_set = 0xff;
        clamper_vel_set = 0x10;
        clamper_torque_set = 0x10;
        break;
    case 0x6:
        clamper_pos_set = 0x00;
        clamper_vel_set = 0x10;
        clamper_torque_set = 0x10;
        break;
    default:
        break;
    }
}

void clamper_set_param(char type, uint16_t address, uint8_t data)
{
    switch (type) {
    case 0:
        {
            clamper_params[address].pos_ = data;
            break;
        }
    case 1:
        {
            clamper_params[address].vel_ = data;
            break;
        }
    case 2:
        {
            clamper_params[address].tor_ = data;
            break;
        }
    case 3:
        {
            if(data == 1)
            {
                clamper_params[address].is_used = 0x5A;
                uint16_t write_address = NVM_CLAMPER_DATA_ADDRESS + address;
                HAL_FLASH_Unlock(); 
                EE_WriteVariable32bits(write_address, (*(uint32_t *)clamper_params)); 
                HAL_FLASH_Lock();
                //ee24_write(write_address, (uint8_t *)&clamper_params[address], sizeof(clamper_param_t), 100);
            }
            break;
        }
    default:
        break;
    }
}

uint16_t clamper_get_param(char type, uint16_t address)
{
    switch (type) {
    case 0:
        {
            return clamper_params[address].pos_;
        }
    case 1:
        {
            return clamper_params[address].vel_;
        }
    case 2:
        {
            return clamper_params[address].tor_;
        }
    default:
        return 0;
    }
}

int8_t clamper_spi_get_vel(void)
{
    float _vel = g_optical_encoder.vel_rpm_;
    int8_t _fb = 0;
    if(_vel > 18000.0f)
    {
        _fb = 127;
    }
    else if(_vel < -18000.0f) 
    {
        _fb = -127;
    }
    else
    {
        _fb = (int8_t)(_vel / 18000.0f * 127.0f);
    }
    return _fb;
}

int8_t clamper_spi_get_torque(void)
{
    float _torque = g_motor.current_control_.Iq_measured;
    int8_t _fb = 0;
    if(_torque > 0.6f)
    {
        _fb = 127;
    }
    else if(_torque < -0.6f) 
    {
        _fb = -127;
    }
    else
    {
        _fb = (int8_t)(_torque / 0.6f * 127.0f);
    }
    return _fb;
}

int32_t clamper_spi_get_pos(void)
{
    int32_t pos = 0;
    if(g_ctrl.pos_estimate > _open_limit_point)
    {
        pos = 65535;
    }
    else if(g_ctrl.pos_estimate < _close_limit_point)
    {
        pos = 0;
    }
    else
    {
        pos = (int32_t)((g_ctrl.pos_estimate - _close_limit_point) / (_open_limit_point - _close_limit_point) * 65535.0f);
    }
    
    return pos;
}

void clamper_spi_set_vel(uint8_t vel)
{
    //g_ctrl.config_.vel_limit = 18500.0f;
    //if(_clamper_status >= 6)
    //{
    //    g_ctrl.config_.vel_limit = (float) clamper_vel_set / 255.0f * 17500.0f * 6.0f + 1000.0f * 6.0f;
    //}
}

void clamper_spi_set_torque(uint8_t torque)
{
    //g_motor.config_.requested_current_range = 0.6f;
    //if(_clamper_status >= 6)
    //{
    //    g_motor.config_.requested_current_range = (float)torque / 255.0f * 0.45f + 0.15f;
    //}
}

void clamper_spi_set_pos(int32_t pos)
{
    if(_clamper_status >= 6)
    {
        if(pos < 0) pos = 0;
        if(pos > 65535) pos = 65535;

        float pos_ = (float)pos / 65535.0f * (_open_limit_point - _close_limit_point) + _close_limit_point;
        g_ctrl.set_pos_setpoint(pos_, 0.0f, 0.0f);
    }
}