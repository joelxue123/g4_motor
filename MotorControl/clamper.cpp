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
}

uint8_t clamper_get_status(void)
{
    uint8_t _ret_status = _clamper_status;
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
        g_ctrl.config_.vel_limit = 9000.0f * 6.0f;

        g_motor.reset_current_control();
        break;

    case 2:
        g_ctrl.reset();
        _tick_time = 0;
        g_ctrl.set_vel_setpoint(5000.0f, 0.3f);
        _clamper_status = 3;
        break;

    case 3:
        if(g_motor.current_control_.Iq_measured >= 0.1f
           && g_optical_encoder.vel_rpm_ < 100.0f)
        {
            _tick_time++;
        }
        else
        {
            _tick_time = 0;
        }

        if(_tick_time >= 40)
        {
            _open_limit_point = g_ctrl.pos_estimate - 200.0f;
            _clamper_status = 4;
        }
        break;
    
    case 4:
        g_ctrl.reset();
        _tick_time = 0;
        g_ctrl.set_vel_setpoint(-5000.0f, -0.3f);
        _clamper_status = 5;
        break;

    case 5:
        if(g_motor.current_control_.Iq_measured <= -0.1f
           && g_optical_encoder.vel_rpm_ > -60.0f)
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
            _close_limit_point = g_ctrl.pos_estimate + 200.0f;
            __disable_irq();
            g_ctrl.reset();
            g_motor.reset_current_control();
            g_ctrl.set_pos_setpoint(_open_limit_point, 0.0f, 0.0f);
            _clamper_status = 6;
            __enable_irq();
        }
        break;

    case 6:
        _tick_time = 0;
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

    if(_rstop == 1)
    {
        _clamper_stop_move();
    }

    clamper_last_rACT = _rACT;
    clamper_last_rGTO = _rGTO;
}

int8_t clamper_spi_get_vel(void)
{
    float _vel = g_optical_encoder.vel_abs_rpm_;
    uint8_t _fb = 0;
    if(_vel > 10000.0f)
    {
        _fb = 255;
    }
    else 
    {
        _fb = (uint8_t)(_vel / 18000.0f * 255.0f);
    }
    return _fb;
}

int8_t clamper_spi_get_torque(void)
{
    float _torque = g_motor.extern_torque;
    uint8_t _fb = 0;
    if(_torque > 0.5f)
    {
        _fb = 255;
    }
    else
    {
        _fb = (uint8_t)(_torque / 0.55f * 255.0f);
    }
    return _fb;
}

int32_t clamper_spi_get_pos(void)
{
    int32_t pos = 0;
    if(g_ctrl.pos_estimate > _open_limit_point)
    {
        pos = 255;
    }
    else if(g_ctrl.pos_estimate < _close_limit_point)
    {
        pos = 0;
    }
    else
    {
        pos = (int32_t)((g_ctrl.pos_estimate - _close_limit_point) / (_open_limit_point - _close_limit_point) * 65535.0f);
    }
    pos = 255 - pos;
    return pos;
}

void clamper_spi_set_vel(uint8_t vel)
{
    if(_clamper_status >= 6)
    {
        g_ctrl.config_.vel_limit = (float) vel / 255.0f * 8500.0f * 6.0f + 500.0f * 6.0f;
    }
}

void clamper_spi_set_torque(uint8_t torque)
{
    if(_clamper_status >= 6)
    {
        g_motor.config_.requested_current_range = (float)torque / 255.0f * 0.4f + 0.15f;
    }
}

void clamper_spi_set_pos(int32_t pos)
{
    if(_clamper_status >= 6)
    {
        if(pos < 0) pos = 0;
        if(pos > 255) pos = 255;
        pos = 255 - pos;
        float pos_ = (float)pos / 255.0f * (_open_limit_point - _close_limit_point) + _close_limit_point;
        g_ctrl.set_pos_setpoint(pos_, 0.0f, 0.0f);
    }
}