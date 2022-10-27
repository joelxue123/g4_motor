#include "optical_encoder.hpp"
#include "utils.hpp"
#include "motor.hpp"
#include <cmath>
#include <math.h>

OpticalEncoder::OpticalEncoder(optical_encoder_config_t& _config)
    : config_(_config)
{
    hfi_state.hfi_step = 1;
}

bool OpticalEncoder::init()
{
    error_ = ERROR_NONE;
    index_found_ = false;
    shadow_count_ = g_encoder_value_org;
    count_in_cpr_ = g_encoder_value_org;
    turn_ = 0;
    interpolation_ = 0.0f;
    phase_ = 0.0f;    // [count]
    pos_estimate_ = 0.0f;  // [count]
    pos_cpr_ = g_encoder_value_org;  // [count]
    vel_estimate_ = g_encoder_value_org;  // [count/s]
    pll_kp_ = 0.0f;   // [count/s / count]
    pll_ki_ = 0.0f;   // [(count/s^2) / count]
    vel_rpm_ = 0.0f;
    pos_degree_ = 0.0f;
    hfi_state.hfi_step = 1;
    update_pll_gains();
    is_ready_ = true;
    return true;
}

void OpticalEncoder::set_error(Error_t _error)
{
    error_ = _error;
}

void OpticalEncoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

bool OpticalEncoder::update(int motor_pole_pairs, float __delta)
{
    if(!is_ready_) return false;
    int32_t delta_enc = 0;
    int __cpr = config_.cpr;

    if(config_.num == 0)
    {
        delta_enc = ( g_encoder_value_org) - shadow_count_;
    }
    else
    {
        delta_enc = (g_encoder1_value_org) - shadow_count_;
    }

    if(delta_enc > (__cpr / 2))
    {
        turn_--;
    }

    if(delta_enc < -(__cpr / 2))
    {
        turn_++;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = count_in_cpr_ % __cpr;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_ += current_meas_period * vel_estimate_ * __delta;
    pos_cpr_      += current_meas_period * vel_estimate_ * __delta;
    // discrete phase detector
    float delta_pos     = (float)(shadow_count_ - (int32_t)floor(pos_estimate_));
    float delta_pos_cpr = (float)(count_in_cpr_ - (int32_t)floor(pos_cpr_));
    delta_pos_cpr = wrap_pm(delta_pos_cpr, 0.5f * (float)(config_.cpr));
    // pll feedback
    pos_estimate_ += current_meas_period * pll_kp_ * delta_pos  * __delta;
    pos_cpr_  += current_meas_period * pll_kp_ * delta_pos_cpr * __delta;
    pos_cpr_ = fmodf_pos(pos_cpr_, (float)(config_.cpr));
    vel_estimate_  += current_meas_period * pll_ki_ * delta_pos_cpr * __delta;
    bool snap_to_zero_vel = false;
    if (fabs(vel_estimate_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_ = 0.0f; //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    
    float interpolated_enc = corrected_enc + interpolation_;
    vel_rpm_ = vel_estimate_ / ((float)__cpr) * 60.0f;
    vel_abs_rpm_ = fabsf(vel_rpm_);
    pos_degree_ = count_in_cpr_ / ((float)(__cpr)) * 360.0f;

    pos_estimate_degree =  ((float)count_in_cpr_ / (float)(__cpr) + (float)(turn_)) * 360.0f;
    vel_estimate_degree = (vel_estimate_ / (float)(__cpr)) * 360.0f;
    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = motor_pole_pairs * 2 * M_PI * (1.0f / (float)(__cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.offset_float);
    phase_ = wrap_pm_pi(ph);

    return true;
}

bool OpticalEncoder::calibrate_offset_rotator(Motor& motor_, float __delta)
{   
    switch(hfi_state.hfi_step)
    {
       case 1:
        {
            hfi_state.hfi_step++;
            motor_.servo_on();
            hfi_state.hfi_time = 0;
            break;
        }
       case 2:
       {
           hfi_state.hfi_time++;
           motor_.FOC_voltage(4.0f, 0.0f, 0.0f);
           if(hfi_state.hfi_time >= 1000)
           {
             hfi_state.encvaluesum = count_in_cpr_ + turn_ * config_.cpr;
             hfi_state.start_pos = count_in_cpr_ + turn_ * config_.cpr;
             hfi_state.hfi_step = 3;
             hfi_state.hfi_time = 0;
           }
           break;
       }
       case 3:
       {
           hfi_state.hfi_time++;
           float ph_ = (float)hfi_state.hfi_time / 8000.0f * 2 * M_PI;
           motor_.FOC_voltage(4.0f, 0.0f, ph_);

           if(hfi_state.hfi_time % 2000 == 0)
           {
            int _offset =  (int)((config_.cpr / motor_.config_.pole_pairs) * (hfi_state.hfi_time / 8000.0f));
            hfi_state.encvaluesum += count_in_cpr_ + turn_ * config_.cpr - _offset;
           }

           if(hfi_state.hfi_time >= 8000)
           {
             int _offset =  (int)(config_.cpr / motor_.config_.pole_pairs);
             int _delta_pos = count_in_cpr_ + turn_ * config_.cpr - hfi_state.start_pos - _offset;
             if(vel_rpm_ < 0.0f || vel_abs_rpm_ < 10.0f || abs(_delta_pos) > _offset / 6)
             {
                motor_.servo_off();
                error_ = ERROR_ERROR_DIRECTION;
                return true;
             }
             
             hfi_state.encvaluesum += count_in_cpr_ + turn_ * config_.cpr - _offset;
             hfi_state.hfi_step = 4;
             hfi_state.hfi_time = 0;
           }

           break;
       }
       case 4:
       {
           hfi_state.hfi_time++;
           float ph_ = 2 * M_PI - (float)hfi_state.hfi_time/ 8000.0f * 2 * M_PI;
           motor_.FOC_voltage(4.0f, 0.0f, ph_);

           if(hfi_state.hfi_time % 2000 == 0)
           {
               int _offset1 =  (int)(config_.cpr / motor_.config_.pole_pairs);
               int _offset2 =  (int)((config_.cpr / motor_.config_.pole_pairs) * (hfi_state.hfi_time / 8000.0f));
               hfi_state.encvaluesum += count_in_cpr_ + turn_ * config_.cpr - _offset1 + _offset2;
           }

           if(hfi_state.hfi_time >= 8000)
           {
            int _offset =  (int)(config_.cpr / motor_.config_.pole_pairs);
             int _delta_pos = count_in_cpr_ + turn_ * config_.cpr - hfi_state.start_pos;  
             if(vel_rpm_ > 0.0f || vel_abs_rpm_ < 10.0f || abs(_delta_pos) > _offset / 6)
             {
                motor_.servo_off();
                error_ = ERROR_ERROR_DIRECTION;
                return true;
             }
             hfi_state.hfi_step = 5;
             hfi_state.hfi_time = 0;
           }
           break;
       }
        case 5:
        {
            config_.offset = hfi_state.encvaluesum / 10;
            config_.offset_float = 0.0f;
            motor_.servo_off();
            hfi_state.hfi_step = 1;
            return true;
        }
    }
    return false;
}

//拉D轴找启动角方式
bool OpticalEncoder::calibrate_offset_clamper(Motor& motor_, float __delta)
{   
    switch(hfi_state.hfi_step)
    {
       case 1:
        {
            hfi_state.hfi_step++;
            motor_.servo_on();
            hfi_state.hfi_time = 0;
            break;
        }
       case 2:
       {
           hfi_state.hfi_time++;
           motor_.FOC_voltage(4.0f, 0.0f, 0.0f);
           if(hfi_state.hfi_time >= 8000)
           {
             hfi_state.hfi_step = 3;
             hfi_state.hfi_time = 0;
           }
           break;
        }
       case 3:
       {
           hfi_state.hfi_time++;
           float ph_ = (float)hfi_state.hfi_time / 8000.0f * 2 * M_PI;
           //motor_.FOC_current(0.0f, -0.3f, ph_, 0.0f);
           motor_.FOC_voltage(6.0f, 0.0f, ph_);
           break;
       }
        case 4:
        {
            config_.offset = count_in_cpr_;
            config_.offset_float = 0.0f;
            motor_.servo_off();
            hfi_state.hfi_step = 1;
            return true;
        }
    }
    return false;
}