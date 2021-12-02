#include "motor.hpp"
#include "interface.h"
#include "utils.hpp"

#define PWM_PERIOD 5250

Motor::Motor(motor_config_t& _config) 
    : config_(_config)
{
    init();
}

bool Motor::init() {
    motor_off();
    reset_current_control();
    calibrate_step = 0;
    calibrate_voltage = 0.0f;
    calibrate_Ialphas[0] = 0.0f;
    calibrate_Ialphas[1] = 0.0f;
    state_ = STATE_INIT;
    error_ = ERROR_NONE;
    init_count = 0;
    return true;
}

void Motor::reset_current_control() {
    current_control_.p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    float plant_pole = config_.phase_resistance / config_.phase_inductance;
    current_control_.i_gain = plant_pole * current_control_.p_gain;
    current_control_.max_allowed_current = config_.requested_current_range;
    current_control_.v_current_control_integral_d = 0.0f;
    current_control_.v_current_control_integral_q = 0.0f;
    state_ = STATE_NORMAL;
    //error_ = ERROR_NONE;
}

void Motor::set_error(Error_t error){
    error_ = error;
    state_ = STATE_ERROR;
}

bool Motor::adjust_adc_offset(void)
{
    if(config_.num == 0) {
        ADC_offset_.iaOffset = 0.95f * ADC_offset_.iaOffset + 0.05f * g_pmsm_ia_org;
        ADC_offset_.ibOffset = 0.95f * ADC_offset_.ibOffset + 0.05f * g_pmsm_ib_org;
    }
    else {
        ADC_offset_.iaOffset = 0.95f * ADC_offset_.iaOffset + 0.05f * g_pmsm_ia1_org;
        ADC_offset_.ibOffset = 0.95f * ADC_offset_.ibOffset + 0.05f * g_pmsm_ib1_org;
    }

    bus_udc_ = ((float)(g_bus_volt_org) / 4096.0f * 3.3f) * 11.0f;
    bus_udc_filt_ = 0.98f * bus_udc_filt_ + 0.02f * bus_udc_;
    return true;
}

bool Motor::phase_current_from_adcval(void) {
    float k = config_.adc_current_k;

    if(config_.num == 0)
    {
        current_meas_.phB = (float) (g_pmsm_ib_org - ADC_offset_.ibOffset) * k;
        current_meas_.phC = (float) (g_pmsm_ia_org - ADC_offset_.iaOffset) * k;
    }
    else
    {
        current_meas_.phB = (float) (g_pmsm_ib1_org - ADC_offset_.ibOffset) * k;
        current_meas_.phC = (float) (g_pmsm_ia1_org - ADC_offset_.iaOffset) * k;
    }
    current_meas_.phA = -(current_meas_.phB + current_meas_.phC);
    //current_meas_.phC = -(current_meas_.phB + current_meas_.phA);
        //Izero = current_meas_.phA  + current_meas_.phB + current_meas_.phC;
    
    bus_udc_ = ((float)(g_bus_volt_org) / 4096.0f * 3.3f) * 11.0f;
    bus_udc_filt_ = 0.98f * bus_udc_filt_ + 0.02f * bus_udc_;
    Ialpha = current_meas_.phA;
    Ibeta = -one_by_sqrt3 * (current_meas_.phA + 2.0f *current_meas_.phB);

    // 过流判断
    if(current_meas_.phA > config_.current_lim || current_meas_.phA < -config_.current_lim)
    {
        set_error(ERROR_OVER_CURRENT);
        return false;
    }

    if(current_meas_.phB > config_.current_lim || current_meas_.phB < -config_.current_lim)
    {
        set_error(ERROR_OVER_CURRENT);
        return false;
    }
    
    if(bus_udc_filt_ < 20.0f)
    {
        set_error(ERROR_UNDER_VOLTAGE);
        return false;
    }

    if(bus_udc_filt_ > 30.0f)
    {
        set_error(ERROR_OVER_VOLTAGE);
        return false;
    }

    return true;
}

// 开环电压控制
bool Motor::FOC_voltage(float v_d, float v_q, float phase) {
    if(!is_enable) return false;

    float c = our_arm_cos_f32(phase);
    float s = our_arm_sin_f32(phase);
    float v_alpha = s*v_d + c*v_q;
    float v_beta  = -s*v_q + c*v_d;

    float tA = 0.0f;
    float tB = 0.0f; 
    float tC = 0.0f;
    simple_svm(v_alpha, v_beta, bus_udc_filt_, &tA, &tB, &tC);
    //SVM(v_alpha,v_beta, bus_udc_filt_, &tA, &tB, &tC);
        
    int result_valid =
            tA >= 0.0f && tA <= 1.0f
         && tB >= 0.0f && tB <= 1.0f
         && tC >= 0.0f && tC <= 1.0f;

    if(result_valid)
    {
        A_pwm = (uint32_t)(tA * PWM_PERIOD);
        B_pwm = (uint32_t)(tB * PWM_PERIOD);
        C_pwm = (uint32_t)(tC * PWM_PERIOD);
        motor_set_out_put(config_.num, A_pwm, B_pwm, C_pwm);
    }
    return true;
}

// 闭环FOC电流环控制
bool Motor::FOC_current(float Id_des, float Iq_des, float phase, float omega) {
    if(!is_enable) return false;
    float kp = current_control_.p_gain;
    float ki = current_control_.i_gain;
    CurrentControl_t& ictrl = current_control_;

    // For Reporting
    ictrl.Iq_setpoint = Iq_des;
    float c = our_arm_cos_f32(phase);
    float s = our_arm_sin_f32(phase);
    float Id = s * Ialpha + c * Ibeta;
    float Iq = -s * Ibeta + c * Ialpha;

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;
    ictrl.Iq_measured = Iq;
    ictrl.Id_measured = Id;
    extern_torque = fabsf(Iq);

#if 0
    // 由于电流测量精度有限且方差较大，故增加控制死区
    if(Ierr_d > - ictrl.deadband && Ierr_d < ictrl.deadband)
    {
        Ierr_d = 0.0f;
    }

    if(Ierr_q > - ictrl.deadband && Ierr_q < ictrl.deadband)
    {
        Ierr_q = 0.0f;
    }
#endif

#if 0
    // 变速积分控制，自动调节积分参数，减小超调
    float delt_abs = fabsf(Ierr_q);
    if(delt_abs > (ictrl.i_B_delta + ictrl.i_A_delta))
    {
        ki = 0;
    }
    else if(delt_abs > ictrl.i_B_delta && delt_abs <= (ictrl.i_B_delta + ictrl.i_A_delta))
    {
        ki = ki * (ictrl.i_A_delta - delt_abs + ictrl.i_B_delta) / ictrl.i_A_delta;
    }
    else
    {
        ki = ki;
    }   
#endif

    // Apply PI control
    float Vd = ictrl.v_current_control_integral_d + Ierr_d * kp;
    float Vq = ictrl.v_current_control_integral_q + Ierr_q * kp;
    //电流环前馈解耦项
    //Vq += omega * 3.5f / 1000.0f;
    //Vd -= config_.phase_inductance * (omega  / 60.0f * 4.0f) * 2 * M_PI * Iq;

    float mod_to_V = (2.0f / 3.0f) * bus_udc_filt_;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // 积分饱和运算
    float mod_scalefactor = 0.97f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        if(fabsf(ictrl.v_current_control_integral_d) > 0.0001f)
            ictrl.v_current_control_integral_d *= 0.99f;
        if(fabsf(ictrl.v_current_control_integral_q) > 0.0001f)
            ictrl.v_current_control_integral_q *= 0.99f;
    } else {
        ictrl.v_current_control_integral_d += Ierr_d * (ki * current_meas_period);
        ictrl.v_current_control_integral_q += Ierr_q * (ki * current_meas_period);
    }

    // Compute estimated bus current
    ictrl.Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float mod_alpha = s * mod_d + c * mod_q;
    float mod_beta  = -s * mod_q + c * mod_d;
    
    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl.final_v_alpha = mod_to_V * mod_alpha;
    ictrl.final_v_beta = mod_to_V * mod_beta;

    // SVPWM输出
    float tA = 0.0f;
    float tB = 0.0f; 
    float tC = 0.0f;
    simple_svm(  ictrl.final_v_alpha, ictrl.final_v_beta, bus_udc_filt_, &tA, &tB, &tC);

    int result_valid =
            tA >= 0.0f && tA <= 1.0f
         && tB >= 0.0f && tB <= 1.0f
         && tC >= 0.0f && tC <= 1.0f;

    if(result_valid)
    {
    //SVM( ictrl.final_v_alpha, ictrl.final_v_beta, bus_udc_filt_, &tA, &tB, &tC);
        A_pwm =  (uint32_t)(tA * PWM_PERIOD);
        B_pwm =  (uint32_t)(tB * PWM_PERIOD);
        C_pwm =  (uint32_t)(tC * PWM_PERIOD);
        motor_set_out_put(config_.num, A_pwm, B_pwm, C_pwm);
    }
    return true;
}

// 先给定一个5A的目标电流，得到需要的电压矢量值，然后给一个10A的目标电流，得到新的电压矢量值。
// 以线性关系近似处理，取这个斜率为相电阻值。
bool Motor::measure_phase_resistance() {
    static const float kI = 1.25f;                                 // [(V/s)/A]
    static const int num_test_cycles = static_cast<int>(4.0f / current_meas_period); // Test runs for 3s
    
    if(calibrate_step < num_test_cycles)
    {
        calibrate_voltage += (kI * current_meas_period) * (5.0f - Ialpha);
        if(calibrate_voltage > 12.0f || calibrate_voltage < -12.0f)
        {
            set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
            servo_off();
            return false;
        }
        FOC_voltage(calibrate_voltage, 0.0f, M_PI / 2);
        calibrate_step++;
        if(calibrate_step == num_test_cycles)
        {
            calibrate_voltage1 = calibrate_voltage;
        }
        return false;
    }
    else if(calibrate_step >= num_test_cycles && calibrate_step < 2 * num_test_cycles)
    {
        calibrate_voltage += (kI * current_meas_period) * (10.0f - Ialpha);
        if(calibrate_voltage > 12.0f || calibrate_voltage < -12.0f)
        {
            set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
            servo_off();
            return false;
        }
        FOC_voltage(calibrate_voltage, 0.0f, M_PI / 2);
        calibrate_step++;
        return false;
    }
    else
    {
        float R = (calibrate_voltage - calibrate_voltage1) / 5.0f;
        phase_resistance = R;
        if (R < 0.05f || R > 0.5f)
        {
            set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
        }
        return true; // if we ran to completion that means success
    }
}

// 给定一个交流方波，求平均响应电流，做比例关系得到相电感的值。
bool Motor::measure_phase_inductance() {
    static const int num_cycles = 40000;

    if(calibrate_step < (num_cycles << 1) )
    {
        int i = calibrate_step & 1;
        calibrate_Ialphas[i] += Ialpha;

        FOC_voltage(calibrate_voltage, 0.0f, M_PI / 2);
        calibrate_step++;
        calibrate_voltage = -calibrate_voltage;
        return false;
    }
    else
    {
        float v_L = 2.0f;
        // Note: A more correct formula would also take into account that there is a finite timestep.
        // However, the discretisation in the current control loop inverts the same discrepancy
        float dI_by_dt = (calibrate_Ialphas[0] - calibrate_Ialphas[1]) / (current_meas_period * (float)num_cycles);
        float L = v_L / dI_by_dt;

        phase_inductance = L;
        // TODO arbitrary values set for now
        //if (L < 500e-6f || L > 4000e-7f)
        //{
        //    set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE);
        //}
        return true;
    }
}

// 电机状态不为错误态时，切到测量状态。
void Motor::calibrate(void)
{
    if((state_ != STATE_CALIBRE_R && state_ != STATE_CALIBRE_L) && state_ != STATE_ERROR)
    {
        servo_on();
        calibrate_step = 0;
        calibrate_voltage = 0.0f;
        state_ = STATE_CALIBRE_R;
    }
}

bool Motor::update(void) {
    switch(state_)
    {
        case STATE_INIT:
        {
            // 初始态存在6万次心跳，测量ADC偏移
            adjust_adc_offset();
            init_count++;
            if(init_count == 4000)
            {
                state_ = STATE_NORMAL;
                servo_off();
            }
            break;
        }
        case STATE_CALIBRE_R:
        {
            if(phase_current_from_adcval())
            {
                if(measure_phase_resistance())
                {
                    calibrate_step = 0;
                    calibrate_voltage = 6.0f;
                    calibrate_Ialphas[0] = 0.0f;
                    calibrate_Ialphas[1] = 0.0f;
                    state_ = STATE_CALIBRE_L;
                }
            }
            break;
        }
        case STATE_CALIBRE_L:
        {
            if(phase_current_from_adcval())
            {
                if(measure_phase_inductance())
                {
                    calibrate_step = 0;
                    calibrate_voltage = 0.0f;
                    calibrate_Ialphas[0] = 0.0f;
                    calibrate_Ialphas[1] = 0.0f;
                    state_ = STATE_NORMAL;
                    servo_off();
                }
            }
            break;
        }
        case STATE_NORMAL:
        {
            phase_current_from_adcval();
            break;
        }
        case STATE_ERROR:
        {
            motor_off();
            phase_current_from_adcval();
            break;
        }
        default:
        {
            motor_off();
        }
    }
    
    return true;
}
