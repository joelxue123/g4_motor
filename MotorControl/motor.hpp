#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "interface.h"

/*
*** 电机配置数据，存放在永久储存区，启动时读入
*/
typedef struct{
    int     num;
    //bool pre_calibrated; 
    int     motor_type;         // 电机种类
    int32_t pole_pairs;         // 电机磁极对，目前帧正电机磁极对为4，上有厂电机磁极对为8
    float phase_inductance;     // 电机相电感，初始值为电机厂给定值，可通过程序测定，单位[Ω]
    float phase_resistance;     // 电机相电流，初始值为电机厂给定值，可通过程序测定，单位[H]

    //float adc_offset_phA;                   // A相电流零位点的ADC值，需要自检程序预测试好，单位为ADC值
    //float adc_offset_phB;                   // B相电流零位点的ADC值，需要自检程序预测试好，单位为ADC值
    float adc_current_k;                    // ADC电流转换系数，单位[A]
    float current_lim;                      // 可测的最大电流值范围，单位[A]
    float requested_current_range;          // 最大输入电流范围，单位[A]
    float current_control_bandwidth;        // 电流环设定带宽，单位[rad/s]
}motor_config_t;

/*
*** 电机管理类
*/
class Motor {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x0001,   //电机电阻测量值超出范围
        ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x0002,   //电机电感测量值超出范围
        ERROR_ADC_FAILED = 0x0004,                      // ADC零漂超出可接受极限
        ERROR_OVER_CURRENT = 0x0008,                    //电机相电流超过可测量极限
        ERROR_OVER_VOLTAGE = 0x0010,                    //电机输入电压过高
        ERROR_UNDER_VOLTAGE = 0x0020,                   //电机输入电压过低
        ERROR_OVER_TEMP = 0x0040,                       //电机过热
    };

    enum State_t {
        STATE_NORMAL = 0,                               // 正常态
        STATE_ERROR = 0x0001,                           // 错误状态
        STATE_INIT  = 0x0002,                           // 初始化状态
        STATE_CALIBRE_R = 0x0003,                       // 电阻测量状态
        STATE_CALIBRE_L = 0x0004                        // 电感测量状态
    };

    typedef struct Iph_ABC_t {
        float phA;
        float phB;
        float phC;
    }Iph_ABC_t;

    typedef struct adc_current_offset_t{
        float iaOffset;
        float ibOffset;
        float extOffset;
    }adc_current_offset_t;

    typedef struct CurrentControl_t{
        float p_gain; // [V/A]
        float i_gain; // [V/As]
        float v_current_control_integral_d; // [V]
        float v_current_control_integral_q; // [V]
        float Ibus; // DC bus current [A]
        float final_v_alpha; // [V]
        float final_v_beta; // [V]
        float Iq_setpoint;
        float Iq_measured;
        float Id_measured;
        float max_allowed_current;
        float deadband;
        float i_A_delta;
        float i_B_delta;
    }CurrentControl_t;

    Motor(motor_config_t& _config);

    bool init();
    
    // 设计上，使得电机上电，U、V、W三相就短接到地，使得有个发电力矩保持和重力力矩抗衡，不会出现机械臂快速坠落的情况。
    // 所有这里设置三种状态，不论servo_on，还是servo_off都会保证pwm输出打开而非不可知的悬空状态。
    inline void servo_on(void) {motor_set_out_put(config_.num, 2100, 2100, 2100);motor_servo_on(config_.num);is_enable = true;}
    inline void servo_off(void) {is_enable = false;motor_set_out_put(config_.num, 2100, 2100, 2100);motor_servo_on(config_.num);}
    inline void motor_off(void) {motor_servo_off(config_.num);is_enable = false;}

    // 抱闸关闭，电机静止，三相断路，此时测试ADC零漂值，判断与预定义的相差多大
    bool adjust_adc_offset(void);
    bool phase_current_from_adcval(void);

    // 根据设定带宽，重新生成计算pid参数
    void reset_current_control(void);
    void set_error(Error_t error);
    
    bool FOC_voltage(float v_d, float v_q, float phase);
    bool FOC_current(float Id_des, float Iq_des, float phase, float omega);

    bool measure_phase_resistance(void);
    bool measure_phase_inductance(void);
    void calibrate(void);

    // 电机类"时钟心跳"进程
    bool update(void);

    //Axis* axis_ = nullptr; // set by Axis constructor

//private:

    motor_config_t& config_;

    Error_t error_ = ERROR_NONE;
    float bus_udc_ = 0.0f;
    float bus_udc_filt_ = 0.0f;
    float low_udc_ = 0.0f;
    float low_udc_filt_ = 0.0f; 
    float Ialpha = 0;
    float Ibeta = 0;
    float Izero = 0;
    const float current_meas_period = 0.0000625f;
    bool is_enable = false;
    bool is_calibrated_ = false;
    bool is_over_current = false;
    int  init_count = 0;
    float phase_inductance = 0.0f;        // 相电感测量值
    float phase_resistance = 0.0f;        // 相电阻测量值
    float mcu_tempure = 0.0f;
    float extern_tempure = 0.0f;
    float extern_torque = 0.0f;
    uint32_t A_pwm = 0;
    uint32_t B_pwm = 0;
    uint32_t C_pwm = 0;

    // 测量时，使用的临时变量
    int  calibrate_step = 0;
    float calibrate_voltage = 0.0f;
    float calibrate_voltage1 = 0.0f;
    float calibrate_Ialphas[2] = {0.0f};

    State_t   state_ = STATE_INIT;
    Iph_ABC_t current_meas_ = {0.0f, 0.0f, 0.0f};
    adc_current_offset_t ADC_offset_ = {2048.0f, 2048.0f};
    CurrentControl_t current_control_ = {
        .p_gain = 1.675f,        // [V/A] 
        .i_gain = 450.0f,        // [V/As] 
        .v_current_control_integral_d = 0.0f,
        .v_current_control_integral_q = 0.0f,
        .Ibus = 0.0f,
        .final_v_alpha = 0.0f,
        .final_v_beta = 0.0f,
        .Iq_setpoint = 0.0f,
        .Iq_measured = 0.0f,
        .Id_measured = 0.0f,
        .max_allowed_current = 32.0f,
        .deadband = 2.50f,
        .i_A_delta = 1.5f,
        .i_B_delta = 0.5f,
    };
};

#endif // __MOTOR_HPP
