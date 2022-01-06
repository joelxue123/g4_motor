#ifndef __OPTICALENCODER_HPP
#define __OPTICALENCODER_HPP

#include "interface.h"
#include <stdint.h>

class Motor;

/*
*** 电机配置数据，存放在永久储存区，启动时读入
*/
typedef struct{
    int  num;
    bool use_index;         
    float idx_search_speed; // [rad/s electrical]
    bool zero_count_on_find_idx = true;
    int32_t cpr = (2048 * 4);           // 电机每旋转一圈所对应的脉冲数
    int32_t offset = 0;                 // 编码器零点与电气角零点之间的偏移，单位为脉冲计数。
    float offset_float = 0.0f;          // 偏移的浮点数插补出的精确值，单位为脉冲。
    float calib_range = 0.02f;          // 寻找偏移角的范围。
    float bandwidth = 5000.0f;          // PLL锁相环设定带宽，单位[rad/s]。
}optical_encoder_config_t;

/*
*** 光编管理类
*/
class OpticalEncoder {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
        ERROR_CPR_OUT_OF_RANGE = 0x02,
        ERROR_ERROR_DIRECTION = 0x04,
        ERROR_INDEX_NOT_FOUND_YET = 0x20,
    };

    OpticalEncoder(optical_encoder_config_t& _config);

    bool  init();
    
    void update_pll_gains();
    bool update(int motor_pole_pairs, float __delta);
    void set_error(Error_t _error);
    void set_align();
    bool calibrate_offset_rotator(Motor& motor_, float __delta);
    bool calibrate_offset_clamper(Motor& motor_, float __delta);

    optical_encoder_config_t& config_;
    Error_t error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    int32_t turn_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;    // [count]
    float pos_estimate_ = 0.0f;  // [count]
    float pos_cpr_ = 0.0f;  // [count]
    float vel_estimate_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float vel_rpm_ = 0.0f;   // [rpm]
    float vel_abs_rpm_ = 0.0f;   // [rpm]
    float pos_degree_ = 0.0f; //[degree]
    float pos_estimate_degree = 0.0f;
    float vel_estimate_degree = 0.0f;

    /*
    ** 在没有保证光编码器的index信号有效的情况下
    ** 采用复杂的hfi启动方法求出一个近似的offset出来
    ** 后续会采用index信号对应一个准确的offset，启动时采用磁编绝对位置启动(误差10度)
    */
    typedef struct
    {
        float filt_bufferM;
        float filt_bufferM_K_1;
        float filt_bufferY;
        float filt_bufferY_K_1;
    } iir2_fliter_t;

    typedef struct
    {
        iir2_fliter_t hfi_sin_filter;
        iir2_fliter_t hfi_cos_filter;
        int           hfi_step;
        float         hfi_max_idh;
        int           hfi_search_index;
        float         hfi_search_angle[3];
        int           hfi_res_cnt;
        float         hfi_res_current[3];    
        float         hfi_rad_cnd;
        float         hfi_temp_angle;
        float         hfi_phase_offset;
        int           hfi_time;
        int           hfi_rst_delay;
        int64_t       encvaluesum;
        int32_t       start_pos;
    } hfi_state_t;

    hfi_state_t hfi_state = {0};

    const float current_meas_period = 5e-5f;
};

#endif
