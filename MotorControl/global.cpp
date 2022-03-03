/**
 * @file global.cpp
 * @brief 
 * @author htkang (htkang616@outlook.com)
 * @version 1.0
 * @date 2021-04-30
 * 
 * @copyright Copyright (c) 2021  苏州钧舵机器人有限公司
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-04-30 <td>1.0     <td>htkang     <td>内容
 * </table>
 */
 
#include "interface.h"
#include "motor.hpp"
#include "controller.hpp"
#include "optical_encoder.hpp"
#include "global.hpp"

trap_config_t g_trap_conf =
{
    .vel_limit =  10000.00f * 6.0f,   // [degree/s]
    .accel_limit = 1000000.0f * 6.0f,  // [degree/s^2]
    .decel_limit = 1000000.0f * 6.0f, // [degree/s^2]
    .A_per_css = 0.0f,   // [A/(degree/s^2)]
};

ctrl_config_t g_ctrl_conf =
{
    .pos_gain = 10.0f,  // [(degree/s) / degree]
    .vel_gain = 0.005f / 360.0f,  // [A/(degree/s)] (速度环kp = bandwidth * J / kt, 取带宽40Hz，大关节空载转动惯量比测得为: J/kt = 3.44 / (150 * 2pi * 1.01) A/(rad/s^2))
    .vel_integrator_gain = 0.01f / 360.0f,  // [A/(degree/s * s)] (速度环ki = bandwidth * kp, 取带宽40Hz，控制)
    .vel_limit = 20000.0f * 6.0f,        // [degree/s]
    .acc_limit = 2000000.0f * 6.0f,      // [degree/s^2]
    .jerk_limit = 2000000.0f * 6.0f,     // [degree/s^3]
    .vel_limit_tolerance = 0.0f,  // ratio to vel_lim. 0.0f to disable
    .vel_ramp_rate = 80000.0f * 6.0f ,  // [(degree/s) / s]
};

motor_config_t g_motor_config =
{
    .num = 0,   //1号电机
    .motor_type = 0,
    .pole_pairs = 4,                             //极对数
    .phase_inductance = 0.33e-3f,               // 电机相电感，初始值为电机厂给定值，可通过程序测定，单位[H]
    .phase_resistance = 11.5f,                  // 电机相电组，初始值为电机厂给定值，可通过程序测定，单位[Ω]
    .adc_current_k = 3.3f / 4096.0f / 0.02f / 21.0f,        //单位 安培每AD值
    .current_lim = 1.0f,                      // 可测的最大电流值范围，单位[A]
    .requested_current_range = 0.5f,          // 最大输入电流范围，单位[A]
    .current_control_bandwidth = 1000,  // [rad/s]
};

optical_encoder_config_t g_encoder_config =
{
    .num = 0,    //1号编码器
    .use_index = false,
    .idx_search_speed = 0.0f, // [rad/s electrical]
    .zero_count_on_find_idx = false,
    .cpr = 32768,
    .offset = 0,        // Offset between encoder count and rotor electrical phase
    .offset_float = 0.5f, // Sub-count phase alignment offset
    .calib_range = 0.02f,
    .bandwidth = 500.0f,
};

Motor g_motor(g_motor_config);
OpticalEncoder g_optical_encoder(g_encoder_config);
Controller g_ctrl(g_ctrl_conf, g_trap_conf);
