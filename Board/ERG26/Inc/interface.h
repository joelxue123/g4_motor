/**
 * @file interface.h
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
 
#ifndef __INTERFACE_H__
#define __INTERFACE_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

extern volatile int g_pmsm_ia_org;
extern volatile int g_pmsm_ib_org;
extern volatile int g_pmsm_ia1_org;
extern volatile int g_pmsm_ib1_org;
extern volatile int g_bus_volt_org;

extern volatile uint32_t g_encoder_value_org;
extern volatile uint32_t g_encoder1_value_org;

extern void main_setup(void);
extern void main_loop(void);

extern void high_realtime_interrupt(void);
void high_realtime_interrupt_init(void);

extern void realtime_interrupt(void);
void realtime_interrupt_init(void);

void encoder_init(int __init_value, int __cpr);
#define encoder_get_value() g_encoder_value_org;

void motor_servo_on(int num);
void motor_servo_off(int num);
void motor_brake_on(int num);
void motor_brake_off(int num);
void motor_brake_hold(int num);
void motor_set_out_put(int num, uint32_t pha, uint32_t phb, uint32_t phc);
uint8_t motor_is_brake_off(int num);
#define motor_get_ia_value() g_pmsm_ia_org;
#define motor_get_ib_value() g_pmsm_ib_org;

void led_fault_set(void);
void led_fault_clr(void);
void led_fault_toggle(void);
void led_state_toggle(void);
void led_state_clr(void);

void system_delay_ms(int ms);
void system_delay_us(uint32_t us);
void system_restart(void);

#ifdef __cplusplus
 }
#endif

#endif
 