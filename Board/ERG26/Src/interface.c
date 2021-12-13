/**
 * @file interface.c
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
#include "stm32g4xx.h"
#include "gpio.h"
#include "stm32g4xx_ll_tim.h"
#include "tim.h"
#include "main.h"
#include "stspin32g4.h"

STSPIN32G4_HandleTypeDef hdlG4;


void high_realtime_interrupt_init(void)
{
    HAL_StatusTypeDef ret = STSPIN32G4_init(&hdlG4);
    /*************************************************/
    /*   STSPIN32G4 driver component initialization  */
    /*************************************************/
    STSPIN32G4_init( &hdlG4 );
    STSPIN32G4_reset( &hdlG4 );
    STSPIN32G4_setVCC( &hdlG4, (STSPIN32G4_confVCC){ .voltage = _EXT,
                                                           .useNFAULT = true,
                                                           .useREADY = false } );
    STSPIN32G4_setVDSP( &hdlG4, (STSPIN32G4_confVDSP){ .deglitchTime = _4us,
                                                             .useNFAULT = true } );
    STSPIN32G4_clearFaults( &hdlG4 );
   
    LL_TIM_GenerateEvent_CC4(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);
    
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    LL_ADC_INJ_StartConversion(ADC2);
    LL_ADC_INJ_StartConversion(ADC1);
    LL_ADC_EnableIT_JEOS(ADC2);
}

void realtime_interrupt_init(void)
{
}

void encoder_init(int __init_value, int __cpr)
{

}

void motor_servo_on(int num)
{
    //LL_TIM_EnableAllOutputs(TIM1);
    //HAL_GPIO_WritePin(PIN_EN_GATE_1_GPIO_Port, PIN_EN_GATE_1_Pin, GPIO_PIN_SET);
}

void motor_servo_off(int num)
{
    //LL_TIM_DisableAllOutputs(TIM1);
}

void motor_set_out_put(int num, uint32_t pha, uint32_t phb, uint32_t phc)
{
    LL_TIM_OC_SetCompareCH1(TIM1, pha);
    LL_TIM_OC_SetCompareCH2(TIM1, phb);
    LL_TIM_OC_SetCompareCH3(TIM1, phc);
}

void motor_brake_on(int num)
{

}

void motor_brake_hold(int num)
{

}

void motor_brake_off(int num)
{

}

uint8_t motor_is_brake_off(int num)
{
    return 0;
}

void led_fault_set(void)
{
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

void led_fault_clr(void)
{
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}

void led_fault_toggle(void)
{
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
}

void led_state_toggle(void)
{
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}

void led_state_clr(void)
{
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}

void system_delay_ms(int ms) {
    HAL_Delay(ms);
}

void system_restart(void)
{
    HAL_DeInit();
    HAL_MspDeInit();
    NVIC_SystemReset();
}

// @brief: Returns number of microseconds since system startup
uint32_t system_micros(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
     } while (ms != HAL_GetTick());

    return ((ms * 1000) + cycle_cnt);
}

// @brief: Busy wait delay for given amount of microseconds (us)
void system_delay_us(uint32_t us)
{
    uint32_t start = system_micros();
    uint64_t _sum = (uint64_t)start + (uint64_t)us;
    // 时间溢出处理
    if(_sum < UINT32_MAX)
    {
        while (system_micros() - start < us) {
            asm volatile ("nop");
        }
    }
    else {
        while (system_micros() > start) {
            asm volatile ("nop");
        }
        while (UINT32_MAX + system_micros() - start < us) {
            asm volatile ("nop");
        }
    }
}
