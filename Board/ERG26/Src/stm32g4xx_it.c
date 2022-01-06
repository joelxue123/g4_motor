/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "interface.h"
#ifdef SYSVIEW_DEBUG
#include "SEGGER_SYSVIEW.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */
volatile int g_pmsm_ia_org = 0;
volatile int g_pmsm_ib_org = 0;
volatile int g_pmsm_ia1_org = 0;
volatile int g_pmsm_ib1_org = 0;
volatile int g_bus_volt_org = 0;

volatile uint32_t g_encoder_value_org = 0;
volatile uint32_t g_encoder1_value_org = 0;
uint32_t g_encoder_value_temp = 0;
volatile uint32_t g_fault_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
inline static void ADC_DataUpdate(void)
{
  g_pmsm_ia_org = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
  g_pmsm_ib_org = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
  g_bus_volt_org = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
  g_fault_flag = (HAL_GPIO_ReadPin(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin) == GPIO_PIN_RESET);
  //g_encoder_value_org = 4095 - LL_TIM_GetCounter(TIM3);
  //g_encoder_value_org = 0;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  g_encoder_value_org = (g_encoder_value_temp & 0x7fff);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  HAL_SPI_Receive_IT(hspi, (uint8_t *)(&g_encoder_value_temp), 1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  motor_servo_off(0);
  motor_servo_off(1);
  led_state_clr();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
    led_fault_set();
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  motor_servo_off(0);
  motor_servo_off(1);
  led_state_clr();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    led_fault_set();
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  motor_servo_off(0);
  motor_servo_off(1);
  led_state_clr();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    led_fault_set();
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  motor_servo_off(0);
  motor_servo_off(1);
  led_state_clr();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    led_fault_set();
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  motor_servo_off(0);
  motor_servo_off(1);
  led_state_clr();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    led_fault_set();
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END ADC1_2_IRQn 0 */
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
  if(LL_ADC_IsEnabledIT_JEOS(ADC2))
  {
    uint16_t ang_reg_v = 0x8021;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)(&ang_reg_v), 1);

    ADC_DataUpdate();
    high_realtime_interrupt();
    LL_ADC_ClearFlag_JEOS(ADC2);
  }
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordEnterISR();
#endif
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_RecordExitISR();
#endif
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
