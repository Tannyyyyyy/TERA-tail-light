/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt service routines.
 *
 * DMA2 Stream 2 -> TIM1_CH2 (WS2812 strip 0 on PB14)
 * DMA2 Stream 6 -> TIM1_CH3 (WS2812 strip 1 on PB15)
 * TIM10         -> 5 kHz input sampling tick (shares the vector with the
 *                  TIM1 update interrupt, which this project does not use)
 ******************************************************************************
 */
#include "main.h"
#include "stm32f4xx_it.h"

extern TIM_HandleTypeDef htim10;
extern DMA_HandleTypeDef hdma_tim1_ch2;
extern DMA_HandleTypeDef hdma_tim1_ch3;

/* ---- Cortex-M4 core exceptions ------------------------------------------ */

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* ---- Peripheral interrupts ----------------------------------------------- */

void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim1_ch2);
}

void DMA2_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim1_ch3);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim10);
}
