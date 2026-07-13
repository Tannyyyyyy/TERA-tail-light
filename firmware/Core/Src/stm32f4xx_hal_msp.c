/**
 ******************************************************************************
 * @file    stm32f4xx_hal_msp.c
 * @brief   MSP (low-level) initialisation: clocks, GPIO alternate functions,
 *          DMA stream wiring, NVIC.
 ******************************************************************************
 */
#include "main.h"

extern DMA_HandleTypeDef hdma_tim1_ch2;
extern DMA_HandleTypeDef hdma_tim1_ch3;

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
 * TIM1 (WS2812B bit stream):
 *  - PB14 = TIM1_CH2N, PB15 = TIM1_CH3N (AF1). The internal pull-down keeps
 *    each data line low while its channel is stopped between frames.
 *  - DMA2 Stream 2 / channel 6 serves TIM1_CH2, Stream 6 / channel 6 serves
 *    TIM1_CH3 (RM0383 table 28), memory-to-peripheral, half-word, normal
 *    (one-shot) mode.
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance != TIM1)
  {
    return;
  }

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = WS_STRIP0_PIN | WS_STRIP1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(WS_GPIO_PORT, &GPIO_InitStruct);

  hdma_tim1_ch2.Instance = DMA2_Stream2;
  hdma_tim1_ch2.Init.Channel = DMA_CHANNEL_6;
  hdma_tim1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim1_ch2.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim1_ch2.Init.Mode = DMA_NORMAL;
  hdma_tim1_ch2.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim1_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_tim1_ch2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC2], hdma_tim1_ch2);

  hdma_tim1_ch3.Instance = DMA2_Stream6;
  hdma_tim1_ch3.Init.Channel = DMA_CHANNEL_6;
  hdma_tim1_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim1_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim1_ch3.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim1_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim1_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim1_ch3.Init.Mode = DMA_NORMAL;
  hdma_tim1_ch3.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim1_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_tim1_ch3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC3], hdma_tim1_ch3);
}

/**
 * TIM10: 5 kHz input-sampling time base. Its update interrupt shares the
 * TIM1_UP_TIM10 vector; the TIM1 update interrupt itself stays disabled.
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM10)
  {
    return;
  }

  __HAL_RCC_TIM10_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}
