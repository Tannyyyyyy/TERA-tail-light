#include "main.h"

extern TIM_HandleTypeDef htim1;

#define PWM_MAX 1000

/* PB14/PB15 are TIM1 complementary outputs (CH2N/CH3N). Their level is the
 * inverse of the compare register, so we invert here: 0 = off, 1000 = full. */
static inline uint16_t inv(uint16_t v){ if (v > PWM_MAX) v = PWM_MAX; return PWM_MAX - v; }

void LED_Init(void)
{
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  /* PB14 */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  /* PB15 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_MAX);  /* start off */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_MAX);  /* start off */
}

void LED_SetRed(uint16_t level)   /* PB14 = TIM1_CH2N */
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, inv(level));
}

void LED_SetAux(uint16_t level)   /* PB15 = TIM1_CH3N */
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, inv(level));
}
