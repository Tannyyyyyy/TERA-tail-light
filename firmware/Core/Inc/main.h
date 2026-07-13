/**
 ******************************************************************************
 * @file    main.h
 * @brief   TERA tail light controller -- pin map and common declarations.
 *
 * MCU: STM32F411CEU6, 25 MHz HSE, SYSCLK 100 MHz.
 ******************************************************************************
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Opto-isolated inputs (PC817 emitter into the pin, external 10 k pull-down
 * to GND). HIGH (~3.3 V) = vehicle signal present. The vehicle side may be
 * PDM/PWM, so these pins are never read directly -- see input_filter.c. */
#define IN_GPIO_PORT        GPIOB
#define IN_RUN_PIN          GPIO_PIN_1   /* running / tail light            */
#define IN_BRAKE_PIN        GPIO_PIN_2   /* brake light (PB2 = BOOT1, see
                                            firmware/README.md)             */
#define IN_TURN_PIN         GPIO_PIN_3   /* turn signal (JTDO by default,
                                            freed by SWD-only debug config) */
#define IN_REVERSE_PIN      GPIO_PIN_4   /* reverse light (NJTRST default)  */
#define IN_AUX_PIN          GPIO_PIN_5   /* auxiliary / rear fog            */

/* WS2812B data outputs. The internal pull-down keeps the data line low
 * while the timer channel is stopped between frames. */
#define WS_GPIO_PORT        GPIOB
#define WS_STRIP0_PIN       GPIO_PIN_14  /* TIM1_CH2N, AF1 */
#define WS_STRIP1_PIN       GPIO_PIN_15  /* TIM1_CH3N, AF1 */

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
