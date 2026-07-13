/**
 ******************************************************************************
 * @file    ws2812.h
 * @brief   WS2812B driver for two strips on TIM1_CH2N / TIM1_CH3N via DMA.
 ******************************************************************************
 */
#ifndef WS2812_H
#define WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Strip geometry ---------------------------------------------------- */
#define WS_NUM_STRIPS   2u
#define WS_NUM_LEDS     24u   /* LEDs per strip -- set to your strip length */

/* ---- Bit timing (assumes TIM1 kernel clock = 100 MHz) -------------------
 * One WS2812 bit = one 800 kHz PWM period; the compare value picks the
 * high time: 0.40 us => "0", 0.80 us => "1". The frame ends with
 * WS_RESET_SLOTS zero-duty periods (80 us low) to latch the LEDs.        */
#define WS_TIM_ARR      124u  /* 125 ticks @ 100 MHz = 1.25 us (800 kHz)   */
#define WS_CCR_0        40u   /* 0.40 us high = "0" bit                    */
#define WS_CCR_1        80u   /* 0.80 us high = "1" bit                    */
#define WS_RESET_SLOTS  64u   /* 64 x 1.25 us = 80 us latch time           */

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} ws_color_t;

void ws2812_init(void);

/* Global brightness scale 0..255, applied when a frame is encoded.
 * Useful for limiting total LED current without touching the colors. */
void ws2812_set_brightness(uint8_t brightness);

/* Frame-buffer edits. Out-of-range strip/index arguments are ignored. */
void ws2812_set(uint8_t strip, uint16_t idx, ws_color_t c);
void ws2812_fill(uint8_t strip, uint16_t first, uint16_t count, ws_color_t c);
void ws2812_clear_all(void);

/* True while a previous frame is still being clocked out by DMA. */
bool ws2812_busy(void);

/* Encode both frame buffers and start the DMA transfers.
 * Returns false (and does nothing) if the previous frame is still busy. */
bool ws2812_show(void);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
