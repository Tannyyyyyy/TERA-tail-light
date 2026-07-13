/**
 ******************************************************************************
 * @file    input_filter.h
 * @brief   PDM/PWM-tolerant digital input filter for the opto-isolated
 *          vehicle signals on PB1..PB5.
 ******************************************************************************
 */
#ifndef INPUT_FILTER_H
#define INPUT_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Channel indices, in PB1..PB5 order. */
typedef enum
{
  IN_RUN = 0,   /* PB1: running / tail light */
  IN_BRAKE,     /* PB2: brake light          */
  IN_TURN,      /* PB3: turn signal          */
  IN_REVERSE,   /* PB4: reverse light        */
  IN_AUX,       /* PB5: auxiliary / rear fog */
  IN_COUNT
} input_ch_t;

/* A TIM10 interrupt calls input_filter_sample() at IN_SAMPLE_HZ. Every
 * IN_WINDOW_SAMPLES samples (20 ms) the duty cycle of each channel is
 * computed and pushed through the hysteresis thresholds below. This makes
 * a 10 % PDM running-light feed read as steadily "active" instead of
 * flickering, while still reacting to real edges within 20-40 ms.
 *
 * If your body controller dims with PWM slower than ~100 Hz, enlarge
 * IN_WINDOW_SAMPLES so one window still spans at least one PWM period. */
#define IN_SAMPLE_HZ        5000u
#define IN_WINDOW_SAMPLES   100u    /* 100 / 5 kHz = 20 ms per window */

/* Hysteresis thresholds, in percent duty over one window. */
#define IN_ON_PCT           6u      /* >= : channel becomes active     */
#define IN_OFF_PCT          3u      /* <= : channel becomes inactive   */
#define IN_FULL_ON_PCT      85u     /* >= : treated as solid HIGH      */
#define IN_FULL_OFF_PCT     70u     /* <= : no longer solid HIGH       */

void input_filter_init(void);

/* Call from the 5 kHz timer ISR only. */
void input_filter_sample(void);

/* Debounced results, safe to read from the main loop. */
bool    input_active(input_ch_t ch); /* signal present at any duty >= ~6 % */
bool    input_full(input_ch_t ch);   /* signal is solid HIGH (~100 % duty) */
uint8_t input_duty(input_ch_t ch);   /* last window duty cycle, 0..100     */

#ifdef __cplusplus
}
#endif

#endif /* INPUT_FILTER_H */
