/**
 ******************************************************************************
 * @file    ws2812.c
 * @brief   WS2812B driver: TIM1 PWM + DMA, one pre-rendered duty buffer
 *          per strip.
 *
 * PB14 = TIM1_CH2N -> strip 0, fed by DMA2 Stream 2 (channel 6, TIM1_CH2)
 * PB15 = TIM1_CH3N -> strip 1, fed by DMA2 Stream 6 (channel 6, TIM1_CH3)
 *
 * Each WS2812 bit is one 1.25 us PWM period; the DMA stream writes the next
 * compare value into CCRx on every capture/compare event, so the pulse train
 * is fully hardware-paced and immune to interrupt jitter. The buffer ends
 * with WS_RESET_SLOTS zero-duty periods (line low > 50 us) so the strip
 * latches, then the transfer-complete callback stops the channel. PB14/PB15
 * carry an internal pull-down (see stm32f4xx_hal_msp.c) so the data line
 * stays low while the channel is stopped between frames.
 *
 * Only the OCxN complementary outputs are enabled (the positive OCx pins are
 * left disabled), so OCxN carries the normal, non-inverted PWM waveform.
 ******************************************************************************
 */
#include "ws2812.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

#define WS_BITS_PER_LED  24u
#define WS_BUF_LEN       (WS_NUM_LEDS * WS_BITS_PER_LED + WS_RESET_SLOTS)

static ws_color_t        frame[WS_NUM_STRIPS][WS_NUM_LEDS];
static uint16_t          pwm_buf[WS_NUM_STRIPS][WS_BUF_LEN];
static uint8_t           s_brightness = 255u;
static volatile uint8_t  s_busy_mask;   /* bit0 = strip 0, bit1 = strip 1 */

void ws2812_init(void)
{
  /* pwm_buf lives in .bss, so the reset tail is already all-zero and only
   * the first WS_NUM_LEDS * 24 slots are ever rewritten. */
  s_busy_mask = 0u;
  ws2812_clear_all();
}

void ws2812_set_brightness(uint8_t brightness)
{
  s_brightness = brightness;
}

void ws2812_set(uint8_t strip, uint16_t idx, ws_color_t c)
{
  if ((strip < WS_NUM_STRIPS) && (idx < WS_NUM_LEDS))
  {
    frame[strip][idx] = c;
  }
}

void ws2812_fill(uint8_t strip, uint16_t first, uint16_t count, ws_color_t c)
{
  if (strip >= WS_NUM_STRIPS)
  {
    return;
  }
  for (uint16_t i = 0u; (i < count) && ((first + i) < WS_NUM_LEDS); i++)
  {
    frame[strip][first + i] = c;
  }
}

void ws2812_clear_all(void)
{
  const ws_color_t off = {0u, 0u, 0u};
  for (uint8_t s = 0u; s < WS_NUM_STRIPS; s++)
  {
    ws2812_fill(s, 0u, WS_NUM_LEDS, off);
  }
}

bool ws2812_busy(void)
{
  return s_busy_mask != 0u;
}

/* Expand one strip's RGB frame into PWM compare values, GRB order,
 * MSB first, with the global brightness scale applied. */
static void ws2812_encode(uint8_t strip)
{
  uint16_t *dst = pwm_buf[strip];
  const uint32_t scale = (uint32_t)s_brightness + 1u;

  for (uint16_t i = 0u; i < WS_NUM_LEDS; i++)
  {
    const ws_color_t *c = &frame[strip][i];
    const uint32_t g = ((uint32_t)c->g * scale) >> 8;
    const uint32_t r = ((uint32_t)c->r * scale) >> 8;
    const uint32_t b = ((uint32_t)c->b * scale) >> 8;
    const uint32_t grb = (g << 16) | (r << 8) | b;

    for (uint32_t mask = 1uL << 23; mask != 0u; mask >>= 1)
    {
      *dst++ = (grb & mask) ? (uint16_t)WS_CCR_1 : (uint16_t)WS_CCR_0;
    }
  }
}

bool ws2812_show(void)
{
  if (s_busy_mask != 0u)
  {
    return false;
  }

  ws2812_encode(0u);
  ws2812_encode(1u);

  s_busy_mask = 0x03u;
  if (HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_2,
                               (const uint32_t *)pwm_buf[0],
                               WS_BUF_LEN) != HAL_OK)
  {
    s_busy_mask &= (uint8_t)~0x01u;
  }
  if (HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_3,
                               (const uint32_t *)pwm_buf[1],
                               WS_BUF_LEN) != HAL_OK)
  {
    s_busy_mask &= (uint8_t)~0x02u;
  }
  return true;
}

/* DMA transfer complete for a capture/compare channel. The reset tail has
 * already been clocked out at this point, so the strip is latched and the
 * channel can be released until the next frame. */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1)
  {
    return;
  }
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    (void)HAL_TIMEx_PWMN_Stop_DMA(htim, TIM_CHANNEL_2);
    s_busy_mask &= (uint8_t)~0x01u;
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    (void)HAL_TIMEx_PWMN_Stop_DMA(htim, TIM_CHANNEL_3);
    s_busy_mask &= (uint8_t)~0x02u;
  }
  else
  {
    /* other channels unused */
  }
}
