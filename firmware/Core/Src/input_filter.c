/**
 ******************************************************************************
 * @file    input_filter.c
 * @brief   Duty-cycle measurement + hysteresis for the PDM/PWM inputs.
 *
 * Automotive body controllers commonly drive lamp outputs with PWM/PDM
 * (dimming, LED-friendly feeds, shared tail/brake wires), so a raw GPIO read
 * toggles constantly even when the function is logically "on". Instead, all
 * five pins are sampled from a 5 kHz timer interrupt and each channel's duty
 * cycle is measured over a 20 ms window:
 *
 *   duty >= IN_ON_PCT      -> channel active (any PWM dimming level)
 *   duty >= IN_FULL_ON_PCT -> channel "full" (essentially solid HIGH)
 *
 * Both decisions use hysteresis so a signal sitting near a threshold cannot
 * chatter. The window mechanism also debounces ordinary solid 12 V inputs.
 *
 * input_full() lets you split a shared tail/brake wire: a dim PWM feed is
 * active-but-not-full (running lights), a solid feed is full (brake). See
 * COMBINED_TAIL_BRAKE_INPUT in main.c.
 ******************************************************************************
 */
#include "input_filter.h"
#include "main.h"

static const uint16_t s_pin[IN_COUNT] =
{
  IN_RUN_PIN, IN_BRAKE_PIN, IN_TURN_PIN, IN_REVERSE_PIN, IN_AUX_PIN
};

/* Written only from the sampling ISR. */
static uint16_t s_hi[IN_COUNT];
static uint16_t s_n;

/* Single-byte results: reads from the main loop are atomic. */
static volatile uint8_t s_duty[IN_COUNT];
static volatile bool    s_active[IN_COUNT];
static volatile bool    s_full[IN_COUNT];

void input_filter_init(void)
{
  s_n = 0u;
  for (uint32_t i = 0u; i < (uint32_t)IN_COUNT; i++)
  {
    s_hi[i] = 0u;
    s_duty[i] = 0u;
    s_active[i] = false;
    s_full[i] = false;
  }
}

void input_filter_sample(void)
{
  const uint32_t idr = IN_GPIO_PORT->IDR;

  for (uint32_t i = 0u; i < (uint32_t)IN_COUNT; i++)
  {
    if ((idr & s_pin[i]) != 0u)
    {
      s_hi[i]++;
    }
  }

  if (++s_n < IN_WINDOW_SAMPLES)
  {
    return;
  }
  s_n = 0u;

  for (uint32_t i = 0u; i < (uint32_t)IN_COUNT; i++)
  {
    const uint8_t d = (uint8_t)(((uint32_t)s_hi[i] * 100u) / IN_WINDOW_SAMPLES);
    s_hi[i] = 0u;
    s_duty[i] = d;

    if (s_active[i])
    {
      if (d <= IN_OFF_PCT)
      {
        s_active[i] = false;
      }
    }
    else if (d >= IN_ON_PCT)
    {
      s_active[i] = true;
    }

    if (s_full[i])
    {
      if (d <= IN_FULL_OFF_PCT)
      {
        s_full[i] = false;
      }
    }
    else if (d >= IN_FULL_ON_PCT)
    {
      s_full[i] = true;
    }
  }
}

bool input_active(input_ch_t ch)
{
  return (ch < IN_COUNT) ? s_active[ch] : false;
}

bool input_full(input_ch_t ch)
{
  return (ch < IN_COUNT) ? s_full[ch] : false;
}

uint8_t input_duty(input_ch_t ch)
{
  return (ch < IN_COUNT) ? s_duty[ch] : 0u;
}
