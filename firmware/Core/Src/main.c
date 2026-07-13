/**
 ******************************************************************************
 * @file    main.c
 * @brief   TERA tail light controller -- STM32F411CEU6.
 *
 * Inputs  (opto-isolated, PDM/PWM-tolerant, see input_filter.c):
 *   PB1 running/tail, PB2 brake, PB3 turn, PB4 reverse, PB5 aux/fog
 * Outputs (WS2812B, TIM1 + DMA, see ws2812.c):
 *   PB14 strip 0, PB15 strip 1
 *
 * Behaviour (each strip is laid out with LED 0 at the inboard end):
 *
 *   [ 0 .. N/3 )   reverse zone : white while reverse is active
 *   [ N/3 .. 2N/3) fog zone     : full red while aux/fog is active
 *   [ 2N/3 .. N )  turn zone    : sequential amber sweep while flashing
 *
 *   Base layer under the zones: brake (full red, with a 3-pulse attention
 *   flash on application) > running (dim red) > off.
 *
 * Priority rules:
 *   - Brake overrides running everywhere.
 *   - The turn zone belongs exclusively to the indicator while the vehicle
 *     flasher is cycling (dark between flashes, amber sweep during them),
 *     overriding brake/running on those LEDs only.
 *   - Reverse keeps its white zone even while braking, like a real lamp
 *     cluster with a dedicated reverse window.
 ******************************************************************************
 */
#include "main.h"
#include "input_filter.h"
#include "ws2812.h"

/* Set to 1 if your vehicle shares tail + brake on a single PDM wire into
 * the RUN input (dim PWM = running lights, solid HIGH = brake). PB2 is
 * then ignored. */
#define COMBINED_TAIL_BRAKE_INPUT   0

/* ---- Animation / behaviour tuning -------------------------------------- */
#define FRAME_MS            20u     /* 50 fps animation tick                */
#define BRAKE_PULSES        3u      /* attention flashes on brake apply     */
#define BRAKE_PULSE_MS      60u     /* half-period of one attention flash   */
#define BRAKE_REARM_MS      1500u   /* min release time before re-flashing  */
#define TURN_SWEEP_MS       300u    /* time for the amber sweep to fill     */
#define TURN_HOLD_MS        700u    /* zone stays claimed between flasher
                                       pulses (flasher period is ~660 ms)   */

/* ---- Zones (per strip, LED 0 = inboard end) ----------------------------- */
#define REV_ZONE_START      0u
#define REV_ZONE_LEN        (WS_NUM_LEDS / 3u)
#define FOG_ZONE_START      (WS_NUM_LEDS / 3u)
#define FOG_ZONE_LEN        (WS_NUM_LEDS / 3u)
#define TURN_ZONE_START     (2u * WS_NUM_LEDS / 3u)
#define TURN_ZONE_LEN       (WS_NUM_LEDS - TURN_ZONE_START)

static const ws_color_t COL_OFF   = {  0u,   0u,   0u};
static const ws_color_t COL_RUN   = { 48u,   0u,   0u};   /* dim red    */
static const ws_color_t COL_BRAKE = {255u,   0u,   0u};   /* full red   */
static const ws_color_t COL_AMBER = {255u,  80u,   0u};   /* indicator  */
static const ws_color_t COL_WHITE = {255u, 255u, 255u};   /* reverse    */
static const ws_color_t COL_FOG   = {255u,   0u,   0u};   /* rear fog   */

/* ---- Peripheral handles -------------------------------------------------- */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim1_ch3;

/* ---- Tail light state machine -------------------------------------------- */
typedef enum
{
  TAIL_IDLE,        /* everything dark                          */
  TAIL_RUN,         /* running lights, dim red                  */
  TAIL_BRAKE_PULSE, /* brake just applied: 3 attention flashes  */
  TAIL_BRAKE_HOLD   /* brake held: solid full red               */
} tail_state_t;

static tail_state_t s_state = TAIL_IDLE;
static uint32_t s_brake_edge_ms;    /* when TAIL_BRAKE_PULSE was entered   */
static uint32_t s_brake_release_ms; /* when the brake was last released    */
static bool     s_turn_prev;        /* previous filtered turn input        */
static uint32_t s_turn_edge_ms;     /* last turn rising edge (sweep start) */
static uint32_t s_turn_seen_ms;     /* last time the turn input was high   */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);

static void tail_update(uint32_t now)
{
#if COMBINED_TAIL_BRAKE_INPUT
  const bool run   = input_active(IN_RUN) && !input_full(IN_RUN);
  const bool brake = input_full(IN_RUN);
#else
  const bool run   = input_active(IN_RUN);
  const bool brake = input_active(IN_BRAKE);
#endif

  switch (s_state)
  {
    case TAIL_IDLE:
    case TAIL_RUN:
      if (brake)
      {
        if ((now - s_brake_release_ms) >= BRAKE_REARM_MS)
        {
          s_state = TAIL_BRAKE_PULSE;
          s_brake_edge_ms = now;
        }
        else
        {
          /* Re-applied quickly (ABS pumping, stop-and-go): skip the
           * attention flash and go straight to solid. */
          s_state = TAIL_BRAKE_HOLD;
        }
      }
      else
      {
        s_state = run ? TAIL_RUN : TAIL_IDLE;
      }
      break;

    case TAIL_BRAKE_PULSE:
      if (!brake)
      {
        s_brake_release_ms = now;
        s_state = run ? TAIL_RUN : TAIL_IDLE;
      }
      else if ((now - s_brake_edge_ms) >= (2u * BRAKE_PULSES * BRAKE_PULSE_MS))
      {
        s_state = TAIL_BRAKE_HOLD;
      }
      break;

    case TAIL_BRAKE_HOLD:
      if (!brake)
      {
        s_brake_release_ms = now;
        s_state = run ? TAIL_RUN : TAIL_IDLE;
      }
      break;

    default:
      s_state = TAIL_IDLE;
      break;
  }

  /* Turn overlay bookkeeping. The vehicle's flasher relay does the actual
   * blinking; every rising edge restarts the sweep animation. */
  const bool turn = input_active(IN_TURN);
  if (turn && !s_turn_prev)
  {
    s_turn_edge_ms = now;
  }
  if (turn)
  {
    s_turn_seen_ms = now;
  }
  s_turn_prev = turn;
}

static void tail_render(uint32_t now)
{
  /* Layer 1 -- base: brake > running > off. During the attention pulses
   * the strip alternates full red / dim red at ~8 Hz. */
  ws_color_t base = COL_OFF;
  if (s_state == TAIL_RUN)
  {
    base = COL_RUN;
  }
  else if (s_state == TAIL_BRAKE_HOLD)
  {
    base = COL_BRAKE;
  }
  else if (s_state == TAIL_BRAKE_PULSE)
  {
    const uint32_t phase = (now - s_brake_edge_ms) / BRAKE_PULSE_MS;
    base = ((phase & 1u) == 0u) ? COL_BRAKE : COL_RUN;
  }

  const bool turn = input_active(IN_TURN);
  const bool turn_engaged = turn || ((now - s_turn_seen_ms) < TURN_HOLD_MS);

  for (uint8_t s = 0u; s < WS_NUM_STRIPS; s++)
  {
    ws2812_fill(s, 0u, WS_NUM_LEDS, base);

    /* Layer 2 -- rear fog: brake-intensity red in its own zone. */
    if (input_active(IN_AUX))
    {
      ws2812_fill(s, FOG_ZONE_START, FOG_ZONE_LEN, COL_FOG);
    }

    /* Layer 3 -- reverse: dedicated white zone, visible even while
     * braking (a real cluster keeps its reverse window too). */
    if (input_active(IN_REVERSE))
    {
      ws2812_fill(s, REV_ZONE_START, REV_ZONE_LEN, COL_WHITE);
    }

    /* Layer 4 -- turn: while the flasher is cycling the zone belongs to
     * the indicator alone; amber sweeps outward during the ON phase and
     * the zone is dark during the OFF phase. */
    if (turn_engaged)
    {
      ws2812_fill(s, TURN_ZONE_START, TURN_ZONE_LEN, COL_OFF);
      if (turn)
      {
        uint32_t lit = 1u + ((now - s_turn_edge_ms) * (TURN_ZONE_LEN - 1u))
                            / TURN_SWEEP_MS;
        if (lit > TURN_ZONE_LEN)
        {
          lit = TURN_ZONE_LEN;
        }
        ws2812_fill(s, TURN_ZONE_START, (uint16_t)lit, COL_AMBER);
      }
    }
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();

  ws2812_init();
  input_filter_init();

  /* Unsigned wrap-around makes these "long enough ago" at boot so the
   * first brake application flashes and the turn zone starts released. */
  s_brake_release_ms = 0u - BRAKE_REARM_MS;
  s_turn_seen_ms     = 0u - TURN_HOLD_MS;

  /* Start the 5 kHz input sampling tick. */
  if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t next_frame = HAL_GetTick();
  while (1)
  {
    const uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_frame) >= 0)
    {
      next_frame += FRAME_MS;
      tail_update(now);
      tail_render(now);
      /* If the previous frame is somehow still in flight (never at 50 fps
       * with 24 LEDs), skip this one rather than block. */
      (void)ws2812_show();
    }
    __WFI(); /* sleep; SysTick (1 kHz) or TIM10 wakes us */
  }
}

/* 5 kHz input sampling tick. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10)
  {
    input_filter_sample();
  }
}

/**
 * 25 MHz HSE -> PLL (/25, x200, /2) -> 100 MHz SYSCLK.
 * AHB 100 MHz, APB1 50 MHz (timers 100 MHz), APB2 100 MHz (timers 100 MHz).
 * The LSE is not used by this application (no RTC).
 */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * Opto-isolated inputs on PB1..PB5. No internal pulls: the board already
 * has 10 k pull-downs on every optocoupler emitter. Re-initialising PB3/PB4
 * here also releases them from their JTAG reset function (debugging stays
 * available over SWD on PA13/PA14).
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = IN_RUN_PIN | IN_BRAKE_PIN | IN_TURN_PIN
                      | IN_REVERSE_PIN | IN_AUX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_GPIO_PORT, &GPIO_InitStruct);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

/**
 * TIM1 at 100 MHz, 800 kHz PWM for the WS2812B bit stream. Only the
 * complementary outputs CH2N (PB14) and CH3N (PB15) are enabled, which
 * carry the normal non-inverted waveform when the positive outputs are off.
 */
static void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = WS_TIM_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * TIM10: 100 MHz / 100 / 200 = 5 kHz update interrupt for input sampling.
 */
static void MX_TIM10_Init(void)
{
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  Error_Handler();
}
#endif
