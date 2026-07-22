/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==========================================================================
 * Inputs: PB3 = Brake, PB5 = Emergency.  PB1/PB2/PB4 unused (set to analog).
 *
 * Both pins are EXTI, rising AND falling edge.  Debouncing is non-blocking
 * and tick-based:
 *   - Any edge that leaves the pin HIGH asserts the channel IMMEDIATELY
 *     (brake lights must react instantly) and refreshes a "last seen high"
 *     timestamp.
 *   - The channel de-asserts only after the pin has stayed LOW for
 *     INPUT_RELEASE_MS with no further edges.
 * That release delay is what swallows optocoupler contact bounce, and it
 * also keeps the output steady if the BCM ever drives a chopped/PWM signal
 * instead of a clean DC level.
 * ========================================================================== */
#define IN_BRAKE_PIN      GPIO_PIN_3
#define IN_EMERGENCY_PIN  GPIO_PIN_5

#define NUM_INPUTS        2
enum { CH_BRAKE = 0, CH_EMERGENCY };

#define INPUT_RELEASE_MS  60u   /* low-quiet time required before turning off */

static const uint16_t input_pins[NUM_INPUTS] = { IN_BRAKE_PIN, IN_EMERGENCY_PIN };

static volatile uint8_t  input_state[NUM_INPUTS]   = {0};  /* debounced result */
static volatile uint32_t last_high_tick[NUM_INPUTS] = {0};

/* Called from the EXTI ISRs (via HAL) on every rising and falling edge. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (int i = 0; i < NUM_INPUTS; i++) {
        if (GPIO_Pin != input_pins[i]) continue;

        if (HAL_GPIO_ReadPin(GPIOB, input_pins[i]) == GPIO_PIN_SET) {
            input_state[i]    = 1;              /* assert immediately */
            last_high_tick[i] = HAL_GetTick();
        }
        /* Falling edges do NOT clear here - Inputs_Service() handles release
           once the line has been quiet long enough. */
    }
}

/* Non-blocking release timer, called from the main loop. */
static void Inputs_Service(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < NUM_INPUTS; i++) {
        if (!input_state[i]) continue;
        if ((now - last_high_tick[i]) < INPUT_RELEASE_MS) continue;

        if (HAL_GPIO_ReadPin(GPIOB, input_pins[i]) == GPIO_PIN_RESET) {
            input_state[i] = 0;
        } else {
            last_high_tick[i] = now;            /* still high, keep it asserted */
        }
    }
}

/* Configure PB3/PB5 as EXTI inputs and park the unused pins in analog mode.
   Done here (not in CubeMX) so regenerating code can't undo it. */
static void Inputs_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB1, PB2, PB4 unused -> analog, lowest leakage */
    g.Pin  = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB3, PB5 -> interrupt on both edges. External 10k pulls them down. */
    g.Pin  = IN_BRAKE_PIN | IN_EMERGENCY_PIN;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);     /* PB3 */
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);   /* PB5 */
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* Seed the state in case a signal is already present at power-up */
    for (int i = 0; i < NUM_INPUTS; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, input_pins[i]) == GPIO_PIN_SET) {
            input_state[i]    = 1;
            last_high_tick[i] = HAL_GetTick();
        }
    }
}

/* ==========================================================================
 * Light sequences.  Strip driver itself lives in ws2812.c (PB15 via SPI2).
 *
 *   Startup   - red sweeps slowly from the middle out to both ends, then the
 *               whole strip settles down to the dim running level.
 *   Running   - solid red at ~8%  (idle state, after startup)
 *   Brake     - solid red at 100%
 *   Emergency - the outer EDGE_COUNT LEDs at each end blink full-red <-> off
 *               on top of whatever the rest of the strip is showing.
 * ========================================================================== */
#define LED_COUNT        WS_NUM_LEDS   /* 144 */
#define EDGE_COUNT       14            /* emergency LEDs at each end        */

#define LEVEL_FULL       255u          /* brake      = 100%                 */
#define LEVEL_RUN        20u           /* running    =  ~8% of full (dim)   */

#define EMERG_HALF_MS    350u          /* flash on/off half-period (~1.4 Hz) */
#define SWEEP_STEP_MS    61u           /* startup sweep pace: 99 steps * 61ms ~= 6 s */

/* --------------------------------------------------------------------------
 * TEMPORARY DIAGNOSTIC.  Set to 0 to get the normal boot animation back.
 *
 * Runs five slow, unmistakable stages so we can see which assumption is
 * wrong. Each stage holds long enough to photograph:
 *   1. everything OFF for 1.5 s   -> anything still lit is NOT being driven
 *   2. first pixel only           -> shows which physical end is index 0
 *   3. centre pixel only          -> should be the exact middle of the strip
 *   4. last pixel only            -> should be the far end
 *   5. every 10th pixel for 5 s   -> count the dots x10 = real strip length
 * ------------------------------------------------------------------------ */
#define STARTUP_DIAG 0

#if STARTUP_DIAG
static void Startup_Diag(void)
{
    WS2812_Clear();
    WS2812_Show();
    HAL_Delay(1500);                                    /* 1 - all off      */

    WS2812_Clear();
    WS2812_SetPixel(0, LEVEL_FULL, 0, 0);
    WS2812_Show();
    HAL_Delay(1500);                                    /* 2 - first pixel  */

    WS2812_Clear();
    WS2812_SetPixel(LED_COUNT / 2u, LEVEL_FULL, 0, 0);
    WS2812_Show();
    HAL_Delay(1500);                                    /* 3 - centre pixel */

    WS2812_Clear();
    WS2812_SetPixel(LED_COUNT - 1u, LEVEL_FULL, 0, 0);
    WS2812_Show();
    HAL_Delay(1500);                                    /* 4 - last pixel   */

    WS2812_Clear();
    for (uint16_t i = 0; i < LED_COUNT; i += 10u) {
        WS2812_SetPixel(i, LEVEL_FULL, 0, 0);
    }
    WS2812_Show();
    HAL_Delay(5000);                                    /* 5 - count marks  */
}
#endif

/* Boot animation: centre -> outwards, then dim to the running level. */
static void Startup_Sequence(void)
{
    /* Works for an odd or an even strip length. With an odd count both
       halves start on the same centre pixel; with an even count they start
       on the two middle pixels. */
    const uint16_t right0 = LED_COUNT / 2u;
    const uint16_t left0  = (LED_COUNT - 1u) / 2u;
    const uint16_t steps  = LED_COUNT - right0;

    WS2812_Clear();
    WS2812_Show();

    for (uint16_t s = 0; s < steps; s++) {
        WS2812_SetPixel((uint16_t)(right0 + s), LEVEL_FULL, 0, 0);
        WS2812_SetPixel((uint16_t)(left0  - s), LEVEL_FULL, 0, 0);
        WS2812_Show();
        HAL_Delay(SWEEP_STEP_MS);
    }

    HAL_Delay(250);                             /* hold full bright a moment */

    for (uint16_t lvl = LEVEL_FULL; lvl > LEVEL_RUN; lvl -= 3u) {
        WS2812_FillRGB((uint8_t)lvl, 0, 0);
        WS2812_Show();
        HAL_Delay(10);
    }

    WS2812_FillRGB(LEVEL_RUN, 0, 0);
    WS2812_Show();
}

/* Paint one frame from the current input state.
   The strip sits behind a red lens, so amber is not reproducible - green is
   absorbed before it leaves the housing. Emergency therefore signals with
   brightness instead of colour: the end sections blink full-red <-> off, so
   they stay obvious whether the base is the 40% running level or full brake.
   (Blinking against the base level would be invisible while braking.) */
static void Render(bool brake, bool emergency, bool emergency_lit)
{
    uint8_t base = brake ? (uint8_t)LEVEL_FULL : (uint8_t)LEVEL_RUN;

    WS2812_FillRGB(base, 0, 0);

    if (emergency) {
        uint8_t edge = emergency_lit ? (uint8_t)LEVEL_FULL : 0u;
        for (uint16_t i = 0; i < EDGE_COUNT; i++) {
            WS2812_SetPixel(i, edge, 0, 0);
            WS2812_SetPixel((uint16_t)(LED_COUNT - 1u - i), edge, 0, 0);
        }
    }

    WS2812_Show();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  WS2812_Init();       /* TIM1_CH2N on PB14 + blank the strip */
  Inputs_Init();       /* PB3 brake, PB5 emergency EXTI       */

  /* The whole board is powered by the car's 12V, so power-up IS the "start"
     event - the animation runs here, once, every time the car switches on.
     Wait first: at cold power-up the LED strip's own 5V rail needs a moment
     to come alive, and if we start clocking data out before then the strip
     misses the sweep and you only ever see the steady running level. */
  HAL_Delay(300);
  Startup_Sequence();  /* red sweep middle -> ends, settle to running level */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Run the non-blocking release timer for the debounced inputs */
    Inputs_Service();

    /* ---- Inputs (PB1, PB2, PB4 unused) ---- */
    bool brake     = input_state[CH_BRAKE];      /* PB3 = brake light      */
    bool emergency = input_state[CH_EMERGENCY];  /* PB5 = emergency/hazard */

    /* Emergency blinks the outer LEDs; the middle of the strip keeps showing
       the running/brake level underneath. */
    bool emergency_lit = ((HAL_GetTick() / EMERG_HALF_MS) % 2u) == 0u;

    /* Only push a new frame when the picture actually changes */
    uint8_t frame = (uint8_t)((brake                    ? 1u : 0u)
                            | (emergency                ? 2u : 0u)
                            | ((emergency && emergency_lit) ? 4u : 0u));
    static uint8_t last_frame = 0xFF;
    if (frame != last_frame) {
        Render(brake, emergency, emergency_lit);
        last_frame = frame;
    }

    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 95;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 99;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB1 PB2 PB3 PB4
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* EXTI vectors. These live here (not stm32f4xx_it.c) because the EXTI lines
   are configured in code rather than in CubeMX, so CubeMX never generates
   them. HAL_GPIO_EXTI_IRQHandler clears the flag and calls our callback. */
void EXTI3_IRQHandler(void)     /* PB3 = brake */
{
    HAL_GPIO_EXTI_IRQHandler(IN_BRAKE_PIN);
}

void EXTI9_5_IRQHandler(void)   /* PB5 = emergency (line 5 of the 5..9 group) */
{
    HAL_GPIO_EXTI_IRQHandler(IN_EMERGENCY_PIN);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
