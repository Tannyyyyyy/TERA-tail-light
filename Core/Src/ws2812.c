#include "main.h"
#include "ws2812.h"

/* ------------------------------------------------------------------------
 * WS2812B driver for STM32F411, data on PB14, bare-metal (no CubeMX needed).
 *
 * PB14 has no SPI MOSI function on this part (it is SPI2_MISO), so the strip
 * is driven with TIM1_CH2N PWM + DMA instead:
 *
 *   TIM1 runs at 96 MHz, ARR = 119  ->  1.25 us per WS2812 bit.
 *   DMA2 Stream2 Ch6 feeds TIM1->CCR2 with one high-time per bit:
 *       '0' -> 38 ticks (0.40 us high)
 *       '1' -> 76 ticks (0.79 us high)
 *
 * CH2N is a COMPLEMENTARY output - it is the inverse of OCREF. Rather than
 * inverting every value in software, the channel runs in PWM mode 2, whose
 * OCREF is already inverted, so CCR2 is the wanted high time directly.
 *
 * Trailing zero entries hold the line low to latch the frame (>50 us).
 * ---------------------------------------------------------------------- */

#define WS_BITS_PER_LED  24
#define WS_RESET_SLOTS   48                 /* 48 * 1.25us = 60us latch     */
#define WS_BUF_LEN       (WS_NUM_LEDS * WS_BITS_PER_LED + WS_RESET_SLOTS)

#define WS_ARR           119u               /* 96 MHz / 120 = 800 kHz       */
#define WS_T0H           38u                /* 0.40 us                      */
#define WS_T1H           76u                /* 0.79 us                      */

static uint8_t  grb[WS_NUM_LEDS][3];        /* per-LED colour, GRB order    */
static uint16_t ws_buf[WS_BUF_LEN];         /* one CCR value per bit        */
static uint8_t  s_bright = 96;              /* brightness cap (limit current) */

/* ---- TIM1_CH2N on PB14 + DMA2 Stream2, configured by hand ---- */
static void ws_hw_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* PB14 -> AF1 (TIM1_CH2N), push-pull, very high speed */
    g.Pin       = GPIO_PIN_14;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB15 is not used by the strip; park it so the timer cannot drive it */
    g.Pin  = GPIO_PIN_15;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g);

    /* 800 kHz bit clock */
    TIM1->CR1  = 0;
    TIM1->PSC  = 0;
    TIM1->ARR  = WS_ARR;
    TIM1->CCR2 = 0;

    /* CH2: PWM mode 2 (0b111) + output compare preload */
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
    TIM1->CCMR1 |= (7u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    /* enable the complementary output, active high */
    TIM1->CCER &= ~(TIM_CCER_CC2NP | TIM_CCER_CC2E);
    TIM1->CCER |= TIM_CCER_CC2NE;

    TIM1->BDTR |= TIM_BDTR_MOE;             /* advanced timer main output   */
    TIM1->DIER |= TIM_DIER_CC2DE;           /* CC2 raises a DMA request     */
    TIM1->EGR   = TIM_EGR_UG;               /* load PSC/ARR/preload         */
}

/* Blocking DMA burst of the whole frame. ~4.4 ms for 144 LEDs. */
static void ws_send(void)
{
    DMA_Stream_TypeDef *st = DMA2_Stream2;  /* Ch6 = TIM1_CH2 */

    st->CR = 0;
    while (st->CR & DMA_SxCR_EN) { }

    /* clear stream 2 interrupt flags (low ISR register, bits 16..21) */
    DMA2->LIFCR = 0x3Fu << 16;

    st->PAR  = (uint32_t)&TIM1->CCR2;
    st->M0AR = (uint32_t)ws_buf;
    st->NDTR = WS_BUF_LEN;
    st->CR   = (6u << DMA_SxCR_CHSEL_Pos)   /* channel 6                  */
             | (1u << DMA_SxCR_DIR_Pos)     /* memory -> peripheral       */
             | (1u << DMA_SxCR_PSIZE_Pos)   /* 16-bit peripheral          */
             | (1u << DMA_SxCR_MSIZE_Pos)   /* 16-bit memory              */
             | DMA_SxCR_MINC
             | (2u << DMA_SxCR_PL_Pos);     /* high priority              */
    st->CR  |= DMA_SxCR_EN;

    TIM1->CNT  = 0;
    TIM1->CR1 |= TIM_CR1_CEN;

    while (!(DMA2->LISR & DMA_LISR_TCIF2)) { }   /* wait for frame out */

    TIM1->CR1 &= ~TIM_CR1_CEN;
    st->CR     = 0;
    TIM1->CCR2 = 0;                         /* leave the line low */
}

/* one colour byte -> 8 CCR values, MSB first */
static void encode(uint8_t v, uint16_t *dst)
{
    for (int i = 0; i < 8; i++) {
        dst[i] = (v & (uint8_t)(0x80u >> i)) ? (uint16_t)WS_T1H
                                             : (uint16_t)WS_T0H;
    }
}

void WS2812_Init(void)                 { ws_hw_init(); WS2812_Clear(); WS2812_Show(); }
void WS2812_SetBrightness(uint8_t b)   { s_bright = b; }

void WS2812_SetPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b)
{
    if (i >= WS_NUM_LEDS) return;
    grb[i][0] = (uint8_t)((uint16_t)g * s_bright / 255u);
    grb[i][1] = (uint8_t)((uint16_t)r * s_bright / 255u);
    grb[i][2] = (uint8_t)((uint16_t)b * s_bright / 255u);
}

void WS2812_FillRGB(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < WS_NUM_LEDS; i++) WS2812_SetPixel(i, r, g, b);
}

void WS2812_Clear(void) { WS2812_FillRGB(0, 0, 0); }

void WS2812_Show(void)
{
    uint32_t k = 0;
    for (uint16_t i = 0; i < WS_NUM_LEDS; i++) {
        encode(grb[i][0], &ws_buf[k]); k += 8;
        encode(grb[i][1], &ws_buf[k]); k += 8;
        encode(grb[i][2], &ws_buf[k]); k += 8;
    }
    while (k < WS_BUF_LEN) ws_buf[k++] = 0;         /* reset / latch gap */
    ws_send();
}
