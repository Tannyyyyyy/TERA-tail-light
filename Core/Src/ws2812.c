#include "main.h"
#include "ws2812.h"

/* ------------------------------------------------------------------------
 * WS2812B driver for STM32F411 using SPI2 (MOSI = PB15), bare-metal.
 *
 * Each WS2812 bit is sent as 4 SPI bits at 3 MHz (one SPI bit = 0.333 us):
 *     '0' -> 1000   (high ~0.33 us, low ~1.0 us)
 *     '1' -> 1100   (high ~0.67 us, low ~0.67 us)
 * SPI2 clock = APB1 (48 MHz) / 16 = 3.0 MHz.  Colour order is GRB.
 *
 * Only MOSI is driven; in master mode the SPI clock runs internally, so the
 * SCK pin is not needed. A steady frame + a slow flash means we can use a
 * simple blocking send (no DMA needed).
 * ---------------------------------------------------------------------- */

#define WS_BYTES_PER_LED 12                 /* 3 colours * 4 SPI bytes      */
#define WS_RESET_BYTES   40                 /* trailing zeros => >50us reset */
#define WS_BUF_LEN       (WS_NUM_LEDS * WS_BYTES_PER_LED + WS_RESET_BYTES)

static uint8_t grb[WS_NUM_LEDS][3];         /* per-LED colour, GRB order    */
static uint8_t ws_buf[WS_BUF_LEN];          /* encoded SPI stream           */
static uint8_t s_bright = 96;               /* brightness cap (limit current) */

/* ---- bare-metal SPI2 (no HAL SPI module required) ---- */
static void spi2_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PB15 -> AF5 (SPI2_MOSI), push-pull, very high speed */
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_15;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);

    /* master, 8-bit, MSB first, fPCLK1/16, CPOL=0/CPHA=0, software NSS high */
    SPI2->CR1 = SPI_CR1_MSTR
              | (0x3u << SPI_CR1_BR_Pos)            /* baud = fPCLK1 / 16 */
              | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI2->CR2 = 0;
    SPI2->CR1 |= SPI_CR1_SPE;
}

static void spi2_tx(const uint8_t *d, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        while (!(SPI2->SR & SPI_SR_TXE)) { }
        *(volatile uint8_t *)&SPI2->DR = d[i];
    }
    while (!(SPI2->SR & SPI_SR_TXE)) { }
    while (SPI2->SR & SPI_SR_BSY)    { }
}

/* one colour byte -> 4 SPI bytes (MSB first) */
static void encode(uint8_t v, uint8_t *dst)
{
    static const uint8_t code[2] = { 0x8u, 0xCu };  /* 1000 , 1100 */
    for (int i = 0; i < 4; i++) {
        uint8_t hi = (uint8_t)((v >> (7 - 2 * i)) & 1u);
        uint8_t lo = (uint8_t)((v >> (6 - 2 * i)) & 1u);
        dst[i] = (uint8_t)((code[hi] << 4) | code[lo]);
    }
}

void WS2812_Init(void)                 { spi2_init(); WS2812_Clear(); WS2812_Show(); }
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
        encode(grb[i][0], &ws_buf[k]); k += 4;
        encode(grb[i][1], &ws_buf[k]); k += 4;
        encode(grb[i][2], &ws_buf[k]); k += 4;
    }
    while (k < WS_BUF_LEN) ws_buf[k++] = 0x00;      /* reset / latch gap */
    spi2_tx(ws_buf, WS_BUF_LEN);
}
