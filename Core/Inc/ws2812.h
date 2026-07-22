#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

/* Number of LEDs on the strip */
#define WS_NUM_LEDS 197

void WS2812_Init(void);
void WS2812_SetBrightness(uint8_t b);                 /* 0..255 global cap  */
void WS2812_SetPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b);
void WS2812_FillRGB(uint8_t r, uint8_t g, uint8_t b); /* set whole strip    */
void WS2812_Clear(void);
void WS2812_Show(void);                               /* push buffer to strip */

#endif /* WS2812_H */
