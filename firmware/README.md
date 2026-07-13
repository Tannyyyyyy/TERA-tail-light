# TERA tail light — firmware

Custom automotive tail light controller for the **STM32F411CEU6**.

- **Inputs (opto-isolated, PC817 → 10 k pull-down, HIGH = signal present):**
  `PB1` running/tail · `PB2` brake · `PB3` turn · `PB4` reverse · `PB5` aux/fog.
  The vehicle side may be PDM/PWM — all inputs go through a duty-cycle filter
  (`input_filter.c`), never a raw GPIO read.
- **Outputs:** two WS2812B data lines, `PB14` (strip 0, TIM1_CH2N) and
  `PB15` (strip 1, TIM1_CH3N), driven by TIM1 PWM + DMA.

> **Assumption:** the output connector routes **5 V + GND** next to PB14/PB15,
> which matches 5 V addressable LEDs, so this firmware drives **WS2812B**
> strips. If your lamp actually uses 12 V LEDs behind MOSFETs instead, keep
> everything except `ws2812.c`: configure TIM1 CH2N/CH3N as ordinary ~1 kHz
> PWM (prescaler 99, period 999) and replace `ws2812_show()` with two
> `__HAL_TIM_SET_COMPARE()` calls derived from the same state machine.

## 1. STM32CubeMX configuration guide

Start a project for `STM32F411CEU6` and configure:

### System Core

| Setting | Value | Why |
|---|---|---|
| **SYS → Debug** | **Serial Wire** | *Required.* Frees PB3 (JTDO) and PB4 (NJTRST) for the turn/reverse inputs. JTAG must stay disabled; debugging uses SWD on PA13/PA14. |
| **RCC → HSE** | Crystal/Ceramic Resonator | 25 MHz crystal |
| **RCC → LSE** | Disabled (or Crystal if you later want the RTC) | Not used by this firmware |

### Clock tree

| Field | Value |
|---|---|
| PLL Source | HSE (25 MHz) |
| PLLM | /25 |
| PLLN | ×200 |
| PLLP | /2 |
| SYSCLK / HCLK | **100 MHz** |
| APB1 prescaler | /2 (PCLK1 = 50 MHz) |
| APB2 prescaler | /1 (PCLK2 = 100 MHz) |
| Flash latency | 3 WS (set automatically) |

Both timer kernels then run at 100 MHz, which the WS2812 bit timing in
`ws2812.h` and the TIM10 sampling tick depend on.

### GPIO

| Pin | Mode | Pull | Notes |
|---|---|---|---|
| PB1, PB2, PB3, PB4, PB5 | Input | **None** | Board already has 10 k pull-downs on the opto emitters |
| PB14 | TIM1_CH2N (AF1) | **Pull-down** | Keeps the WS2812 data line low between frames |
| PB15 | TIM1_CH3N (AF1) | Pull-down | Speed "High" is sufficient |

PB2 doubles as **BOOT1**: with BOOT0 strapped low the MCU always boots from
flash, so a brake signal present at power-up is harmless — just don't strap
BOOT0 high in-vehicle.

### TIM1 — WS2812 bit clock (800 kHz)

- Channel 2: **PWM Generation CH2N**, Channel 3: **PWM Generation CH3N**
- Prescaler `0`, Counter Period `124` (→ 100 MHz / 125 = 800 kHz), PWM mode 1,
  pulse 0, auto-reload preload **enabled**
- **DMA Settings tab:** add `TIM1_CH2` (DMA2 Stream 2) and `TIM1_CH3`
  (DMA2 Stream 6), direction *Memory To Peripheral*, data width
  **Half Word / Half Word**, mode *Normal*, memory increment on
- NVIC: enable **DMA2 stream2** and **DMA2 stream6** global interrupts

### TIM10 — input sampling tick (5 kHz)

- Activated, Prescaler `99`, Counter Period `199`
  (100 MHz / 100 / 200 = 5 kHz)
- NVIC: enable **TIM1 update interrupt and TIM10 global interrupt**

### Project Manager

- Toolchain of your choice (STM32CubeIDE / Makefile / CMake)
- "Copy only the necessary library files"

## 2. Dropping in this source

After generating, replace/add these files (they are complete, self-contained
versions of what CubeMX generates plus the application):

```
Core/Inc/main.h              pin map
Core/Inc/ws2812.h            LED driver API + bit timing
Core/Inc/input_filter.h      PDM filter API + thresholds
Core/Inc/stm32f4xx_it.h      ISR prototypes
Core/Src/main.c              init, state machine, animations, main loop
Core/Src/ws2812.c            WS2812B TIM1+DMA driver
Core/Src/input_filter.c      5 kHz sampling, duty measurement, hysteresis
Core/Src/stm32f4xx_it.c      ISRs (SysTick, DMA2 S2/S6, TIM1_UP_TIM10)
Core/Src/stm32f4xx_hal_msp.c clocks, AF pins, DMA wiring, NVIC
```

Keep the CubeMX-generated `stm32f4xx_hal_conf.h`, `system_stm32f4xx.c`,
startup file and linker script. Required HAL modules: `TIM`, `DMA`, `GPIO`,
`RCC`, `PWR`, `FLASH`, `CORTEX` (all enabled automatically by the
configuration above).

## 3. How it works

```
              5 kHz TIM10 IRQ                     50 fps main loop
 PB1..PB5 ──> input_filter_sample() ──┐   ┌─> tail_update()  (state machine)
              20 ms duty windows      ├──>│   tail_render()  (layered zones)
              + hysteresis            ┘   └─> ws2812_show()  (TIM1 DMA kick)
```

### PDM/PWM input filtering

Body control modules routinely PWM their lamp outputs, so each input's duty
cycle is measured over a 20 ms window and pushed through two hysteresis
comparators:

- `input_active(ch)` — duty ≥ 6 % (turns off below 3 %): "the function is on",
  at any dimming level. A 10 % PDM running-light feed reads as solidly active.
- `input_full(ch)` — duty ≥ 85 % (clears below 70 %): "the wire is solid 12 V".

If your car shares tail + brake on **one** wire (dim PWM = tail, solid =
brake), feed it to PB1 and set `COMBINED_TAIL_BRAKE_INPUT 1` in `main.c`;
PB2 is then ignored. If your body controller PWMs slower than ~100 Hz,
enlarge `IN_WINDOW_SAMPLES` so a window still spans a full PWM period.

### State machine and priorities

Base layer state machine (whole strip): `IDLE` → `RUN` (dim red) →
`BRAKE_PULSE` (3 × ~8 Hz full-red attention flashes on brake application) →
`BRAKE_HOLD` (solid full red). A brake re-applied within 1.5 s skips the
flashes (ABS pumping / stop-and-go shouldn't strobe).

Overlays, drawn over the base in order (LED 0 = inboard end of each strip):

| Zone | LEDs | Function | Priority behaviour |
|---|---|---|---|
| Reverse | first third | solid white while reverse active | kept visible even while braking |
| Fog | middle third | full red while aux active | brake-intensity, steady |
| Turn | outer third | sequential amber sweep (300 ms fill) | owns its zone exclusively while the vehicle flasher cycles — dark between flashes, overriding brake/run there |

The vehicle's flasher relay does the blinking; each rising edge of the
filtered turn input restarts the sweep, and the zone is released 700 ms
after the last pulse.

### Tuning

Everything lives in two places:

- `ws2812.h` — `WS_NUM_LEDS` (per strip), bit timing, reset time
- `main.c` — colors (`COL_*`), zone boundaries, flash/sweep timing,
  `COMBINED_TAIL_BRAKE_INPUT`; `ws2812_set_brightness()` gives a global
  current-limiting dimmer that doesn't touch the colors

## 4. Hardware notes

- **Level shifting:** WS2812B V_IH is 0.7 × VDD = 3.5 V at a 5 V supply, so
  3.3 V data from PB14/PB15 is marginal. Use a 74AHCT125/74HCT245 as a
  3.3 V → 5 V shifter near the MCU, plus the usual ~300 Ω series resistor and
  1000 µF across 5 V/GND at the strip head.
- **Power budget:** full-white draws up to 60 mA/LED — 24 LEDs × 2 strips ≈
  2.9 A worst case. The default animations only hit white in the reverse zone,
  but size the 5 V supply for the worst case or lower
  `ws2812_set_brightness()`.
- **Grounding:** MCU GND, strip GND and the optocoupler emitter pull-downs
  must share a common ground; the opto inputs are otherwise fully isolated
  from the 12 V vehicle side.
- **Legal:** flashing brake lights and dynamic indicators are not road-legal
  in every jurisdiction — set `BRAKE_PULSES 0` for a plain solid brake light.

## 5. Verification status

- All sources compile clean (`-Wall -Wextra`, no warnings) against the
  current ST `stm32f4xx_hal_driver` + CMSIS headers.
- `input_filter.c` is host-unit-tested: 10 % PWM → steady *active*,
  solid → *full*, both hysteresis bands, and channel independence.
- DMA stream/channel mapping (DMA2 S2/CH6 = TIM1_CH2, S6/CH6 = TIM1_CH3) and
  the complementary-output behaviour (CHxN alone carries non-inverted PWM)
  verified against RM0383 and the HAL sources.
- Not yet run on hardware — the WS2812 bit timing (40/80 of 125 ticks) is
  the standard 800 kHz recipe but deserves a scope check on first power-up.
