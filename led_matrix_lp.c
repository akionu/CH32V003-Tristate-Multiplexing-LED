/*
 * CH32V003J4M6 (SOP-8) drives 5x6 LED matrix
 *
 * Reference:
 *  - https://github.com/cnlohr/ch32v003fun/tree/master/examples/systick_irq
 *  - http://www.technoblogy.com/show?2H0K
 *  - https://en.wikipedia.org/wiki/Charlieplexing
 *
 * Aug 2023 by Li Mingjie
 *  - Email:  limingjie@outlook.com
 *  - GitHub: https://github.com/limingjie/
 */

#include <string.h>  // memcpy

#include "ch32v003fun.h"
#include "ch32v003_GPIO_branchless.h"
#include "fonts.h"

// Bit definitions for systick regs
#define SYSTICK_SR_CNTIF   (1 << 0)
#define SYSTICK_CTLR_STE   (1 << 0)
#define SYSTICK_CTLR_STIE  (1 << 1)
#define SYSTICK_CTLR_STCLK (1 << 2)
#define SYSTICK_CTLR_STRE  (1 << 3)
#define SYSTICK_CTLR_SWIE  (1 << 31)

uint8_t pins[LED_MATRIX_NUM_PINS] = {
  GPIOv_from_PORT_PIN(GPIO_port_C, 1),  // IO1
  GPIOv_from_PORT_PIN(GPIO_port_C, 2),  // IO2
  GPIOv_from_PORT_PIN(GPIO_port_C, 3),  // IO3
  GPIOv_from_PORT_PIN(GPIO_port_C, 4),  // IO4
  GPIOv_from_PORT_PIN(GPIO_port_C, 5),  // IO5
  GPIOv_from_PORT_PIN(GPIO_port_C, 6),  // IO6
};

// PWM duty cycles of LEDs
uint8_t led_duty_cycles[LED_MATRIX_SIZE];

static inline void led_matrix_run();

//#define BOARD 0

// Start up the SysTick IRQ
void systick_init(void)
{
  // Disable default SysTick behavior
  SysTick->CTLR = 0;

  // Enable the SysTick IRQ
  NVIC_EnableIRQ(SysTicK_IRQn);

  // Set the tick interval
  SysTick->CMP = (FUNCONF_SYSTEM_CORE_CLOCK / 50000) - 1;

  // Start at zero
  SysTick->CNT = 0;

  // Enable SysTick counter, IRQ, HCLK/1
  SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;
}

// SysTick ISR just counts ticks
__attribute__((interrupt)) void SysTick_Handler(void)
{
  // Move the compare further ahead in time. as a warning, if more than this length of time
  // passes before triggering, you may miss your interrupt.
  SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK / 50000);

  // Clear IRQ
  SysTick->SR = 0;

  led_matrix_run();
}

static inline void led_matrix_init()
{
  //    GPIO_port_enable(GPIO_port_A);
  GPIO_port_enable(GPIO_port_C);
  //    GPIO_port_enable(GPIO_port_D);

  for (uint8_t i = 0; i < LED_MATRIX_NUM_PINS; i++)
  {
    GPIO_pinMode(pins[i], GPIO_pinMode_I_floating, GPIO_Speed_10MHz);
  }
}

static inline void led_matrix_run()
{
  static uint8_t  cycle = 0, row = 0, column = 0;
  static uint8_t *led = led_duty_cycles;

  // Turn off the LED by put column pin in high impedance mode.
  // Put it at the beginning to avoid blink on cycle 0, although not noticeable.
  if (cycle == *led)
  {
    GPIO_pinMode(pins[column], GPIO_pinMode_I_floating, GPIO_Speed_10MHz);
  }
  else if (cycle == 0)  // Initialization or moved to the next LED.
  {
    // Pull down the row pin and pull up the column pin to turn on the LED.
    GPIO_pinMode(pins[row], GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
    GPIO_digitalWrite(pins[row], low);
    GPIO_pinMode(pins[column], GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
    GPIO_digitalWrite(pins[column], high);
  }

  // Move to the next LED after LED_PWM_CYCLES
  if (++cycle == LED_PWM_CYCLES)
  {
    cycle = 0;
    led++;

    // Put the current column pin in high impedance mode.
    GPIO_pinMode(pins[column], GPIO_pinMode_I_floating, GPIO_Speed_10MHz);

    // The column and row cannot be the same pin.
    if (++column == row)
    {
      column++;
    }

    // Move to the next row.
    if (column == LED_MATRIX_NUM_PINS)
    {
      column = 0;

      // Put the current row pin in high impedance mode.
      GPIO_pinMode(pins[row], GPIO_pinMode_I_floating, GPIO_Speed_10MHz);

      // Reset to the first LED.
      if (++row == LED_MATRIX_NUM_PINS)
      {
        row    = 0;
        column = 1;
        led    = led_duty_cycles;
      }
    }
  }
}

void led_putchar(uint8_t c)
{
  uint32_t ch  = font[c - 27];
  uint8_t *led = led_duty_cycles;
  for (uint8_t line = 0; line < LED_MATRIX_NUM_PINS; line++)
  {
    for (uint8_t pixel = 0; pixel < LED_MATRIX_NUM_PINS - 1; pixel++)
    {
      *led++ = (ch & 0x01) ? 16 : 0;
      ch >>= 1;
    }
  }
}

void led_show_array(const char *arr, uint8_t size)
{
  for (uint8_t i = 0; i < size; i++)
  {
    led_putchar(arr[i]);
    Delay_Ms(300);
  }
}

static inline void set_effect(uint8_t i)
{
  memcpy(led_duty_cycles, effects[i], LED_MATRIX_SIZE * sizeof(uint8_t));
}


static inline void sleep_intr_init() {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;
  // GPIOA: Set to output
//  GPIOA->CFGLR = ((GPIO_CNF_OUT_PP | GPIO_Speed_2MHz)<<(4*2)) |
//    ((GPIO_CNF_OUT_PP | GPIO_Speed_2MHz)<<(4*1));
//  GPIOA->BSHR = GPIO_BSHR_BS2 | GPIO_BSHR_BR1;
  // GPIOC: Set to input with mixed pull-up / pull-down
//  GPIOC->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) |
//    (GPIO_CNF_IN_PUPD<<(4*6)) |
//    (GPIO_CNF_IN_PUPD<<(4*5)) |
//    (GPIO_CNF_IN_PUPD<<(4*4)) |
//    (GPIO_CNF_IN_PUPD<<(4*3)) |
//    (GPIO_CNF_IN_PUPD<<(4*2)) |
//    (GPIO_CNF_IN_PUPD<<(4*1)) |
//    (GPIO_CNF_IN_PUPD<<(4*0));
//  GPIOC->BSHR = GPIO_BSHR_BS7 |
//    GPIO_BSHR_BR6 |
//    GPIO_BSHR_BS5 |
//    GPIO_BSHR_BR4 |
//    GPIO_BSHR_BS3 |
//    GPIO_BSHR_BR2 |
//    GPIO_BSHR_BS1 |
//    GPIO_BSHR_BR0;
  // GPIOD: D2 set to input pull-up
  GPIOD->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) |
    (GPIO_CNF_IN_PUPD<<(4*6)) |
    (GPIO_CNF_IN_PUPD<<(4*5)) |
    (GPIO_CNF_IN_PUPD<<(4*4)) |
    (GPIO_CNF_IN_PUPD<<(4*3)) |
    (GPIO_CNF_IN_PUPD<<(4*2)) |
    (GPIO_CNF_IN_PUPD<<(4*0));
  GPIOD->BSHR = GPIO_BSHR_BR7 |
    GPIO_BSHR_BS6 |
    GPIO_BSHR_BR5 |
    GPIO_BSHR_BS4 |
    GPIO_BSHR_BR3 |
    GPIO_BSHR_BS2 |
    GPIO_BSHR_BR0;

  // AFIO is needed for EXTI
  RCC->APB2PCENR |= RCC_AFIOEN;

  // assign pin 2 interrupt from portD (0b11) to EXTI channel 2
  AFIO->EXTICR |= (uint32_t)(0b11 << (2*2));

  // enable line2 interrupt event
  EXTI->EVENR |= EXTI_Line2;
  EXTI->FTENR |= EXTI_Line2;

  // select standby on power-down
  PWR->CTLR |= PWR_CTLR_PDDS;

  // peripheral interrupt controller send to deep sleep
  PFIC->SCTLR |= (1 << 2);
}

int main()
{
  SystemInit();
  Delay_Ms(5000);

  // configure deep sleep
  sleep_intr_init();

  // Init LED matrix
  led_matrix_init();

  // Init systick
  systick_init();

  // wake up 
  const char *start = "\x1c\x1d\x1e\x1f";
  led_show_array(start, strlen(start));
  const char *count_down = "543210";

  while (1)
  {
    // kill irq before deep sleep
    NVIC_DisableIRQ(SysTicK_IRQn); 
    // put off all LEDs
    for (uint8_t i = 0; i < LED_MATRIX_NUM_PINS; i++) {
      GPIO_pinMode(pins[i], GPIO_pinMode_I_pullDown, GPIO_Speed_10MHz);
    }
    // GPIO C0 Push-Pull LOW
	  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
	  GPIOC->CFGLR &= ~(0xf<<(4*0));
	  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);
		GPIOC->BSHR = (1<<16); // LOW
    // dive into deep sleep
    __WFE();


    SystemInit();

	  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
	  GPIOC->CFGLR &= ~(0xf<<(4*0));
	  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);
		GPIOC->BSHR = (1<<0); // HIGH
    Delay_Ms(100);

    led_matrix_init();
    systick_init();

    led_show_array(count_down, strlen(count_down));

    // Run effects
    for (uint8_t e = 0; e < sizeof(effects) / sizeof(effects[0]); e++)
    {
      set_effect(e);
      for (uint8_t loop = 0; loop < LED_MATRIX_SIZE; loop++)
      {
        // Shuffle the LED duty cycles
        uint8_t t = led_duty_cycles[0];
        for (uint8_t i = 0; i < LED_MATRIX_SIZE - 1; i++)
        {
          led_duty_cycles[i] = led_duty_cycles[i + 1];
        }
        led_duty_cycles[LED_MATRIX_SIZE - 1] = t;

        Delay_Ms(50);
      }

    }

    const char msg[] = "HelloWorld!GoodNight!\x1b\x1b\x1b\x1b\x1b";
    // \x1b == heart
    for (uint8_t i = 0; i < sizeof(msg) / sizeof(msg[0]); i++)
    {
      led_putchar(msg[i]);
      Delay_Ms(150);
    }
    const char *end = "\x1f\x1e\x1d\x1c";
    led_show_array(end, strlen(end));
  }
}
