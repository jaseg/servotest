

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#include <stm32f1xx.h>
#pragma GCC diagnostic pop

#include <system_stm32f1xx.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

/* 
 * Part number: STM32F030F4C6
 */

static volatile unsigned int sys_time;

static GPIO_TypeDef *gpiob = GPIOB;
static TIM_TypeDef *tim1 = TIM1;

int main(void) {
    /* External crystal: 8MHz */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));

    /* PLL HSE pre :2, PLL x8 -> 8/2*8 = 32.0MHz */
    /* AHB/APB1/APB2 not divided, so also 32.0MHz */
    RCC->CFGR &= ~RCC_CFGR_PLLMULL_Msk & ~RCC_CFGR_SW_Msk & ~RCC_CFGR_PPRE1_Msk & ~RCC_CFGR_PPRE2_Msk & ~RCC_CFGR_HPRE_Msk;
    RCC->CFGR |= (6<<RCC_CFGR_PLLMULL_Pos) | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));

    /* Switch to PLL */
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    SystemCoreClockUpdate();

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOB->CRH &=
          ~GPIO_CRH_CNF9_Msk;
    GPIOB->CRH |=
          (2<<GPIO_CRH_CNF9_Pos); /* input with pull-up/down */
    GPIOB->BSRR |=
          GPIO_BSRR_BR9; /* pull-down */

    TIM1->CR1 = TIM_CR1_CEN;
    TIM1->PSC = 32; /* Count at 1MHz */

    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    int gpio_lastval = 0;
    bool debounce_low = false;
    bool debounce_high = false;
    while (42) {
        static uint16_t start __attribute__((used));
        static uint16_t end __attribute__((used));
        static uint16_t diff __attribute__((used));
        int gpio_val = GPIOB->IDR & (1<<9);
        uint16_t now = TIM1->CNT;
        if (gpio_val && !gpio_lastval && debounce_low) {
            gpio_lastval = gpio_val;
            start = now;
            debounce_high = false;
        } else if (!gpio_val && gpio_lastval && debounce_high) {
            gpio_lastval = gpio_val;
            end = now;
            diff = end-start;
            debounce_low = false;
        }

        if ((uint16_t)(now-end) > 1000)
            debounce_low = true;
        if ((uint16_t)(now-start) > 100)
            debounce_high = true;
    }
}

void NMI_Handler(void) {
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler() {
    asm volatile ("bkpt");
}

void SVC_Handler(void) {
}


void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
    sys_time++;
}

void _init(void) {
}

