

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

    struct channel {
        GPIO_TypeDef *gpio;
        int pin;
    };
    struct channel channels[] = {
        {GPIOB, 9},
        {GPIOB, 8},
        {GPIOB, 7},
        {GPIOB, 6}};
#define NCH (sizeof(channels)/sizeof(channels[0]))

    for (int i=0; i<NCH; i++) {
        GPIO_TypeDef *gpio = channels[i].gpio;
        int pin = channels[i].pin;
        if (pin < 8) {
            gpio->CRL &= ~(0xf<<(pin*4));
            gpio->CRL |= (0x8<<(pin*4)); /* input with pull-up/down */
        } else {
            gpio->CRH &= ~(0xf<<((pin-8)*4));
            gpio->CRH |= (0x8<<((pin-8)*4)); /* input with pull-up/down */
        }
        gpio->BSRR |= 1<<16<<pin; /* pull-down */
    }

    TIM1->CR1 = TIM_CR1_CEN;
    TIM1->PSC = 32; /* Count at 1MHz */

    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    uint32_t debounce_low = 0;
    uint32_t debounce_high = 0;
    uint32_t gpio_lastval = 0;
    uint16_t last[NCH] = {0};
    uint16_t diff[NCH] = {0};
    while (42) {
        uint16_t now = TIM1->CNT;
        for (int i=0; i<NCH; i++) {
            uint32_t mask = 1<<i;
            int gpio_val = channels[i].gpio->IDR & (1<<channels[i].pin);
            if (gpio_val && !(gpio_lastval&mask) && debounce_low) {
                gpio_lastval |= mask;
                debounce_high &= ~mask;
                last[i] = now;
            } else if (!gpio_val && (gpio_lastval&mask) && debounce_high) {
                gpio_lastval &= ~mask;
                debounce_low &= ~mask;
                diff[i] = now-last[i];
                last[i] = now;
            }

            if ((uint16_t)(now-last[i]) > 1000)
                debounce_low |= mask;
            if ((uint16_t)(now-last[i]) > 100)
                debounce_high |= mask;
        }
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

