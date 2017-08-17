

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
static SysTick_Type *systick = SysTick;

enum GpioIndex {
    IDX_GPIOA,
    IDX_GPIOB,
    NGPIOS
};

struct channel {
    enum GpioIndex gpio;
    int pin;
};
static struct channel channels[] = {
    {IDX_GPIOB, 9},
    {IDX_GPIOB, 8},
    {IDX_GPIOB, 7},
    {IDX_GPIOB, 6},
    {IDX_GPIOB, 5}};
#define NCH (sizeof(channels)/sizeof(channels[0]))

static struct channel channels_out[] = {
    {IDX_GPIOA, 0},
    {IDX_GPIOA, 1},
    {IDX_GPIOA, 2},
    {IDX_GPIOA, 3},
    {IDX_GPIOA, 4},
    {IDX_GPIOA, 5},
    {IDX_GPIOA, 6},
    {IDX_GPIOA, 7},
    {IDX_GPIOA, 8},
    {IDX_GPIOA, 9},
    {IDX_GPIOA, 10},
    {IDX_GPIOA, 11}};
#define NCH_OUT (sizeof(channels_out)/sizeof(channels_out[0]))

static uint16_t duration_out_a[NCH_OUT] = {[0 ... (NCH_OUT-1)]=1500};
static uint16_t duration_out_b[NCH_OUT] = {[0 ... (NCH_OUT-1)]=1500};
static uint16_t *duration_out_read = duration_out_a;
static uint16_t *duration_out_write = duration_out_b;
enum {
    OUT_UPDATE_RQ,
    OUT_UPDATE_DONE
} out_comm;
/* Pad with 0xFFFF in the end to nicely terminate CCR interrupt chain after last channel */
static uint16_t sorted_out[NCH_OUT+1] = {[0 ... (NCH_OUT-1)]=0, [NCH_OUT]=0xFFFF};
static uint32_t setmask[NCH_OUT];
static uint32_t resetmask;

void calculate_timechain(void) {
    if (out_comm != OUT_UPDATE_RQ)
        return;
    
    uint16_t *tmp = duration_out_read;
    duration_out_read = duration_out_write;
    duration_out_write = tmp;
    for (int i=0; i<NCH_OUT; i++)
        duration_out_write[i] = duration_out_read[i];
    out_comm = OUT_UPDATE_DONE;

    uint8_t idxs[NCH_OUT];
    for (int i=0; i<NCH_OUT-1; i++)
        idxs[i] = i;
    for (int i=0; i<NCH_OUT; i++) {
        int min_j = i;
        int min_e = duration_out_read[idxs[i]];
        for (int j=i+1; j<NCH_OUT; j++) {
            if (duration_out_read[j] < min_e) {
                min_e = duration_out_read[idxs[j]];
                min_j = j;
            }
        }
        uint8_t tmp = idxs[min_j];
        idxs[min_j] = idxs[i];
        idxs[i] = tmp;
    }

    int last_duration = -1;
    int out_idx = -1;
    for (int i=0; i<NCH_OUT; i++) {
        int idx = idxs[i];
        int duration = duration_out_read[idx];
        if (duration != last_duration) {
            out_idx ++;
            setmask[out_idx] = 0;
            sorted_out[out_idx] = duration;
            last_duration = duration;
        }
        int pin = channels_out[idx].pin;
        int gpio_mask = 1<<pin<<(16*channels_out[idx].gpio);
        resetmask |= gpio_mask;
        setmask[out_idx] |= gpio_mask;
    }
    /* No need to clear later channels */
}

static volatile int out_channel_active;
void TIM1_TRG_COM_IRQHandler(void) {
    out_channel_active = 0;
    GPIOA->BSRR = resetmask&0xFFFF;
    GPIOB->BSRR = resetmask>>16;
    TIM1->CCR1 = sorted_out[0];
}

void TIM1_CC_Handler(void) {
    int ch = out_channel_active++;
    uint32_t mask = setmask[ch];
    GPIOA->BRR = mask&0xFFFF;
    GPIOB->BRR = mask>>16;
    if (ch < 6) {
        TIM1->CCR1 = sorted_out[ch+1];
    } else {
        calculate_timechain();
    }
}

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

    GPIO_TypeDef *gpios[NGPIOS] = {[IDX_GPIOA]=GPIOA, [IDX_GPIOB]=GPIOB};
    for (int i=0; i<NCH; i++) {
        GPIO_TypeDef *gpio = gpios[channels[i].gpio];
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
    TIM1->ARR = 20000; /* 20ms period */
    TIM1->EGR |= TIM_EGR_UG;

    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    uint32_t debounce_low = 0;
    uint32_t debounce_high = 0;
    uint32_t gpio_lastval = 0;
    uint16_t last[NCH] = {0};
    uint16_t diff[NCH] = {0};
    static uint32_t sdiff __attribute__((used));
    while (42) {
        uint32_t stk = SysTick->VAL;
        uint16_t now = TIM1->CNT;
        uint16_t gpio_vals[NGPIOS] = {[IDX_GPIOA]=GPIOA->IDR, [IDX_GPIOB]=GPIOB->IDR};
        for (int i=0; i<NCH; i++) {
            uint32_t mask = 1<<i;
            int gpio_val = gpio_vals[channels[i].gpio] & (1<<channels[i].pin);
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
        sdiff = (stk - SysTick->VAL) & 0xFFFFFF;

        int sel=0, d=diff[0];
        if (d < 800) {
            sel = 0;
        } else if (d < 1333) {
            sel = -1;
        } else if (d < 1666) {
            sel = 0;
        } else if (d < 2200) {
            sel = 1;
        } else {
            sel = 0;
        }
        switch (sel) {
        case -1:
            duration_out_write[ 0] = diff[1];
            duration_out_write[ 1] = diff[2];
            duration_out_write[ 2] = diff[3];
            duration_out_write[ 3] = diff[4];
            break;
        case 0:
            duration_out_write[ 4] = diff[1];
            duration_out_write[ 5] = diff[2];
            duration_out_write[ 6] = diff[3];
            duration_out_write[ 7] = diff[4];
            break;
        case 1:
            duration_out_write[ 8] = diff[1];
            duration_out_write[ 9] = diff[2];
            duration_out_write[10] = diff[3];
            duration_out_write[11] = diff[4];
            break;
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

