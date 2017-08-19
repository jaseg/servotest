

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

#define PIN_IN_START 5
#define NCH_IN 5

#define PIN_OUT_START 0
#define NCH_OUT 12

static uint16_t duration_out_a[NCH_OUT] = {[0 ... (NCH_OUT-1)]=1500};
static uint16_t duration_out_b[NCH_OUT] = {[0 ... (NCH_OUT-1)]=1500};
static uint16_t *duration_out_read = duration_out_a;
static uint16_t *duration_out_write = duration_out_b;
/* Pad with 0xFFFF in the end to nicely terminate CCR interrupt chain after last channel */
static uint16_t sorted_out[NCH_OUT+1] = {[0 ... (NCH_OUT-1)]=0, [NCH_OUT]=0xFFFF};
static uint32_t setmask[NCH_OUT];
static uint32_t resetmask = 0;

uint32_t get_tick() {
    return SysTick->VAL;
}

uint32_t tcdiff = 0;
void calculate_timechain(void) {
    uint32_t stk1 = get_tick();
    uint8_t idxs[NCH_OUT];
    for (int i=0; i<NCH_OUT; i++)
        idxs[i] = i;
    for (int i=0; i<NCH_OUT-1; i++) {
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
        int mask = 1<<(PIN_OUT_START+idx);
        setmask[out_idx] |= mask;
    }
    /* No need to clear later channels */
    uint32_t stk2 = get_tick();
    tcdiff = (stk1 - stk2) & 0xFFFFFF;
}

static volatile int out_channel_active;
void TIM1_UP_IRQHandler(void) {
    TIM1->SR &= ~TIM_SR_UIF_Msk;
    out_channel_active = 0;
    calculate_timechain();
    GPIOA->BSRR = resetmask;
    TIM1->CCR1 = sorted_out[0];
}

void TIM1_CC_IRQHandler(void) {
    TIM1->SR &= ~TIM_SR_CC1IF_Msk;
    int ch = out_channel_active++;
    uint32_t mask = setmask[ch];
    GPIOA->BRR = mask;
    TIM1->CCR1 = sorted_out[ch+1];
}

uint32_t sdiff;
int sel;
int gsel;
int loop;
int last_loop;
uint32_t last_inc = 0;
uint32_t cycles_per_inc;

int main(void) {
    /* External crystal: 8MHz */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));

    /* Sysclk = HCLK = 48MHz */
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PLLMULL_Msk & ~RCC_CFGR_SW_Msk & ~RCC_CFGR_PPRE1_Msk & ~RCC_CFGR_PPRE2_Msk & ~RCC_CFGR_HPRE_Msk))
        | (10<<RCC_CFGR_PLLMULL_Pos) | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | (4<<RCC_CFGR_PPRE1_Pos) | (4<<RCC_CFGR_PPRE2_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));

    /* Switch to PLL */
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    //RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1_Msk & ~RCC_CFGR_PPRE2_Msk))
    //    | (4<<RCC_CFGR_PPRE1_Pos) | (4<<RCC_CFGR_PPRE2_Pos);
    SystemCoreClockUpdate();

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    for (int pin=PIN_IN_START; pin<PIN_IN_START+NCH_IN; pin++) {
        if (pin < 8) {
            GPIOB->CRL &= ~(0xf<<(pin*4));
            GPIOB->CRL |= (0x8<<(pin*4)); /* input with pull-up/down */
        } else {
            GPIOB->CRH &= ~(0xf<<((pin-8)*4));
            GPIOB->CRH |= (0x8<<((pin-8)*4)); /* input with pull-up/down */
        }
        GPIOB->ODR |= 1<<pin; /* pull-down */
    }

    for (int pin=PIN_OUT_START; pin<PIN_OUT_START+NCH_OUT; pin++) {
        resetmask |= 1<<pin;
        if (pin < 8) {
            GPIOA->CRL &= ~(0xf<<(pin*4));
            GPIOA->CRL |= (0x1<<(pin*4)); /* push/pull general purpose output */
        } else {
            GPIOA->CRH &= ~(0xf<<((pin-8)*4));
            GPIOA->CRH |= (0x1<<((pin-8)*4)); /* push/pull general purpose output */
        }
    }

    TIM1->CR1 = TIM_CR1_CEN;
    TIM1->PSC = 48; /* Count at 1MHz */
    TIM1->ARR = 20000; /* 20ms period */
    TIM1->EGR |= TIM_EGR_UG;

    NVIC_EnableIRQ(TIM1_CC_IRQn);
    NVIC_SetPriority(TIM1_CC_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM1_UP_IRQn, 3);
    TIM1->DIER |= TIM_DIER_CC1IE | TIM_DIER_UIE;

    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    uint32_t debounce = 0;
    uint32_t gpio_lastval = 0;
    uint16_t last[NCH_IN] = {0};
    uint16_t diff[NCH_IN] = {[0 ... NCH_IN-1]=1500};
    while (42) {
        loop = get_tick();
        uint32_t stk1 = get_tick();
        uint16_t now = TIM1->CNT;
        uint16_t gpio_vals = GPIOB->IDR;
        for (int i=0; i<NCH_IN; i++) {
            uint32_t mask = 1<<i;
            if (gpio_vals & (1<<(i+PIN_IN_START))) {
                if (~gpio_lastval & debounce &mask) {
                    gpio_lastval |= mask;
                    debounce &= ~mask;
                    last[i] = now;
                }
            } else {
                //if ((gpio_lastval&mask) && (debounce&mask)) {
                if (gpio_lastval & debounce & mask) {
                    gpio_lastval &= ~mask;
                    debounce &= ~mask;
                    diff[i] = now-last[i];
                    last[i] = now;
                }
            }

            if ((uint16_t)(now-last[i]) > 100)
                debounce |= mask;
        }
        uint32_t stk2 = get_tick();
        sdiff = (stk1 - stk2) & 0xFFFFFF;

        int booldiff[NCH_IN];
        for (int i=0; i<NCH_IN; i++) {
            int d=diff[i];
            if (d < 800) {
                booldiff[i] = -1;
            } else if (d < 1333) {
                booldiff[i] = 0;
            } else if (d < 1666) {
                booldiff[i] = 1;
            } else if (d < 2200) {
                booldiff[i] = 2;
            } else {
                booldiff[i] = -1;
            }
        }
        booldiff[0] = booldiff[0] == -1 ? 0 : booldiff[0];
        booldiff[1] = booldiff[1] == -1 ? 0 : booldiff[1];
        booldiff[2] = booldiff[2] == -1 ? 0 : booldiff[2];
        booldiff[3] = booldiff[3] == -1 ? 0 : booldiff[3];
        /* FIXME DEBUG */
        sel = booldiff[NCH_IN-1];
        /* FIXME DEBUG */
        gsel = 0;
        int speed = 0;
        last_inc += (last_loop-loop)&0xffffff; /* SysTick is a 24-bit down-counter. */
#define SWITCH_TO_PROP_FULLSCALE_TIME_MS 2000000
        cycles_per_inc = (SWITCH_TO_PROP_FULLSCALE_TIME_MS/1000) * (SystemCoreClock/1000);
        if (last_inc > cycles_per_inc) {
            last_inc = 0;
            speed = 1;
        }
        uint16_t clip(uint16_t val) {
            if (val > 2000)
                return 2000;
            if (val < 1000)
                return 1000;
            return val;
        }
        switch (gsel) {
        case 0:
            duration_out_write[ 0] = clip(duration_out_read[ 0] + (booldiff[0]-1)*speed);
            duration_out_write[ 1] = clip(duration_out_read[ 1] + (booldiff[1]-1)*speed);
            duration_out_write[ 2] = diff[ 2];
            duration_out_write[ 3] = diff[ 3];
            duration_out_write[ 4] = duration_out_read[ 4];
            duration_out_write[ 5] = duration_out_read[ 5];
            duration_out_write[ 6] = duration_out_read[ 6];
            duration_out_write[ 7] = duration_out_read[ 7];
            duration_out_write[ 8] = duration_out_read[ 8];
            duration_out_write[ 9] = duration_out_read[ 9];
            duration_out_write[10] = duration_out_read[10];
            duration_out_write[11] = duration_out_read[11];
            break;
        case 1:
            duration_out_write[ 0] = duration_out_read[ 0];
            duration_out_write[ 1] = duration_out_read[ 1];
            duration_out_write[ 2] = duration_out_read[ 2];
            duration_out_write[ 3] = duration_out_read[ 3];
            duration_out_write[ 4] = clip(duration_out_read[ 0] + booldiff[0]*speed);
            duration_out_write[ 5] = clip(duration_out_read[ 1] + booldiff[1]*speed);
            duration_out_write[ 6] = diff[ 2];
            duration_out_write[ 7] = diff[ 3];
            duration_out_write[ 8] = duration_out_read[ 8];
            duration_out_write[ 9] = duration_out_read[ 9];
            duration_out_write[10] = duration_out_read[10];
            duration_out_write[11] = duration_out_read[11];
            break;
        case 2:
            duration_out_write[ 0] = duration_out_read[ 0];
            duration_out_write[ 1] = duration_out_read[ 1];
            duration_out_write[ 2] = duration_out_read[ 2];
            duration_out_write[ 3] = duration_out_read[ 3];
            duration_out_write[ 4] = duration_out_read[ 4];
            duration_out_write[ 5] = duration_out_read[ 5];
            duration_out_write[ 6] = duration_out_read[ 6];
            duration_out_write[ 7] = duration_out_read[ 7];
            duration_out_write[ 8] = clip(duration_out_read[ 0] + booldiff[0]*speed);
            duration_out_write[ 9] = clip(duration_out_read[ 1] + booldiff[1]*speed);
            duration_out_write[10] = diff[ 2];
            duration_out_write[11] = diff[ 3];
            break;
        default:
            duration_out_write[ 0] = duration_out_read[ 0];
            duration_out_write[ 1] = duration_out_read[ 1];
            duration_out_write[ 2] = duration_out_read[ 2];
            duration_out_write[ 3] = duration_out_read[ 3];
            duration_out_write[ 4] = duration_out_read[ 4];
            duration_out_write[ 5] = duration_out_read[ 5];
            duration_out_write[ 6] = duration_out_read[ 6];
            duration_out_write[ 7] = duration_out_read[ 7];
            duration_out_write[ 8] = duration_out_read[ 8];
            duration_out_write[ 9] = duration_out_read[ 9];
            duration_out_write[10] = duration_out_read[10];
            duration_out_write[11] = duration_out_read[11];
            break;
        }
        uint16_t *tmp = duration_out_read;
        duration_out_read = duration_out_write;
        duration_out_write = tmp;
        last_loop = loop;
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

void MemManage_Handler(void) __attribute__((naked));
void MemManage_Handler() {
    asm volatile ("bkpt");
}

void BusFault_Handler(void) __attribute__((naked));
void BusFault_Handler() {
    asm volatile ("bkpt");
}
