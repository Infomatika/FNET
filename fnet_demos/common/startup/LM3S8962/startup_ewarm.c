#pragma language=extended
#include "common.h"


#ifndef STACK_SIZE
#    define STACK_SIZE                2048
#endif

#ifndef NVIC_INTERRUPT_MAX
#    define NVIC_INTERRUPT_MAX        (16 + PWM0_3_IRQn)
#endif

// обработчики исключений
static void nmi_isr(void);
static void hard_fault_isr(void);
static void bus_fault_isr(void);
static void usage_fault_isr(void);
static void default_isr(void);
static void watchdog_isr(void);
static void sysctl_isr(void);

// функа, вызываемая до инициализации рантайма
static void __before_program_start(void);
// инициализация тактового генератора, флеша и т.д.
extern void board_init(void);
// инициализация рантайма
extern void __iar_program_start(void);

// прототип функций-обработчиков прерываний
typedef void (*irq_handler_t)(void);

// обработчики прерываний
extern void systick_isr(void);
extern void ethernet_isr(void);
extern void uart0_isr(void);

// начальный стек 
static unsigned long stack[STACK_SIZE] @ ".noinit";

// таблица векторов прерываний во флеше
__root const irq_handler_t __vector_table[NVIC_INTERRUPT_MAX+1] @ ".intvec" =
{
    // Cortex-M3 core interrupts
    (irq_handler_t)(stack + STACK_SIZE),    // The initial stack pointer
    __before_program_start,                 // The reset handler
    nmi_isr,                                // The NMI handler
    hard_fault_isr,                         // The hard fault handler
    default_isr,                            // The MPU fault handler
    bus_fault_isr,                          // The bus fault handler
    usage_fault_isr,                        // The usage fault handler
    (irq_handler_t)0x00001100,              // Reserved
    (void*)0,                               // Reserved
    (void*)0,                               // Reserved
    (void*)0,                               // Reserved
    (void*)0,                               // SVCall handler
    default_isr,                            // Debug monitor handler
    (void*)0,                               // Reserved
    default_isr,                            // The PendSV handler
    systick_isr,                            // The SysTick handler
   // LM3S6965 specific interrupts Numbers
    default_isr,                            //  0 GPIOA_IRQn
    default_isr,                            //  1 GPIOB_IRQn
    default_isr,                            //  2 GPIOC_IRQn
    default_isr,                            //  3 GPIOD_IRQn
    default_isr,                            //  4 GPIOE_IRQn
    uart0_isr,                              //  5 UART0_IRQn
    default_isr,                            //  6 UART1_IRQn
    default_isr,                            //  7 SSI0_IRQn
    default_isr,                            //  8 I2C0_IRQn
    default_isr,                            //  9 PWM0_FAULT_IRQn
    default_isr,                            // 10 PWM0_0_IRQn
    default_isr,                            // 11 PWM0_1_IRQn
    default_isr,                            // 12 PWM0_2_IRQn
    default_isr,                            // 13 QEI0_IRQn
    default_isr,                            // 14 ADC0SS0_IRQn
    default_isr,                            // 15 ADC0SS1_IRQn
    default_isr,                            // 16 ADC0SS2_IRQn
    default_isr,                            // 17 ADC0SS3_IRQn
    watchdog_isr,                           // 18 WATCHDOG0_IRQn
    default_isr,                            // 19 TIMER0A_IRQn
    default_isr,                            // 20 TIMER0B_IRQn
    default_isr,                            // 21 TIMER1A_IRQn
    default_isr,                            // 22 TIMER1B_IRQn
    default_isr,                            // 23 TIMER2A_IRQn
    default_isr,                            // 24 TIMER2B_IRQn
    default_isr,                            // 25 COMP0_IRQn
    default_isr,                            // 26 COMP1_IRQn
    default_isr,                            // 27 COMP2_IRQn
    sysctl_isr,                             // 28 SYSCTL_IRQn
    default_isr,                            // 29 FLASH_CTRL_IRQn
    default_isr,                            // 30 GPIOF_IRQn
    default_isr,                            // 31 GPIOG_IRQn
    default_isr,                            // 
    default_isr,                            // 33 UART2_IRQn
    default_isr,                            // 
    default_isr,                            // 35 TIMER3A_IRQn
    default_isr,                            // 36 TIMER3B_IRQn
    default_isr,                            // 37 I2C1_IRQn
    default_isr,                            // 38 QEI1_IRQn
    default_isr,                            // 
    default_isr,                            // 
    default_isr,                            //            
    ethernet_isr,                           // 42 ETH_IRQn
    default_isr,                            // 43 HIB_IRQn
    default_isr,                            // 
    default_isr,                            // 45 PWM0_3_IRQn
};

// обработчик немаскируемого прерывания
static void nmi_isr(void)
{
    while(1)
    {
    }
}

// обработчик исключения неустранимой ошибки
static void hard_fault_isr(void)
{
    while(1)
    {
    }
}

// обработчик исключения ошибки шины
static void bus_fault_isr(void)
{
    while(1)
    {
    }
}

// обработчик исключения ошибки использования
static void usage_fault_isr(void)
{
    while(1)
    {
    }
}

// прерывание сторожевого таймера
void watchdog_isr(void)
{
    while(1)
    {
    }
}

void sysctl_isr(void)
{
    while(1)
    {
    }
}


// обработчик прерываний "по умолчанию"
void default_isr(void)
{
    while(1)
    {
    }
}

static void delay_loop(uint32_t count)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   delay_loop\n"
          "    bx      lr");
}


static void __before_program_start(void)
{
    uint32_t i;
    
    /* silicon errata workaround - rise core voltage */
    SYSCTL->LDOPCTL = SYSCTL_LDOPCTL_2_75V;
    __DSB();
  
    /* start crystal oscillator */
    SYSCTL->RCC &= ~ SYSCTL_RCC_MOSCDIS;
    __DSB();
    delay_loop(524288);

    /* select crystal frequency, switch CPU clocking to crystal and enable PLL */ 
    SYSCTL->RCC = ((SYSCTL->RCC & 
                    ~(SYSCTL_RCC_ACG | SYSCTL_RCC_PWRDN | SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M)) | 
                    SYSCTL_RCC_XTAL_8MHZ | SYSCTL_RCC_OSCSRC_MAIN);
    __DSB();
    delay_loop(524288);

    /* wait for PLL lock */ 
    for (i = 32768; i > 0; i--)
    {
        if (SYSCTL->RIS & SYSCTL_RIS_PLLLRIS)
        {
            break;
        }
    }

    /* switch clocking to PLL/4, so 50MHz */ 
    SYSCTL->RCC = ((SYSCTL->RCC & ~(SYSCTL_RCC_BYPASS | SYSCTL_RCC_SYSDIV_M)) |
                    SYSCTL_RCC_SYSDIV_4 | SYSCTL_RCC_USESYSDIV);
    __DSB();
    delay_loop(300);  

    /* enable CPU exception handling */ 
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;

    /* set 3-bit preemption priority */ 
    SCB->AIRCR = 0x05FA0000 | 0x00000300; //SCB_AIRCR_PRIGROUP3

    /* enable DWT subsystem */ 
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* enable cycle-accurate counter */ 
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    delay_loop(300);

    board_init();

    __iar_program_start(); 
}
