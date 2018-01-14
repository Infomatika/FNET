#pragma language=extended
#include "lm3s8962.h"


#ifndef STACK_SIZE                                                                                               
#    define STACK_SIZE                2048
#endif

#ifndef NVIC_INTERRUPT_MAX
#    define NVIC_INTERRUPT_MAX        (16 + 54)
#endif

/* exception stack frame - useful for debugging */ 
typedef struct fault_frame_s
{
    uint32_t   stacked_R0;
    uint32_t   stacked_R1;
    uint32_t   stacked_R2;
    uint32_t   stacked_R3;
    uint32_t   stacked_R12;
    uint32_t   stacked_LR;
    uint32_t   stacked_PC;
    uint32_t   stacked_xPSR;
} fault_frame_t;

/* pointer to exception stack frame */
__root fault_frame_t * fault_frame;


/* exception handler */ 
static void default_isr(void);

/* startup entry point */ 
static void __before_program_start(void);
/* IAR startup entry */ 
extern void __iar_program_start(void);

/* interrupt/exception handlers proto */ 
typedef void (*irq_handler_t)(void);

/* used interrupt handlers */ 
extern void systick_isr(void);
extern void ethernet_isr(void);
extern void timer_isr(void);

/* initial stack */  
static unsigned long stack[STACK_SIZE] @ ".noinit";

/* Vector table */
__root const irq_handler_t __vector_table[NVIC_INTERRUPT_MAX+1] @ ".intvec" =
{
    /* Cortex-M3 core interrupts */
    (irq_handler_t)(stack + STACK_SIZE),    /* The initial stack pointer */
    __before_program_start,                 /* The reset handler         */
    default_isr,                            /* The NMI handler           */
    default_isr,                            /* The hard fault handler    */
    default_isr,                            /* The MPU fault handler     */
    default_isr,                            /* The bus fault handler     */
    default_isr,                            /* The usage fault handler   */
    (void*)0,                               /* Reserved                  */
    (void*)0,                               /* Reserved                  */
    (void*)0,                               /* Reserved                  */
    (void*)0,                               /* Reserved                  */
    default_isr,                            /* SVCall handler            */
    default_isr,                            /* Debug monitor handler     */
    (void*)0,                               /* Reserved                  */
    default_isr,                            /* The PendSV handler        */
    systick_isr,                            /* The SysTick handler       */
    /* Stellaris-specific interrupts Numbers */
    default_isr,                            /*  0 GPIOA_IRQn         */ 
    default_isr,                            /*  1 GPIOB_IRQn         */ 
    default_isr,                            /*  2 GPIOC_IRQn         */ 
    default_isr,                            /*  3 GPIOD_IRQn         */ 
    default_isr,                            /*  4 GPIOE_IRQn         */ 
    default_isr,                            /*  5 UART0_IRQn         */ 
    default_isr,                            /*  6 UART1_IRQn         */ 
    default_isr,                            /*  7 SSI0_IRQn          */ 
    default_isr,                            /*  8 I2C0_IRQn          */ 
    default_isr,                            /*  9 PWM0_FAULT_IRQn    */ 
    default_isr,                            /* 10 PWM0_0_IRQn        */ 
    default_isr,                            /* 11 PWM0_1_IRQn        */ 
    default_isr,                            /* 12 PWM0_2_IRQn        */ 
    default_isr,                            /* 13 QEI0_IRQn          */ 
    default_isr,                            /* 14 ADC0SS0_IRQn       */ 
    default_isr,                            /* 15 ADC0SS1_IRQn       */ 
    default_isr,                            /* 16 ADC0SS2_IRQn       */ 
    default_isr,                            /* 17 ADC0SS3_IRQn       */ 
    default_isr,                            /* 18 WATCHDOG0_IRQn     */ 
    default_isr,                            /* 19 TIMER0A_IRQn       */ 
    default_isr,                            /* 20 TIMER0B_IRQn       */ 
    default_isr,                            /* 21 TIMER1A_IRQn       */ 
    default_isr,                            /* 22 TIMER1B_IRQn       */ 
    default_isr,                            /* 23 TIMER2A_IRQn       */ 
    default_isr,                            /* 24 TIMER2B_IRQn       */ 
    default_isr,                            /* 25 COMP0_IRQn         */ 
    default_isr,                            /* 26 COMP1_IRQn         */ 
    default_isr,                            /* 27 COMP2_IRQn         */ 
    default_isr,                            /* 28 SYSCTL_IRQn        */ 
    default_isr,                            /* 29 FLASH_CTRL_IRQn    */ 
    default_isr,                            /* 30 GPIOF_IRQn         */ 
    default_isr,                            /* 31 GPIOG_IRQn         */ 
    default_isr,                            /* 32 GPIOH_IRQn         */ 
    default_isr,                            /* 33 UART1_IRQn         */ 
    default_isr,                            /* 34 SSI1_IRQn          */ 
    default_isr,                            /* 35 TIMER3A_IRQn       */ 
    default_isr,                            /* 36 TIMER3B_IRQn       */ 
    default_isr,                            /* 37 I2C1_IRQn          */ 
    default_isr,                            /* 38 QEI1_IRQn          */ 
    default_isr,                            /* 39 CAN0_IRQn          */
    default_isr,                            /* 40 CAN1_IRQn          */
    default_isr,                            /* 41 CAN2_IRQn          */
    ethernet_isr,                           /* 42 ETH_IRQn           */
    default_isr,                            /* 43 HIB_IRQn           */
    default_isr,                            /* 44 USB0_IRQn          */
    default_isr,                            /* 45 PWM0_3_IRQn        */
    default_isr,                            /* 46 UDMA_IRQn          */
    default_isr,                            /* 47 UDMAERR_IRQn       */
    default_isr,                            /* 48 ADC1SS0_IRQn       */
    default_isr,                            /* 49 ADC1SS1_IRQn       */
    default_isr,                            /* 50 ADC1SS2_IRQn       */
    default_isr,                            /* 51 ADC1SS3_IRQn       */
    default_isr,                            /* 52 I2S0_IRQn          */
    default_isr,                            /* 53 EPI0_IRQn          */
    default_isr,                            /* 54 GPIOJ_IRQn         */
};              
                

/* default interrupt handler */ 
void default_isr(void)
{
    if (__get_CONTROL() & 2)
    {
        fault_frame = (fault_frame_t*)__get_PSP();
    }
    else
    {
        fault_frame = (fault_frame_t*)__get_MSP();
    }   
    while(1)
    {
    }
}

static void __before_program_start(void)
{
    volatile uint32_t i;
    
    /* silicon errata workaround - rise core voltage to 2.75V */
    SYSCTL->LDOPCTL = 0x0000001B;
    __DSB();
  
    /* start crystal oscillator */
    SYSCTL->RCC &= 0xFFFFFFFE;
    __DSB();
    for (i=0; i<524288; i++);

    /* select 8MHz crystal frequency, switch CPU clocking to crystal and enable PLL */ 
    SYSCTL->RCC = (SYSCTL->RCC & 0xF7FFDC0F) | 0x00000380;
    __DSB();
    for (i=0; i<524288; i++);

    /* wait for PLL lock */ 
    for (i = 32768; i > 0; i--)
    {
        if (SYSCTL->RIS & 0x00000040)
        {
            break;
        }
    }

    /* switch clocking to PLL/4, so 50MHz */ 
    SYSCTL->RCC = (SYSCTL->RCC & 0xF87FF7FF) | 0x01C00000;
    __DSB();
    for (i=0; i<300; i++);  

    /* enable CPU exception handling */ 
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;

    /* set 3-bit preemption priority */ 
    SCB->AIRCR = 0x05FA0000 | 0x00000300; 

    /* enable DWT subsystem */ 
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* enable cycle-accurate counter */ 
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    for (i=0; i<300; i++);

    //board_init();

    __iar_program_start(); 
}
