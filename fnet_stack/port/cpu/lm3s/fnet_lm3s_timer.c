#include "fnet_config.h"

#if FNET_LM3S
#include "fnet.h"
#include "stack/fnet_timer_prv.h"

/******************************************************************************
 *  Vector number of the timer interrupt.
 *  NOTE: User application should not change this parameter.
 ******************************************************************************/
#ifndef FNET_CFG_CPU_TIMER_VECTOR_NUMBER
    #if (FNET_CFG_CPU_TIMER_NUMBER != FNET_CFG_CPU_TIMER_MAX)
        #define FNET_CFG_CPU_TIMER_VECTOR_NUMBER       (35U + FNET_CFG_CPU_TIMER_NUMBER*2)
    #else
        #define FNET_CFG_CPU_TIMER_VECTOR_NUMBER       (51U)
    #endif
#endif

#if   (FNET_CFG_CPU_TIMER_NUMBER == 0)
    #define FNET_CFG_CPU_TIMER_PTR                     (FNET_LM3S_TIMER0_BASE_PTR)
    #define FNET_CFG_CPU_TIMER_RCGC1                   (FNET_LM3S_SYSCTL_RCGC1_TIMER0)
#elif (FNET_CFG_CPU_TIMER_NUMBER == 1)
    #define FNET_CFG_CPU_TIMER_PTR                     (FNET_LM3S_TIMER1_BASE_PTR)
    #define FNET_CFG_CPU_TIMER_RCGC1                   (FNET_LM3S_SYSCTL_RCGC1_TIMER1)
#elif (FNET_CFG_CPU_TIMER_NUMBER == 2)
    #define FNET_CFG_CPU_TIMER_PTR                     (FNET_LM3S_TIMER2_BASE_PTR)
    #define FNET_CFG_CPU_TIMER_RCGC1                   (FNET_LM3S_SYSCTL_RCGC1_TIMER2)
#elif (FNET_CFG_CPU_TIMER_NUMBER == 3)
    #define FNET_CFG_CPU_TIMER_PTR                     (FNET_LM3S_TIMER3_BASE_PTR)
    #define FNET_CFG_CPU_TIMER_RCGC1                   (FNET_LM3S_SYSCTL_RCGC1_TIMER3)
#else
    #error FNET_CFG_CPU_TIMER_NUMBER must be [0..3]
#endif

static void fnet_cpu_timer_handler_top(void *cookie);

/************************************************************************
* DESCRIPTION: Top interrupt handler. Increment fnet_current_time
*              and clear interrupt flag.
*************************************************************************/
static void fnet_cpu_timer_handler_top(void *cookie)
{
    /* Clear the timer overflow flag. */
    FNET_CFG_CPU_TIMER_PTR->ICR = FNET_LM3S_TIMER_ICR_TATOCINT;

    /* Update RTC counter.
     */
    fnet_timer_ticks_inc();
}

/************************************************************************
* DESCRIPTION: Starts TCP/IP hardware timer. delay_ms - period of timer (ms)
*         e.g. Time-out period = (1/FNET_CFG_SYSTEM_CLOCK_KHZ)x(1)x(124+1)x528x100 = 100 ms
*************************************************************************/
fnet_return_t fnet_cpu_timer_init( fnet_time_t period_ms )
{
    fnet_return_t result;
    fnet_uint32_t reload, prescale;

    /* Install interrupt handler and enable interrupt in NVIC.
    */
    result = fnet_isr_vector_init(FNET_CFG_CPU_TIMER_VECTOR_NUMBER, fnet_cpu_timer_handler_top,
                                    fnet_timer_handler_bottom, 
                                    FNET_CFG_CPU_TIMER_VECTOR_PRIORITY, 0u);
    if(result == FNET_OK)
    {
        /* Initialize the timer to generate an interrupt every period_ms */

        /* Enable the clock to the timer module. */
        FNET_LM3S_SYSCTL_BASE_PTR->RCGC1 |= FNET_CFG_CPU_TIMER_RCGC1;

        /* Setup 16-bit mode */
        FNET_CFG_CPU_TIMER_PTR->CFG = FNET_LM3S_TIMER_CFG_16_BIT;
        /* Setup TimerA mode */
        FNET_CFG_CPU_TIMER_PTR->TAMR = FNET_LM3S_TIMER_TAMR_TAMR_PERIOD;
        /* Calculate the timeout value. */
        reload = period_ms * (FNET_CFG_CPU_CLOCK_HZ/1000);
        prescale = 1;
        while ((reload/prescale) > 0xffff)
        {
            prescale++;
        }
        reload /= prescale;
        /* Setup prescaler value */
        FNET_CFG_CPU_TIMER_PTR->TAPMR = prescale - 1;
        /* Setup reload value */
        FNET_CFG_CPU_TIMER_PTR->TAILR = reload - 1;
        /* Enable interrupt on overflow */
        FNET_CFG_CPU_TIMER_PTR->IMR   = FNET_LM3S_TIMER_IMR_TATOIM;
        /* Enable the timer with stall in debugger */
        FNET_CFG_CPU_TIMER_PTR->CTL   = FNET_LM3S_TIMER_CTL_TAEN | FNET_LM3S_TIMER_CTL_TASTALL;
    }

    return result;
}

/************************************************************************
* DESCRIPTION: Releses TCP/IP hardware timer.
*************************************************************************/
void fnet_cpu_timer_release( void )
{
    /* disable interrupts */
    FNET_CFG_CPU_TIMER_PTR->IMR = 0;
    /* Disable the timer */
    FNET_CFG_CPU_TIMER_PTR->CTL = 0;
    /* Uninstall interrupt handler.
     */
    fnet_isr_vector_release(FNET_CFG_CPU_TIMER_VECTOR_NUMBER);
}

/* If vector table is in ROM, pre-install FNET ISR for the Timer Event interrupt*/
#if !FNET_CFG_CPU_VECTOR_TABLE_IS_IN_RAM
#if FNET_CFG_CPU_TIMER_NUMBER == 0
void lm3s_timer_isr (void)
{
    FNET_ISR_HANDLER();
}
#elif FNET_CFG_CPU_TIMER_NUMBER == 1
void TIMER1A_Handler (void)
{
    FNET_ISR_HANDLER();
}
#elif FNET_CFG_CPU_TIMER_NUMBER == 2
void TIMER2A_Handler (void)
{
    FNET_ISR_HANDLER();
}
#elif FNET_CFG_CPU_TIMER_NUMBER == 3
void TIMER3A_Handler (void)
{
    FNET_ISR_HANDLER();
}
#endif
#endif

#endif /* FNET_LM3S */
