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


static void fnet_cpu_timer_handler_top(void *cookie);

/************************************************************************
* DESCRIPTION: Top interrupt handler. Increment fnet_current_time
*              and clear interrupt flag.
*************************************************************************/
static void fnet_cpu_timer_handler_top(void *cookie)
{
    /* Clear the PIT timer flag. */
    FNET_MK_PIT_TFLG(FNET_CFG_CPU_TIMER_NUMBER) |= FNET_MK_PIT_TFLG_TIF_MASK;

    /* Read the load value to restart the timer. */
    (void)FNET_MK_PIT_LDVAL(FNET_CFG_CPU_TIMER_NUMBER);

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
    fnet_uint32_t timeout;

    /* Install interrupt handler and enable interrupt in NVIC.
    */
    result = fnet_isr_vector_init(FNET_CFG_CPU_TIMER_VECTOR_NUMBER, fnet_cpu_timer_handler_top,
                                    fnet_timer_handler_bottom, 
                                    FNET_CFG_CPU_TIMER_VECTOR_PRIORITY, 0u);
    if(result == FNET_OK)
    {
        /* Initialize the PIT timer to generate an interrupt every period_ms */

        /* Enable the clock to the PIT module. Clock for PIT Timers to be enabled */
        FNET_MK_SIM_SCGC6 |= FNET_MK_SIM_SCGC6_PIT_MASK;

        /* Enable the PIT timer module. */
        FNET_MK_PIT_MCR &= ~FNET_MK_PIT_MCR_MDIS_MASK;

        /* Calculate the timeout value. */
        timeout = period_ms * FNET_MK_PERIPH_CLOCK_KHZ;
        FNET_MK_PIT_LDVAL(FNET_CFG_CPU_TIMER_NUMBER) = timeout;

        /* Enable the timer and enable interrupts */
        FNET_MK_PIT_TCTRL(FNET_CFG_CPU_TIMER_NUMBER) |= FNET_MK_PIT_TCTRL_TEN_MASK | FNET_MK_PIT_TCTRL_TIE_MASK;
    }

    return result;
}

/************************************************************************
* DESCRIPTION: Relaeses TCP/IP hardware timer.
*************************************************************************/
void fnet_cpu_timer_release( void )
{
    /* Disable the timer and disable interrupts */
    FNET_MK_PIT_TCTRL(FNET_CFG_CPU_TIMER_NUMBER) &= ~(FNET_MK_PIT_TCTRL_TEN_MASK | FNET_MK_PIT_TCTRL_TIE_MASK);

    /* Uninstall interrupt handler.
     */
    fnet_isr_vector_release(FNET_CFG_CPU_TIMER_VECTOR_NUMBER);
}

/* If vector table is in ROM, pre-install FNET ISR for the Timer Event interrupt*/
#if !FNET_CFG_CPU_VECTOR_TABLE_IS_IN_RAM
#if FNET_CFG_CPU_TIMER_NUMBER == 0
void TIMER0A_Handler (void)
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
