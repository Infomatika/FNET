/***************************************************************************
*
*  Stellaris Serial port I/O functions.
*
***************************************************************************/

#include "fnet_config.h"

#if FNET_LM3S

#include "fnet.h"

#define FNET_LM3S_UART_PORT_NUMBER                (3)
static FNET_LM3S_UART_MemMapPtr fnet_lm3s_get_uart_port_ptr[FNET_LM3S_UART_PORT_NUMBER] =
{
    FNET_LM3S_UART0_BASE_PTR,
    FNET_LM3S_UART1_BASE_PTR,
    FNET_LM3S_UART2_BASE_PTR
};


/********************************************************************/
void fnet_cpu_serial_putchar (fnet_index_t port_number, fnet_char_t character)
{
    FNET_LM3S_UART_MemMapPtr port_ptr = fnet_lm3s_get_uart_port_ptr[port_number];

    /* Wait until space is available in the FIFO */
    while(port_ptr->FR & FNET_LM3S_UART_FR_TXFF) 
    {}
                                             
    /* Send the character */
    port_ptr->DR = character;
}
/********************************************************************/
fnet_int32_t fnet_cpu_serial_getchar (fnet_index_t port_number)
{
    FNET_LM3S_UART_MemMapPtr port_ptr = fnet_lm3s_get_uart_port_ptr[port_number];

    /* Check for character has been received */
    if((port_ptr->FR & FNET_LM3S_UART_FR_RXFE) == 0u)
    {
        /* Return the 8-bit data from the receiver */
        return (fnet_int32_t)(port_ptr->DR & 0xFFu);
    }

    return FNET_ERR;
}


#if FNET_CFG_CPU_SERIAL_IO_INIT 
static inline void fnet_cpu_serial_gpio_init(fnet_index_t port_number)
{
    /* Enable the pins for the selected UART */
    /* Enable the clock to the selected UART */
    switch(port_number)
    {
        case 0: /* UART0 - PA0, PA1 */
            /* Enable clock */
            FNET_LM3S_SYSCTL_BASE_PTR->RCGC2 |= FNET_LM3S_SYSCTL_RCGC2_GPIOA;
            FNET_LM3S_SYSCTL_BASE_PTR->RCGC1 |= FNET_LM3S_SYSCTL_RCGC1_UART0;
            (void)FNET_LM3S_SYSCTL_BASE_PTR->RCGC2;
//            __DSB();
            /* Configure pins */
            FNET_LM3S_PORTA_BASE_PTR->DEN    |= (1 << 0) | (1 << 1);
            FNET_LM3S_PORTA_BASE_PTR->DIR    |= (1 << 1);
            FNET_LM3S_PORTA_BASE_PTR->AFSEL  |= (1 << 0) | (1 << 1);               
            break;

        case 1: /* UART1 - PD2, PD3 */
            /* Enable clock */
            FNET_LM3S_SYSCTL_BASE_PTR->RCGC2 |= FNET_LM3S_SYSCTL_RCGC2_GPIOD;
            FNET_LM3S_SYSCTL_BASE_PTR->RCGC1 |= FNET_LM3S_SYSCTL_RCGC1_UART1; 
            (void)FNET_LM3S_SYSCTL_BASE_PTR->RCGC2;
//            __DSB();
            /* Configure pins */
            FNET_LM3S_PORTD_BASE_PTR->DEN    |= (1 << 2) | (1 << 3);
            FNET_LM3S_PORTD_BASE_PTR->DIR    |= (1 << 3);
            FNET_LM3S_PORTD_BASE_PTR->AFSEL  |= (1 << 2) | (1 << 3);
            break;

        case 2: /* UART2 (available on LM3S6965, absent on LM3S8962) - PG0, PG1 */
            /* Enable clock */
            FNET_LM3S_SYSCTL_BASE_PTR->RCGC2 |= FNET_LM3S_SYSCTL_RCGC2_GPIOG;
            FNET_LM3S_SYSCTL_BASE_PTR->RCGC1 |= FNET_LM3S_SYSCTL_RCGC1_UART2; 
            (void)FNET_LM3S_SYSCTL_BASE_PTR->RCGC2;
//            __DSB();
            /* Configure pins */
            FNET_LM3S_PORTG_BASE_PTR->DEN    |= (1 << 0) | (1 << 1);
            FNET_LM3S_PORTG_BASE_PTR->DIR    |= (1 << 1);                          
            FNET_LM3S_PORTG_BASE_PTR->AFSEL  |= (1 << 0) | (1 << 1);
            break;
    }
}
#endif /* FNET_CFG_CPU_SERIAL_IO_INIT */

/********************************************************************/
void fnet_cpu_serial_init(fnet_index_t port_number, fnet_uint32_t baud_rate)
{
    FNET_LM3S_UART_MemMapPtr  uartch;
    fnet_uint32_t             temp; 

#if FNET_CFG_CPU_SERIAL_IO_INIT 
    /* Init GPIO.*/
    fnet_cpu_serial_gpio_init(port_number);
#endif

    /* Get UART module base address.*/
    uartch = fnet_lm3s_get_uart_port_ptr[port_number];

    /* Make sure that the transmitter and receiver are disabled while we
    * change settings.
    */
    uartch->CTL = 0;
    /* disable UART interrupts */
    uartch->IM  = 0;
    (void)uartch->CTL;
//    __DSB();
    /* UART Clock divisor */
    temp = (FNET_CFG_CPU_CLOCK_HZ * 4) / baud_rate + 32;
    uartch->IBRD = temp >> 6;
    uartch->FBRD = temp & 0x3F;
    /* Initialize the UART for 8N1 operation, FIFO enabled */
    uartch->LCRH = FNET_LM3S_UART_LCRH_FEN | FNET_LM3S_UART_LCRH_WLEN_8;
    (void)uartch->CTL;
//    __DSB();
    /* Enable transmitter, receiver and UART itself */
    uartch->CTL = FNET_LM3S_UART_CTL_RXE | FNET_LM3S_UART_CTL_TXE | FNET_LM3S_UART_CTL_UARTEN;
}


#endif /*FNET_LM3S*/