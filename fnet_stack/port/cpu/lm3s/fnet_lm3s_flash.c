#include "fnet.h"

#if FNET_LM3S && FNET_CFG_CPU_FLASH

#if (FNET_CFG_CPU_FLASH_PROGRAM_SIZE != 4u)
    #error "LM3S Flash driver supports only 4 size of program-block"
#endif

/************************************************************************
* DESCRIPTION: Erases the specified range of the Flash memory.
************************************************************************/
fnet_return_t fnet_cpu_flash_erase(void *flash_addr, fnet_size_t bytes)
{
    fnet_index_t    n_pages;
    fnet_uint32_t   page_shift = (fnet_uint32_t)flash_addr & (FNET_CFG_CPU_FLASH_PAGE_SIZE - 1U);
    fnet_cpu_irq_desc_t irq_desc;

    flash_addr = (fnet_uint8_t *)flash_addr - page_shift;

    bytes += page_shift;

    n_pages = (fnet_uint32_t)( bytes / FNET_CFG_CPU_FLASH_PAGE_SIZE + ((bytes % FNET_CFG_CPU_FLASH_PAGE_SIZE) ? 1ul : 0ul));

    /* setup FLASH timing register and disable flash-related interrupts */
    FNET_LM3S_FCTL_BASE_PTR->USECRL = FNET_CFG_CPU_CLOCK_HZ/1000000 - 1;
    FNET_LM3S_FCTL_BASE_PTR->FCIM &= ~(FNET_LM3S_FCTL_FCIM_PMASK | FNET_LM3S_FCTL_FCIM_AMASK);  

    while (n_pages)
    {
        irq_desc = fnet_cpu_irq_disable();
        /* Clear error flag. */
        FNET_LM3S_FCTL_BASE_PTR->FCMISC = FNET_LM3S_FCTL_FCMISC_AMISC;  
        /* Erase sector.*/;
        FNET_LM3S_FCTL_BASE_PTR->FMA = (fnet_uint32_t)flash_addr;
        FNET_LM3S_FCTL_BASE_PTR->FMC = FNET_LM3S_FCTL_FMC_WRKEY | FNET_LM3S_FCTL_FMC_ERASE;

        /* Wait erase complete. */
        while (FNET_LM3S_FCTL_BASE_PTR->FMC & FNET_LM3S_FCTL_FMC_ERASE)
        {
        }
        fnet_cpu_irq_enable(irq_desc);
        /* Check errors */
        if (FNET_LM3S_FCTL_BASE_PTR->FCRIS & FNET_LM3S_FCTL_FCRIS_ARIS)
        {
            return FNET_ERR;
        }  

        flash_addr = ((fnet_uint8_t *)flash_addr + FNET_CFG_CPU_FLASH_PAGE_SIZE);
        n_pages --;
    }

    /* TBD check if it was erased */
    return FNET_OK;
}

fnet_return_t fnet_cpu_flash_write( fnet_uint8_t *dest, const fnet_uint8_t *data)
{
    fnet_cpu_irq_desc_t irq_desc;                   

    /* setup FLASH timing register and disable flash-related interrupts */
    FNET_LM3S_FCTL_BASE_PTR->USECRL = FNET_CFG_CPU_CLOCK_HZ/1000000 - 1;
    FNET_LM3S_FCTL_BASE_PTR->FCIM &= ~(FNET_LM3S_FCTL_FCIM_PMASK | FNET_LM3S_FCTL_FCIM_AMASK);
  
    irq_desc = fnet_cpu_irq_disable();
    /* Clear error flag. */
    FNET_LM3S_FCTL_BASE_PTR->FCMISC = FNET_LM3S_FCTL_FCMISC_AMISC;  

    /* Write single DWORD */
    FNET_LM3S_FCTL_BASE_PTR->FMA = (fnet_uint32_t)dest;
    FNET_LM3S_FCTL_BASE_PTR->FMD = *(fnet_uint32_t *)data;
    FNET_LM3S_FCTL_BASE_PTR->FMC = FNET_LM3S_FCTL_FMC_WRKEY | FNET_LM3S_FCTL_FMC_WRITE;

    /* Wait erase complete. */
    while (FNET_LM3S_FCTL_BASE_PTR->FMC & FNET_LM3S_FCTL_FMC_ERASE)
    {
    }
    fnet_cpu_irq_enable(irq_desc);
    /* Check errors */
    if (FNET_LM3S_FCTL_BASE_PTR->FCRIS & FNET_LM3S_FCTL_FCRIS_ARIS)
    {
        return FNET_ERR;
    }  

    /* Verify result. */
#if FNET_CFG_CPU_FLASH_VERIFY
    if ((dest[0] != data[0]) || (dest[1] != data[1]) || (dest[2] != data[2]) || (dest[3] != data[3]))
    {
        return FNET_ERR;
    }
#endif /*FNET_CFG_CPU_FLASH_VERIFY*/

    return FNET_OK;
}

#endif /* FNET_LM3S && FNET_CFG_CPU_FLASH */