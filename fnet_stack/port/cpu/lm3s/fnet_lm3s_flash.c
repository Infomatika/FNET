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

    flash_addr = (fnet_uint8_t *)flash_addr - page_shift;

    bytes += page_shift;

    n_pages = (fnet_uint32_t)( bytes / FNET_CFG_CPU_FLASH_PAGE_SIZE + ((bytes % FNET_CFG_CPU_FLASH_PAGE_SIZE) ? 1ul : 0ul));

    while (n_pages)
    {
        /* Erase sector/page.*/;
        fnet_ftfl_command(FNET_MK_FNET_FTFL_FCCOB0_CMD_ERASE_SECTOR, (fnet_uint32_t *)flash_addr, 0);

        flash_addr = ((fnet_uint8_t *)flash_addr + FNET_CFG_CPU_FLASH_PAGE_SIZE);
        n_pages --;
    }

    /* TBD check if it was erased */

    return FNET_OK;

}

fnet_return_t fnet_cpu_flash_write( fnet_uint8_t *dest, const fnet_uint8_t *data)

#endif /* FNET_LM3S && FNET_CFG_CPU_FLASH */