#include "fnet.h"
#if FNET_LM3S && FNET_CFG_CPU_ETH0

#define FNET_LM3S_MII_TIMEOUT            (0x10000U)   /* Timeout counter for MII communications.*/

/************************************************************************
* Ethernet interface structure.
*************************************************************************/
static fnet_eth_if_t fnet_lm3s_eth0_if =
{
    .eth_prv = &fnet_lm3s_if,                 /* Points to Ethernet driver-specific control data structure. */
    .eth_mac_number = 0,                      /* MAC module number. */
    .eth_output = fnet_lm3s_output,           /* Ethernet driver output.*/
};

fnet_netif_t fnet_cpu_eth0_if =
{
    .netif_name = FNET_CFG_CPU_ETH0_NAME,     /* Network interface name.*/
    .netif_mtu = FNET_CFG_CPU_ETH0_MTU,       /* Maximum transmission unit.*/
    .netif_prv = &fnet_lm3s_eth0_if,          /* Points to interface specific data structure.*/
    .netif_api = &fnet_lm3s_api               /* Interface API */
};

fnet_return_t fnet_lm3s_mii_write(fnet_uint32_t reg_addr, fnet_uint16_t data);
fnet_return_t fnet_lm3s_mii_read(fnet_uint32_t reg_addr, fnet_uint16_t *data);

/************************************************************************
* DESCRIPTION: Read a value from a PHY's MII register.
* reg_addr < address of the register in the PHY
* data < Pointer to storage for the Data to be read from the PHY register (passed by reference)
* Return FNET_ERR on failure, FNET_OK on success
*************************************************************************/
fnet_return_t fnet_lm3s_mii_read(fnet_lm3s_if_t *ethif, fnet_uint32_t reg_addr, fnet_uint16_t *data)
{
    fnet_time_t       timeout;

    /* Wait for any pending transaction to complete. */
    for (timeout = 0U; timeout < FNET_LM3S_MII_TIMEOUT; timeout++)
    {
        if (0 == (FNET_LM3S_MAC_BASE_PTR->MCTL & MAC_MCTL_START))
        {
            break;
        }
    }
    if (FNET_LM3S_MII_TIMEOUT == timeout)
    {
        return FNET_ERR;
    }
    /* Program the PHY register address and initiate the transaction. */
    FNET_LM3S_MAC_BASE_PTR->MCTL = ((reg_addr << FNET_LM3S_MAC_MCTL_REGADR_S) & FNET_LM3S_MAC_MCTL_REGADR_M) | 
                                    FNET_LM3S_MAC_MCTL_START;
    __DSB();
    /* Wait for the read transaction to complete. */
    for (timeout = 0U; timeout < FNET_LM3S_MII_TIMEOUT; timeout++)
    {
        if (0 == (FNET_LM3S_MAC_BASE_PTR->MCTL & MAC_MCTL_START))
        {
            break;
        }
    }
    if (FNET_LM3S_MII_TIMEOUT == timeout)
    {
        return FNET_ERR;
    }

    /* Return the PHY data that was read. */
    *data = FNET_LM3S_MAC_BASE_PTR->MRXD & FNET_LM3S_MAC_MRXD_MDRX_M;
    return FNET_OK;
}

/************************************************************************
* DESCRIPTION: Write a value to a PHY's MII register.
* reg_addr < address of the register in the PHY
* data < Data to be writen to the PHY register (passed by reference)
* Return FNET_ERR on failure (timeout), FNET_OK on success
*************************************************************************/
fnet_return_t fnet_lm3s_mii_read(fnet_uint32_t reg_addr, fnet_uint16_t *data)
{
    fnet_time_t       timeout;

    /* Wait for any pending transaction to complete. */
    for (timeout = 0U; timeout < FNET_LM3S_MII_TIMEOUT; timeout++)
    {
        if (0 == (FNET_LM3S_MAC_BASE_PTR->MCTL & MAC_MCTL_START))
        {
            break;
        }
    }
    if (FNET_LM3S_MII_TIMEOUT == timeout)
    {
        return FNET_ERR;
    }

    /* Program the DATA to be written. */
    FNET_LM3S_MAC_BASE_PTR->MTXD = *data;
    
    /* Program the PHY register address and initiate the transaction. */
    FNET_LM3S_MAC_BASE_PTR->MCTL = ((reg_addr << FNET_LM3S_MAC_MCTL_REGADR_S) & FNET_LM3S_MAC_MCTL_REGADR_M) | 
                                    FNET_LM3S_MAC_MCTL_WRITE | FNET_LM3S_MAC_MCTL_START;
    __DSB();
    /* Wait for the write transaction to complete. */
    for (timeout = 0U; timeout < FNET_LM3S_MII_TIMEOUT; timeout++)
    {
        if (0 == (FNET_LM3S_MAC_BASE_PTR->MCTL & MAC_MCTL_START))
        {
            break;
        }
    }
    if (FNET_LM3S_MII_TIMEOUT == timeout)
    {
        return FNET_ERR;
    }
    return FNET_OK;
}


/************************************************************************
* DESCRIPTION: Ethernet IO initialization.
*************************************************************************/
#if FNET_CFG_CPU_ETH_IO_INIT
void fnet_eth_io_init(void)
{
    /* Enable clocking for EMAC and EPHY */
    FNET_LM3S_SYSCTL_BASE_PTR->RCGC2 |= FNET_LM3S_SYSCTL_RCGC2_EPHY0 | FNET_LM3S_SYSCTL_RCGC2_EMAC0;
    __DSB();

    /* set EPHY LED pins */
    FNET_LM3S_PORTF_BASE_PTR->DEN    |= (1 << 2) | (1 << 3);
    FNET_LM3S_PORTF_BASE_PTR->AFSEL  |= (1 << 2) | (1 << 3);

    /* Set the Ethernet management clock divider based on the system speed clock. */
    FNET_LM3S_MAC0_BASE_PTR->MDV      = (FNET_CFG_CPU_CLOCK_HZ / 2) / 2500000;
}
#endif /*FNET_CFG_CPU_ETH_IO_INIT*/



/************************************************************************
* NAME: fnet_eth_phy_init
*
* DESCRIPTION: Ethernet Physical Transceiver initialization and/or reset.
*************************************************************************/
void fnet_eth_phy_init(void)
{
    fnet_uint16_t reg_value;
    fnet_uint16_t status_value = 0;

    fnet_fec_mii_read(ethif, FNET_FEC_MII_REG_CR, &reg_value);

    /* ANE ENABLED:*/
    fnet_fec_mii_write(ethif, FNET_FEC_MII_REG_CR, (fnet_uint16_t)(reg_value | FNET_FEC_MII_REG_CR_ANE | FNET_FEC_MII_REG_CR_ANE_RESTART));

    while (status_value != 0x0040)
    {
        fnet_fec_mii_read(ethif, FNET_FEC_MII_REG_SR, &status_value);
        status_value &= 0x0040;
    }
}

#endif /* FNET_LM3S && FNET_CFG_CPU_ETH0 */