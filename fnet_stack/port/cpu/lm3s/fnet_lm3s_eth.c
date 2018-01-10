#include "fnet.h"
#if FNET_LM3S && FNET_CFG_CPU_ETH0

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


static uint16_t read_mii(uint8_t reg)
{

    // Wait for any pending transaction to complete.
    while (0 != (MAC->MCTL & MAC_MCTL_START))
    {
    }

    // Program the PHY register address and initiate the transaction.
    MAC->MCTL = ((reg << MAC_MCTL_REGADR_S) & MAC_MCTL_REGADR_M) | MAC_MCTL_START;
    
    // Wait for the write transaction to complete.
    while (0 != (MAC->MCTL & MAC_MCTL_START))
    {
    }

    // Return the PHY data that was read.
    return (MAC->MRXD & MAC_MRXD_MDRX_M);
}


static void write_mii(uint8_t reg, uint16_t data)
{
    // Wait for any pending transaction to complete.
    while (0 != (MAC->MCTL & MAC_MCTL_START))
    {
    }

    // Program the DATA to be written.
    MAC->MTXD = data;
    
    // Program the PHY register address and initiate the transaction.
    MAC->MCTL = ((reg << MAC_MCTL_REGADR_S) & MAC_MCTL_REGADR_M) | MAC_MCTL_WRITE | MAC_MCTL_START;
    
    // Wait for the write transaction to complete.
    while (0 != (MAC->MCTL & MAC_MCTL_START))
    {
    }
}


/************************************************************************
* DESCRIPTION: Ethernet IO initialization.
*************************************************************************/
#if FNET_CFG_CPU_ETH_IO_INIT
void fnet_eth_io_init()
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
void fnet_eth_phy_init(fnet_fec_if_t *ethif)
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