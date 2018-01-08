#include "fnet.h"
#if FNET_LM3S && FNET_CFG_CPU_ETH0

/************************************************************************
* Ethernet interface structure.
*************************************************************************/
static fnet_eth_if_t fnet_lm3s_eth0_if =
{
    .eth_prv = &fnet_lm3s_if,                       /* Points to Ethernet driver-specific control data structure. */
    .eth_mac_number = 0,                            /* MAC module number. */
    .eth_output = fnet_lm3s_output,                 /* Ethernet driver output.*/
};

fnet_netif_t fnet_cpu_eth0_if =
{
    .netif_name = FNET_CFG_CPU_ETH0_NAME,     /* Network interface name.*/
    .netif_mtu = FNET_CFG_CPU_ETH0_MTU,       /* Maximum transmission unit.*/
    .netif_prv = &fnet_lm3s_eth0_if,          /* Points to interface specific data structure.*/
    .netif_api = &fnet_lm3s_api               /* Interface API */
};

